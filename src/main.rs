//! lazypx4: a lazygit-style terminal UI for PX4 over MAVLink.
//!
//! Command-line entry point:
//! parse args, then run the app on the alternate screen.

mod app;
mod app_cmd;
mod app_map;
mod cmdline;
mod config;
mod fleet;
mod geo;
mod host;
mod jobs;
mod lineedit;
mod link;
mod mav;
mod parammeta;
mod preflight;
mod px4mode;
mod ros;
mod tlog;
mod satellite;
mod state;
mod suspend;
mod ui;

use std::io::IsTerminal;

use clap::Parser;

#[derive(Parser, Debug)]
#[command(
    name = "lazypx4",
    version,
    about = "A lazygit-style terminal UI for PX4 over MAVLink.",
    after_help = "Settings can also come from lazypx4.toml (--config, else ./lazypx4.toml, else the one in the \
                  repo lazypx4 was built from). Command-line flags override the file."
)]
struct Cli {
    /// Settings file [default: ./lazypx4.toml, else the repo's]
    #[arg(long, value_name = "FILE")]
    config: Option<String>,

    /// UDP port to listen for the PX4 MAVLink connection on [default: 14560]
    #[arg(short, long, value_name = "N")]
    port: Option<u16>,

    /// Directory for downloaded .ulg flight logs [default: ./px4_logs]
    #[arg(long, value_name = "DIR")]
    log_dir: Option<String>,

    /// PX4 parameters.json (from the firmware build) - enables the "changed
    /// from default" parameter view and replaces the bundled parameter
    /// descriptions / enum meanings
    #[arg(long, value_name = "FILE")]
    param_defaults: Option<String>,

    /// Directory for parameter files saved / loaded on the parameter screen
    /// [default: ./px4_params]
    #[arg(long, value_name = "DIR")]
    param_dir: Option<String>,

    /// Permit starting a flight-log download while the vehicle is armed
    #[arg(long)]
    allow_log_download_while_armed: bool,

    /// Directory for satellite-image snapshots [default: ./px4_maps]
    #[arg(long, value_name = "DIR")]
    map_dir: Option<String>,

    /// Filesystem whose usage the dashboard HOST line shows [default: /]
    #[arg(long, value_name = "PATH")]
    disk_path: Option<String>,

    /// Directory scanned for .px4 firmware files on the flash-firmware screen
    /// [default: ./px4_firmware]
    #[arg(long, value_name = "DIR")]
    firmware_dir: Option<String>,

    /// Directory containing px_uploader.py / upload_log.py / ecl_ekf
    /// [default: ./Tools, else the Tools/ of the repo it was built from]
    #[arg(long, value_name = "DIR")]
    tools_dir: Option<String>,

    /// Load a .kml file as the map's fence/waypoint overlay on start-up
    #[arg(long, value_name = "FILE")]
    kml: Option<String>,

    /// ROS 2 sensor_msgs/PointCloud2 topic for the map's [V] LiDAR overlay
    /// [default: /livox/points]
    #[arg(long, value_name = "TOPIC")]
    lidar_topic: Option<String>,

    /// ROS 2 nav_msgs/Path topic (ENU map frame) for the map's [N] overlay
    /// [default: /navsat_utm_path]
    #[arg(long, value_name = "TOPIC")]
    navpath_topic: Option<String>,

    /// Use /clock (simulation time) for the ROS clock
    #[arg(long)]
    use_sim_time: bool,

    /// Directory for the session .tlog [default: ./px4_tlogs]
    #[arg(long, value_name = "DIR")]
    tlog_dir: Option<String>,

    /// Don't record the session to a .tlog
    #[arg(long)]
    no_tlog: bool,

    /// Replay a .tlog instead of listening for a vehicle (nothing is sent)
    #[arg(long, value_name = "FILE")]
    replay: Option<String>,

    /// Fleet overview: one row per vehicle port, e.g. 14560-14563 or
    /// 14560,14570 (no list: [fleet] ports from lazypx4.toml)
    #[arg(long, value_name = "PORTS", num_args = 0..=1, default_missing_value = "")]
    fleet: Option<String>,

    /// Replay speed multiplier
    #[arg(long, default_value_t = 1.0, value_name = "X")]
    replay_speed: f64,
}

/// `./Tools` when started from the repo root, else the checkout the binary
/// was built from (so an installed binary finds the vendored PX4 scripts
/// from any directory).
fn default_tools_dir() -> String {
    let built_from = concat!(env!("CARGO_MANIFEST_DIR"), "/Tools");
    if !std::path::Path::new("./Tools").is_dir() && std::path::Path::new(built_from).is_dir() {
        built_from.into()
    } else {
        "./Tools".into()
    }
}

fn main() -> std::process::ExitCode {
    let cli = Cli::parse();

    if !std::io::stdin().is_terminal() {
        eprintln!("lazypx4 needs an interactive terminal to run.");
        return std::process::ExitCode::FAILURE;
    }

    let (file, config_path) = match config::load_file_config(cli.config.as_deref()) {
        Ok(loaded) => loaded,
        Err(e) => {
            eprintln!("lazypx4: bad config file {e}");
            return std::process::ExitCode::FAILURE;
        }
    };
    let key_params = match &file.key_params {
        Some(table) => match config::key_params_from_table(table) {
            Ok(columns) => columns,
            Err(e) => {
                eprintln!("lazypx4: bad config file: {e}");
                return std::process::ExitCode::FAILURE;
            }
        },
        None => config::default_key_params(),
    };
    let fleet_ports = match &cli.fleet {
        None => None,
        Some(list) => {
            let text = if list.is_empty() {
                file.fleet.ports.iter().map(u16::to_string).collect::<Vec<_>>().join(",")
            } else {
                list.clone()
            };
            match fleet::parse_ports(&text) {
                Ok(ports) => Some(ports),
                Err(e) => {
                    eprintln!("lazypx4: {e}");
                    return std::process::ExitCode::FAILURE;
                }
            }
        }
    };
    let record_tlog = !cli.no_tlog && cli.replay.is_none() && file.link.record_tlog.unwrap_or(true);
    let (paths, ros, safety) = (file.paths, file.ros, file.safety);
    let pick = |flag: Option<String>, from_file: Option<String>, default: &str| {
        flag.or(from_file).unwrap_or_else(|| default.into())
    };

    let settings = config::Settings {
        port: cli.port.or(file.link.port).unwrap_or(14560),
        log_dir: pick(cli.log_dir, paths.log_dir, "./px4_logs"),
        param_defaults_file: cli.param_defaults.or(paths.param_defaults),
        param_dir: pick(cli.param_dir, paths.param_dir, "./px4_params"),
        allow_log_download_while_armed: cli.allow_log_download_while_armed
            || safety.allow_log_download_while_armed.unwrap_or(false),
        map_dir: pick(cli.map_dir, paths.map_dir, "./px4_maps"),
        disk_path: pick(cli.disk_path, paths.disk_path, "/"),
        firmware_dir: pick(cli.firmware_dir, paths.firmware_dir, "./px4_firmware"),
        tools_dir: cli.tools_dir.or(paths.tools_dir).unwrap_or_else(default_tools_dir),
        ulog_upload_server: "https://logs.px4.io".into(),
        kml_path: cli.kml.or(paths.kml),
        lidar_topic: pick(cli.lidar_topic, ros.lidar_topic, "/livox/points"),
        navpath_topic: pick(cli.navpath_topic, ros.navpath_topic, "/navsat_utm_path"),
        use_sim_time: cli.use_sim_time || ros.use_sim_time.unwrap_or(false),
        battery_low: safety.battery_low.unwrap_or(config::DEFAULT_BATTERY_LOW),
        battery_critical: safety.battery_critical.unwrap_or(config::DEFAULT_BATTERY_CRITICAL),
        tlog_dir: record_tlog.then(|| pick(cli.tlog_dir, paths.tlog_dir, "./px4_tlogs")),
        replay: cli.replay.map(|path| (path, cli.replay_speed)),
        alert_bell: file.alerts.bell.unwrap_or(true),
        key_params,
        keys: file.keys,
        aliases: file.aliases,
        config_path,
    };
    if let Some(ports) = fleet_ports {
        let mut terminal = ratatui::init();
        let result = fleet::run(&mut terminal, &settings, &ports, settings.config_path.as_deref());
        ratatui::restore();
        return match result {
            Ok(()) => std::process::ExitCode::SUCCESS,
            Err(e) => {
                eprintln!("lazypx4: {e}");
                std::process::ExitCode::FAILURE
            }
        };
    }

    let port = settings.port;
    let replay = settings.replay.clone();

    let app = match app::App::new(settings) {
        Ok(app) => app,
        Err(e) => {
            match replay {
                Some((path, _)) => eprintln!("lazypx4: could not open {path}: {e}"),
                None => eprintln!("lazypx4: could not listen on UDP port {port}: {e}"),
            }
            return std::process::ExitCode::FAILURE;
        }
    };

    // ratatui::init() installs a panic hook that restores the terminal.
    let mut terminal = ratatui::init();
    let result = app.run(&mut terminal);
    ratatui::restore();

    match result {
        Ok(()) => std::process::ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("lazypx4: {e}");
            std::process::ExitCode::FAILURE
        }
    }
}
