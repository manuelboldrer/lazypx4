//! lazypx4-rs: a lazygit-style terminal UI for PX4 over MAVLink.
//!
//! Rust / ratatui port of the Python `lazypx4`. Command-line entry point:
//! parse args, then run the app on the alternate screen.

mod app;
mod app_map;
mod config;
mod geo;
mod host;
mod jobs;
mod lineedit;
mod link;
mod mav;
mod parammeta;
mod px4mode;
mod ros;
mod satellite;
mod state;
mod ui;

use std::io::IsTerminal;

use clap::Parser;

#[derive(Parser, Debug)]
#[command(name = "lazypx4", version, about = "A lazygit-style terminal UI for PX4 over MAVLink.")]
struct Cli {
    /// UDP port to listen for the PX4 MAVLink connection on
    #[arg(short, long, default_value_t = 14560, value_name = "N")]
    port: u16,

    /// Directory for downloaded .ulg flight logs
    #[arg(long, default_value = "./px4_logs", value_name = "DIR")]
    log_dir: String,

    /// PX4 parameters.json (from the firmware build) - enables the "changed
    /// from default" parameter view and replaces the bundled parameter
    /// descriptions / enum meanings
    #[arg(long, value_name = "FILE")]
    param_defaults: Option<String>,

    /// Permit starting a flight-log download while the vehicle is armed
    #[arg(long)]
    allow_log_download_while_armed: bool,

    /// Directory for satellite-image snapshots
    #[arg(long, default_value = "./px4_maps", value_name = "DIR")]
    map_dir: String,

    /// Filesystem whose usage the dashboard HOST line shows
    #[arg(long, default_value = "/", value_name = "PATH")]
    disk_path: String,

    /// Directory scanned for .px4 firmware files on the flash-firmware screen
    #[arg(long, default_value = "./px4_firmware", value_name = "DIR")]
    firmware_dir: String,

    /// Directory containing px_uploader.py / upload_log.py / ecl_ekf
    #[arg(long, default_value = "./Tools", value_name = "DIR")]
    tools_dir: String,

    /// Load a .kml file as the map's fence/waypoint overlay on start-up
    #[arg(long, value_name = "FILE")]
    kml: Option<String>,

    /// ROS 2 sensor_msgs/PointCloud2 topic for the [v] screen
    #[arg(long, default_value = "/livox/points", value_name = "TOPIC")]
    lidar_topic: String,

    /// ROS 2 nav_msgs/Path topic (ENU map frame) for the map's [N] overlay
    #[arg(long, default_value = "/navsat_utm_path", value_name = "TOPIC")]
    navpath_topic: String,

    /// ROS 2 Image / CompressedImage topic for the [w] screen's first slot
    #[arg(long, default_value = "/camera/image_raw", value_name = "TOPIC")]
    camera_topic: String,

    /// A second camera topic, shown alongside the first
    #[arg(long, default_value = "", value_name = "TOPIC")]
    camera_topic_2: String,

    /// Use /clock (simulation time) for the ROS clock
    #[arg(long)]
    use_sim_time: bool,
}

fn main() -> std::process::ExitCode {
    let cli = Cli::parse();

    if !std::io::stdin().is_terminal() {
        eprintln!("lazypx4 needs an interactive terminal to run.");
        return std::process::ExitCode::FAILURE;
    }

    let settings = config::Settings {
        port: cli.port,
        log_dir: cli.log_dir,
        param_defaults_file: cli.param_defaults,
        allow_log_download_while_armed: cli.allow_log_download_while_armed,
        map_dir: cli.map_dir,
        disk_path: cli.disk_path,
        firmware_dir: cli.firmware_dir,
        tools_dir: cli.tools_dir,
        ulog_upload_server: "https://logs.px4.io".into(),
        kml_path: cli.kml,
        lidar_topic: cli.lidar_topic,
        navpath_topic: cli.navpath_topic,
        camera_topics: [cli.camera_topic, cli.camera_topic_2],
        use_sim_time: cli.use_sim_time,
    };

    let app = match app::App::new(settings) {
        Ok(app) => app,
        Err(e) => {
            eprintln!("lazypx4: could not listen on UDP port {}: {e}", cli.port);
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
