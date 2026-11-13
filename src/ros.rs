//! ROS 2 side of the console, behind the `ros` cargo feature (via r2r, which
//! uses whatever RMW the sourced ROS install is configured for - just like
//! rclpy): the ROS "now" shown next to the autopilot clock on the dashboard.
//! The [w] ROS pane (rosout, publishes, service calls) has its own node in
//! `rospane.rs`.
//!
//! Without the feature, [`spawn`] just records why ROS is unavailable and the
//! dashboard says so.

use std::sync::{Arc, Mutex};

use crate::config::Settings;

#[derive(Debug, Default)]
pub struct RosState {
    /// The node is up.
    pub available: bool,
    /// Why not, when not.
    pub error: String,

    pub time_ns: i64,
    pub sim_time: bool,
    pub last_time: f64,
}

pub type Ros = Arc<Mutex<RosState>>;

pub fn new(_settings: &Settings) -> Ros {
    Arc::default()
}

#[cfg(not(feature = "ros"))]
pub fn spawn(
    ros: Ros,
    _shared: crate::state::Shared,
    _settings: &Settings,
    _shutdown: Arc<std::sync::atomic::AtomicBool>,
) -> Option<std::thread::JoinHandle<()>> {
    crate::host::lock(&ros).error = "built without ROS 2 support (rebuild with --features ros)".into();
    None
}

#[cfg(feature = "ros")]
pub use imp::spawn;

#[cfg(feature = "ros")]
mod imp {
    use std::sync::Arc;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::time::Duration;

    use futures::StreamExt;
    use futures::task::LocalSpawnExt;
    use r2r::QosProfile;
    use r2r::rosgraph_msgs::msg::Clock;

    use super::{Ros, RosState};
    use crate::config::Settings;
    use crate::host::lock;
    use crate::state::{self, Shared, now};

    /// The node's thread; join it on exit so rcl tears down in order
    /// (otherwise rmw prints teardown errors after the TUI has closed).
    pub fn spawn(ros: Ros, shared: Shared, settings: &Settings, shutdown: Arc<AtomicBool>) -> Option<std::thread::JoinHandle<()>> {
        let use_sim_time = settings.use_sim_time;
        let handle = std::thread::Builder::new()
            .name("RosThread".into())
            .spawn(move || {
                if let Err(e) = run(&ros, use_sim_time, &shutdown) {
                    let mut r = lock(&ros);
                    r.available = false;
                    r.error = format!("ROS 2 unavailable: {e}");
                    state::lock(&shared).warn(format!("ROS 2 unavailable: {e}"));
                }
            })
            .expect("spawn ROS thread");
        Some(handle)
    }

    fn with<R>(ros: &Ros, f: impl FnOnce(&mut RosState) -> R) -> R {
        f(&mut lock(ros))
    }

    fn run(ros: &Ros, use_sim_time: bool, shutdown: &AtomicBool) -> Result<(), String> {
        let ctx = r2r::Context::create().map_err(|e| e.to_string())?;
        let mut node = r2r::Node::create(ctx, "lazypx4", "").map_err(|e| e.to_string())?;
        let clock = node.get_ros_clock();

        let mut pool = futures::executor::LocalPool::new();
        let spawner = pool.spawner();

        // --use-sim-time: follow /clock ourselves. r2r's TimeSource sits
        // behind a cfg flag its build script drops when it reuses cached
        // bindings, so relying on it breaks builds; Clock itself is always
        // generated. Best-effort matches reliable and best-effort publishers.
        let sim_ns = Arc::new(std::sync::atomic::AtomicI64::new(0));
        if use_sim_time {
            let ns = sim_ns.clone();
            let s = node
                .subscribe::<Clock>("/clock", QosProfile::default().best_effort())
                .map_err(|e| e.to_string())?;
            spawner
                .spawn_local(s.for_each(move |msg| {
                    ns.store(msg.clock.sec as i64 * 1_000_000_000 + msg.clock.nanosec as i64, Ordering::Relaxed);
                    futures::future::ready(())
                }))
                .map_err(|e| e.to_string())?;
        }

        with(ros, |r| {
            r.available = true;
            r.error.clear();
        });

        while !shutdown.load(Ordering::Relaxed) {
            node.spin_once(Duration::from_millis(100));
            pool.run_until_stalled();

            // ROS "now" in ns (0 = not known yet).
            let t_ns = if use_sim_time {
                sim_ns.load(Ordering::Relaxed)
            } else {
                lock(&clock).get_now().map(|t| t.as_nanos() as i64).unwrap_or(0)
            };
            with(ros, |r| {
                r.sim_time = use_sim_time;
                if t_ns > 0 {
                    r.time_ns = t_ns;
                    r.last_time = now();
                }
            });
        }
        Ok(())
    }
}
