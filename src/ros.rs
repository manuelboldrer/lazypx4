//! ROS 2 side of the console, behind the `ros` cargo feature (via r2r, which
//! uses whatever RMW the sourced ROS install is configured for - just like
//! rclpy). One node, one background thread, doing what the Python version
//! spreads over five rclpy modules:
//!
//! - `rosclock.py`: ROS "now" shown next to the autopilot clock
//! - `lidar.py`: decimated PointCloud2 sample for the map's LiDAR overlay
//! - `mapfeeds.py`: nav_msgs/Path (planned path) and the /fire_gps_loc fix
//! - `gpspub.py`: [P] one-shot NavSatFix publish of the vehicle position
//!
//! Without the feature, [`spawn`] just records why ROS is unavailable and the
//! screens say so.

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

    pub lidar_topic: String,
    pub lidar_frame_id: String,
    pub lidar_last: f64,
    /// Decimated sensor-frame sample, non-finite points dropped.
    pub lidar_points: Vec<[f64; 3]>,

    pub navpath_topic: String,
    /// Local (north, east): the path is ENU in the map frame.
    pub navpath_points: Vec<(f64, f64)>,
    pub navpath_last: f64,
    pub navpath_publishers: Vec<String>,

    /// (lat, lon, alt) of the last /fire_gps_loc fix.
    pub fire: Option<(f64, f64, f64)>,
    pub fire_last: f64,
    pub fire_publishers: Vec<String>,

    pub gps_publish_requested: bool,
    pub gps_feedback: String,
    pub gps_feedback_ok: bool,
    pub gps_feedback_time: f64,
}

pub type Ros = Arc<Mutex<RosState>>;

pub fn new(settings: &Settings) -> Ros {
    let s = RosState {
        lidar_topic: settings.lidar_topic.clone(),
        navpath_topic: settings.navpath_topic.clone(),
        ..Default::default()
    };
    Arc::new(Mutex::new(s))
}

pub fn set_feedback(ros: &Ros, message: impl Into<String>, ok: bool) {
    let mut r = crate::host::lock(ros);
    r.gps_feedback = message.into();
    r.gps_feedback_ok = ok;
    r.gps_feedback_time = crate::state::now();
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
    use futures::future::{AbortHandle, Abortable};
    use futures::task::LocalSpawnExt;
    use r2r::QosProfile;
    use r2r::nav_msgs::msg::Path;
    use r2r::rosgraph_msgs::msg::Clock;
    use r2r::sensor_msgs::msg::{NavSatFix, PointCloud2};

    use super::{Ros, RosState};
    use crate::config::{GPS_PUBLISH_FRAME_ID, GPS_PUBLISH_TOPIC, Settings};
    use crate::host::lock;
    use crate::state::{self, Shared, now};

    /// Keep at most this many points of a scan for the map overlay.
    const MAX_SCATTER_POINTS: usize = 3000;

    /// The node's thread; join it on exit so rcl tears down in order
    /// (otherwise rmw prints teardown errors after the TUI has closed).
    pub fn spawn(ros: Ros, shared: Shared, settings: &Settings, shutdown: Arc<AtomicBool>) -> Option<std::thread::JoinHandle<()>> {
        let use_sim_time = settings.use_sim_time;
        let handle = std::thread::Builder::new()
            .name("RosThread".into())
            .spawn(move || {
                if let Err(e) = run(&ros, &shared, use_sim_time, &shutdown) {
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

    // --- decoding -------------------------------------------------------

    /// Reads one field value from its bytes (bool = big-endian).
    type Reader = fn(&[u8], bool) -> f64;

    /// (byte width, reader) for a PointField datatype.
    fn field_reader(datatype: u8) -> Option<(usize, Reader)> {
        fn rd<const N: usize>(b: &[u8], be: bool) -> [u8; N] {
            let mut a = [0u8; N];
            a.copy_from_slice(&b[..N]);
            if be {
                a.reverse();
            }
            a
        }
        Some(match datatype {
            1 => (1, |b, _| b[0] as i8 as f64),
            2 => (1, |b, _| b[0] as f64),
            3 => (2, |b, be| i16::from_le_bytes(rd(b, be)) as f64),
            4 => (2, |b, be| u16::from_le_bytes(rd(b, be)) as f64),
            5 => (4, |b, be| i32::from_le_bytes(rd(b, be)) as f64),
            6 => (4, |b, be| u32::from_le_bytes(rd(b, be)) as f64),
            7 => (4, |b, be| f32::from_le_bytes(rd(b, be)) as f64),
            8 => (8, |b, be| f64::from_le_bytes(rd(b, be))),
            _ => return None,
        })
    }

    /// Decimated finite xyz sample.
    fn decode_cloud(msg: &PointCloud2) -> Vec<[f64; 3]> {
        let count = (msg.width * msg.height) as usize;
        let step = msg.point_step as usize;
        let field = |name: &str| {
            let f = msg.fields.iter().find(|f| f.name == name)?;
            let (width, read) = field_reader(f.datatype)?;
            Some((f.offset as usize, width, read))
        };
        let (Some(x), Some(y), Some(z)) = (field("x"), field("y"), field("z")) else { return Vec::new() };
        if count == 0 || step == 0 {
            return Vec::new();
        }
        let stride = (count / MAX_SCATTER_POINTS).max(1);
        let be = msg.is_bigendian;
        let mut out = Vec::with_capacity(count / stride + 1);
        for i in (0..count).step_by(stride) {
            let base = i * step;
            let get = |(off, w, read): (usize, usize, Reader)| {
                msg.data.get(base + off..base + off + w).map(|b| read(b, be))
            };
            if let (Some(px), Some(py), Some(pz)) = (get(x), get(y), get(z))
                && px.is_finite()
                && py.is_finite()
                && pz.is_finite()
            {
                out.push([px, py, pz]);
            }
        }
        out
    }

    // --- node -----------------------------------------------------------

    fn publishers(node: &r2r::Node, topic: &str) -> Vec<String> {
        let mut out: Vec<String> = node
            .get_publishers_info_by_topic(topic, false)
            .unwrap_or_default()
            .into_iter()
            .filter(|i| i.node_name != "lazypx4") // our own [P] publisher
            .map(|i| {
                let mut qos = if i.qos_profile.reliability == r2r::qos::ReliabilityPolicy::BestEffort {
                    "best-effort".to_string()
                } else {
                    "reliable".to_string()
                };
                if i.qos_profile.durability == r2r::qos::DurabilityPolicy::TransientLocal {
                    qos += ", latched";
                }
                format!("{} ({qos})", i.node_name)
            })
            .collect();
        out.sort();
        out
    }

    fn run(ros: &Ros, shared: &Shared, use_sim_time: bool, shutdown: &AtomicBool) -> Result<(), String> {
        let ctx = r2r::Context::create().map_err(|e| e.to_string())?;
        let mut node = r2r::Node::create(ctx, "lazypx4", "").map_err(|e| e.to_string())?;
        let clock = node.get_ros_clock();

        let mut pool = futures::executor::LocalPool::new();
        let spawner = pool.spawner();

        // Spawn a stream consumer that can be cancelled when its topic
        // changes; dropping the stream makes r2r destroy the subscription.
        let spawn_stream = |stream: std::pin::Pin<Box<dyn futures::Stream<Item = ()>>>| -> AbortHandle {
            let (handle, reg) = AbortHandle::new_pair();
            let _ = spawner.spawn_local(async move {
                let _ = Abortable::new(stream.for_each(|_| async {}), reg).await;
            });
            handle
        };

        // --- --use-sim-time: follow /clock ourselves. r2r's TimeSource sits
        // behind a cfg flag its build script drops when it reuses cached
        // bindings, so relying on it breaks builds; Clock itself is always
        // generated. Best-effort matches reliable and best-effort publishers.
        let sim_ns = Arc::new(std::sync::atomic::AtomicI64::new(0));
        if use_sim_time {
            let ns = sim_ns.clone();
            let s = node
                .subscribe::<Clock>("/clock", QosProfile::default().best_effort())
                .map_err(|e| e.to_string())?;
            spawn_stream(Box::pin(s.map(move |msg| {
                ns.store(msg.clock.sec as i64 * 1_000_000_000 + msg.clock.nanosec as i64, Ordering::Relaxed);
            })));
        }
        // ROS "now" in ns (0 = not known yet).
        let ros_now_ns = || -> i64 {
            if use_sim_time {
                sim_ns.load(Ordering::Relaxed)
            } else {
                lock(&clock).get_now().map(|t| t.as_nanos() as i64).unwrap_or(0)
            }
        };

        // --- map feeds: path (volatile reliable / latched / best-effort)
        // and fire fix (reliable / best-effort), so publishers of any QoS
        // are matched.
        let navpath_topic = with(ros, |r| r.navpath_topic.clone());
        let path_qos = [
            QosProfile::default(),
            QosProfile::default().reliable().transient_local().keep_last(1),
            QosProfile::default().best_effort(),
        ];
        for qos in path_qos {
            let r = ros.clone();
            let s = node.subscribe::<Path>(&navpath_topic, qos).map_err(|e| e.to_string())?;
            spawn_stream(Box::pin(s.map(move |msg| {
                let mut pts: Vec<(f64, f64)> = Vec::new();
                for p in &msg.poses {
                    // ENU map frame -> (north, east); drop repeated poses.
                    let pt = (p.pose.position.y, p.pose.position.x);
                    if pts.last() != Some(&pt) {
                        pts.push(pt);
                    }
                }
                with(&r, |r| {
                    r.navpath_points = pts;
                    r.navpath_last = now();
                });
            })));
        }
        for qos in [QosProfile::default(), QosProfile::default().best_effort()] {
            let r = ros.clone();
            let s = node.subscribe::<NavSatFix>(GPS_PUBLISH_TOPIC, qos).map_err(|e| e.to_string())?;
            spawn_stream(Box::pin(s.map(move |msg| {
                with(&r, |r| {
                    r.fire = Some((msg.latitude, msg.longitude, msg.altitude));
                    r.fire_last = now();
                });
            })));
        }

        let gps_pub = node
            .create_publisher::<NavSatFix>(GPS_PUBLISH_TOPIC, QosProfile::default())
            .map_err(|e| e.to_string())?;

        with(ros, |r| {
            r.available = true;
            r.error.clear();
        });

        let mut lidar_sub: Option<(String, AbortHandle)> = None;
        let mut next_graph_poll = 0.0;

        while !shutdown.load(Ordering::Relaxed) {
            // --- (re)subscribe LiDAR when the topic changed.
            let lidar_topic = with(ros, |r| r.lidar_topic.clone());
            if lidar_sub.as_ref().map(|s| &s.0) != Some(&lidar_topic) {
                if let Some((_, h)) = lidar_sub.take() {
                    h.abort();
                }
                with(ros, |r| {
                    r.lidar_frame_id.clear();
                    r.lidar_last = 0.0;
                    r.lidar_points.clear();
                });
                if !lidar_topic.is_empty() {
                    let r = ros.clone();
                    match node.subscribe::<PointCloud2>(&lidar_topic, QosProfile::sensor_data()) {
                        Ok(s) => {
                            let h = spawn_stream(Box::pin(s.map(move |msg| {
                                let pts = decode_cloud(&msg);
                                with(&r, |r| {
                                    r.lidar_frame_id = msg.header.frame_id.clone();
                                    r.lidar_points = pts;
                                    r.lidar_last = now();
                                });
                            })));
                            lidar_sub = Some((lidar_topic.clone(), h));
                        }
                        Err(e) => state::lock(shared).warn(format!("LiDAR subscribe to {lidar_topic} failed: {e}")),
                    }
                }
            }

            node.spin_once(Duration::from_millis(100));
            pool.run_until_stalled();

            // --- clock
            let t_ns = ros_now_ns();
            with(ros, |r| {
                r.sim_time = use_sim_time;
                if t_ns > 0 {
                    r.time_ns = t_ns;
                    r.last_time = now();
                }
            });

            // --- [P] publish request
            if with(ros, |r| std::mem::take(&mut r.gps_publish_requested)) {
                let (valid, lat, lon, alt) = {
                    let st = state::lock(shared);
                    (st.global_pos_valid, st.global_lat, st.global_lon, st.global_alt)
                };
                if !valid {
                    super::set_feedback(ros, "GPS publish: lost the global position before it could be sent", false);
                } else {
                    let mut msg = NavSatFix::default();
                    let t_ns = ros_now_ns();
                    msg.header.stamp.sec = (t_ns / 1_000_000_000) as i32;
                    msg.header.stamp.nanosec = (t_ns % 1_000_000_000) as u32;
                    msg.header.frame_id = GPS_PUBLISH_FRAME_ID.into();
                    msg.status.status = 0; // STATUS_FIX
                    msg.status.service = 1; // SERVICE_GPS
                    msg.latitude = lat;
                    msg.longitude = lon;
                    msg.altitude = alt;
                    msg.position_covariance = vec![0.0; 9];
                    msg.position_covariance_type = 0;
                    match gps_pub.publish(&msg) {
                        Ok(()) => {
                            let text = format!("published to {GPS_PUBLISH_TOPIC}: {lat:.7}, {lon:.7} @ {alt:.2} m");
                            state::lock(shared).command(format!("GPS position {text}"));
                            super::set_feedback(ros, text, true);
                        }
                        Err(e) => super::set_feedback(ros, format!("GPS publish failed: {e}"), false),
                    }
                }
            }

            // --- once a second: the publisher lists.
            let t = now();
            if t >= next_graph_poll {
                next_graph_poll = t + 1.0;
                let fire = publishers(&node, GPS_PUBLISH_TOPIC);
                let nav = publishers(&node, &navpath_topic);
                with(ros, |r| {
                    r.fire_publishers = fire;
                    r.navpath_publishers = nav;
                });
            }
        }
        Ok(())
    }
}
