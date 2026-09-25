//! ROS 2 side of the console, behind the `ros` cargo feature (via r2r, which
//! uses whatever RMW the sourced ROS install is configured for - just like
//! rclpy). One node, one background thread, doing what the Python version
//! spreads over five rclpy modules:
//!
//! - `rosclock.py`: ROS "now" shown next to the autopilot clock
//! - `lidar.py`: PointCloud2 summary + decimated sample for [v] and the map
//! - `camera.py`: up to two Image / CompressedImage feeds for [w]
//! - `mapfeeds.py`: nav_msgs/Path (planned path) and the /fire_gps_loc fix
//! - `gpspub.py`: [P] one-shot NavSatFix publish of the vehicle position
//!
//! Without the feature, [`spawn`] just records why ROS is unavailable and the
//! screens say so.

use std::sync::{Arc, Mutex};

use crate::config::Settings;

#[derive(Debug, Clone, Default)]
pub struct CameraSlot {
    /// Wanted topic ("" = slot unused); the thread re-subscribes on change.
    pub topic: String,
    /// "image" | "compressed" once resolved.
    pub kind: String,
    pub width: u32,
    pub height: u32,
    /// RGB24, width * height * 3.
    pub rgb: Vec<u8>,
    #[cfg_attr(not(feature = "ros"), allow(dead_code))]
    pub frame_count: u64,
    pub fps: f64,
    pub last_frame_at: f64,
    pub error: String,
    /// Non-fatal remark shown with a frame (e.g. a flat mono16 image).
    pub note: String,
}

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
    pub lidar_point_count: usize,
    pub lidar_rate_hz: f64,
    pub lidar_last: f64,
    /// Decimated sensor-frame sample, non-finite points dropped.
    pub lidar_points: Vec<[f64; 3]>,
    pub lidar_range: (f64, f64),

    pub navpath_topic: String,
    /// Local (north, east): the path is ENU in the map frame.
    pub navpath_points: Vec<(f64, f64)>,
    pub navpath_last: f64,
    pub navpath_publishers: Vec<String>,

    /// (lat, lon, alt) of the last /fire_gps_loc fix.
    pub fire: Option<(f64, f64, f64)>,
    pub fire_last: f64,
    pub fire_publishers: Vec<String>,

    pub cameras: [CameraSlot; 2],
    pub camera_low_bw: bool,

    pub gps_publish_requested: bool,
    pub gps_feedback: String,
    pub gps_feedback_ok: bool,
    pub gps_feedback_time: f64,
}

pub type Ros = Arc<Mutex<RosState>>;

pub fn new(settings: &Settings) -> Ros {
    let mut s = RosState {
        lidar_topic: settings.lidar_topic.clone(),
        navpath_topic: settings.navpath_topic.clone(),
        ..Default::default()
    };
    for (slot, topic) in s.cameras.iter_mut().zip(&settings.camera_topics) {
        slot.topic = topic.clone();
    }
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
    use r2r::sensor_msgs::msg::{CompressedImage, Image, NavSatFix, PointCloud2};

    use super::{CameraSlot, Ros, RosState};
    use crate::config::{GPS_PUBLISH_FRAME_ID, GPS_PUBLISH_TOPIC, Settings};
    use crate::host::lock;
    use crate::state::{self, Shared, now};

    /// Keep at most this many points of a scan for stats and the scatter.
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

    /// (point count, decimated finite xyz sample).
    fn decode_cloud(msg: &PointCloud2) -> (usize, Vec<[f64; 3]>) {
        let count = (msg.width * msg.height) as usize;
        let step = msg.point_step as usize;
        let field = |name: &str| {
            let f = msg.fields.iter().find(|f| f.name == name)?;
            let (width, read) = field_reader(f.datatype)?;
            Some((f.offset as usize, width, read))
        };
        let (Some(x), Some(y), Some(z)) = (field("x"), field("y"), field("z")) else { return (0, Vec::new()) };
        if count == 0 || step == 0 {
            return (0, Vec::new());
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
        (count, out)
    }

    /// "Ironbow" false-colour ramp for mono16 (thermal / depth) frames.
    fn thermal(v: u8) -> [u8; 3] {
        const STOPS: [(f64, [f64; 3]); 5] = [
            (0.0, [0.0, 0.0, 0.0]),
            (64.0, [60.0, 0.0, 110.0]),
            (128.0, [170.0, 20.0, 90.0]),
            (192.0, [255.0, 120.0, 20.0]),
            (255.0, [255.0, 255.0, 200.0]),
        ];
        let v = v as f64;
        let i = STOPS.iter().rposition(|s| s.0 <= v).unwrap_or(0).min(3);
        let (a, b) = (STOPS[i], STOPS[i + 1]);
        let t = (v - a.0) / (b.0 - a.0);
        [0, 1, 2].map(|c| (a.1[c] + (b.1[c] - a.1[c]) * t).round() as u8)
    }

    /// (rgb, note) or an error text, for a sensor_msgs/Image.
    fn decode_image(msg: &Image) -> Result<(Vec<u8>, String), String> {
        let (w, h, step) = (msg.width as usize, msg.height as usize, msg.step as usize);
        if w == 0 || h == 0 {
            return Err("empty frame".into());
        }
        if msg.data.len() < h * step {
            return Err("short frame".into());
        }
        let row = |y: usize| &msg.data[y * step..y * step + step];
        let mut rgb = Vec::with_capacity(w * h * 3);

        if msg.encoding == "mono16" {
            if step < w * 2 {
                return Err("row stride too small".into());
            }
            let values: Vec<f64> = (0..h)
                .flat_map(|y| {
                    let r = row(y);
                    (0..w).map(move |x| {
                        let b = [r[x * 2], r[x * 2 + 1]];
                        (if msg.is_bigendian != 0 { u16::from_be_bytes(b) } else { u16::from_le_bytes(b) }) as f64
                    })
                })
                .collect();
            // 1st / 99th percentile stretch: dead / hot pixels would
            // otherwise squash the whole frame into a few shades.
            let mut sorted = values.clone();
            sorted.sort_by(f64::total_cmp);
            let pct = |p: f64| sorted[((sorted.len() - 1) as f64 * p) as usize];
            let (mut lo, mut hi) = (pct(0.01), pct(0.99));
            if hi <= lo {
                (lo, hi) = (sorted[0], sorted[sorted.len() - 1]);
            }
            let mut note = String::new();
            for v in values {
                let s = if hi <= lo { 128 } else { ((v - lo) * 255.0 / (hi - lo)).clamp(0.0, 255.0) as u8 };
                rgb.extend(thermal(s));
            }
            if hi <= lo {
                note = format!("flat frame (every pixel = {lo:.0}) - source has no contrast to color by");
            }
            return Ok((rgb, note));
        }

        let (channels, order): (usize, [usize; 3]) = match msg.encoding.as_str() {
            "rgb8" => (3, [0, 1, 2]),
            "bgr8" => (3, [2, 1, 0]),
            "rgba8" => (4, [0, 1, 2]),
            "bgra8" => (4, [2, 1, 0]),
            "mono8" => (1, [0, 0, 0]),
            other => return Err(format!("unsupported encoding '{other}'")),
        };
        if step < w * channels {
            return Err("row stride too small".into());
        }
        for y in 0..h {
            let r = row(y);
            for x in 0..w {
                let px = &r[x * channels..];
                rgb.extend(order.map(|c| px[c]));
            }
        }
        Ok((rgb, String::new()))
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
        if use_sim_time {
            let ts = node.get_time_source();
            ts.enable_sim_time(&mut node).map_err(|e| e.to_string())?;
        }
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
        // (topic, stream) per slot; None = not set up yet.
        let mut camera_subs: [Option<(String, Option<AbortHandle>)>; 2] = [None, None];
        let lidar_count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let mut rate_base = (now(), 0u64, [0u64; 2]);
        let mut next_graph_poll = 0.0;

        while !shutdown.load(Ordering::Relaxed) {
            // --- (re)subscribe LiDAR / cameras when a topic changed.
            let (lidar_topic, camera_topics) = with(ros, |r| (r.lidar_topic.clone(), [r.cameras[0].topic.clone(), r.cameras[1].topic.clone()]));
            if lidar_sub.as_ref().map(|s| &s.0) != Some(&lidar_topic) {
                if let Some((_, h)) = lidar_sub.take() {
                    h.abort();
                }
                with(ros, |r| {
                    r.lidar_frame_id.clear();
                    r.lidar_point_count = 0;
                    r.lidar_rate_hz = 0.0;
                    r.lidar_last = 0.0;
                    r.lidar_points.clear();
                    r.lidar_range = (0.0, 0.0);
                });
                if !lidar_topic.is_empty() {
                    let (r, count) = (ros.clone(), lidar_count.clone());
                    match node.subscribe::<PointCloud2>(&lidar_topic, QosProfile::sensor_data()) {
                        Ok(s) => {
                            let h = spawn_stream(Box::pin(s.map(move |msg| {
                                count.fetch_add(1, Ordering::Relaxed);
                                let (n, pts) = decode_cloud(&msg);
                                let ranges = pts.iter().map(|p| (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt());
                                let range = ranges.fold((f64::MAX, 0.0f64), |(a, b), v| (a.min(v), b.max(v)));
                                with(&r, |r| {
                                    r.lidar_frame_id = msg.header.frame_id.clone();
                                    r.lidar_point_count = n;
                                    r.lidar_range = if pts.is_empty() { (0.0, 0.0) } else { range };
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

            for idx in 0..2 {
                let topic = &camera_topics[idx];
                if camera_subs[idx].as_ref().map(|s| &s.0) == Some(topic) {
                    continue;
                }
                if let Some((_, Some(h))) = camera_subs[idx].take() {
                    h.abort();
                }
                with(ros, |r| {
                    r.cameras[idx] = CameraSlot { topic: topic.clone(), ..Default::default() }
                });
                camera_subs[idx] = Some((topic.clone(), None));
                if topic.is_empty() {
                    continue;
                }
                // Resolve Image vs CompressedImage from the graph, else by the
                // "/compressed" naming convention.
                let types = node.get_topic_names_and_types().unwrap_or_default();
                let kind = match types.get(topic.as_str()) {
                    Some(t) if t.iter().any(|t| t.contains("CompressedImage")) => "compressed",
                    Some(t) if t.iter().any(|t| t.contains("/Image")) => "image",
                    _ if topic.ends_with("/compressed") => "compressed",
                    _ => "image",
                };
                with(ros, |r| r.cameras[idx].kind = kind.into());
                let r = ros.clone();
                let result = if kind == "compressed" {
                    node.subscribe::<CompressedImage>(topic, QosProfile::sensor_data()).map(|s| {
                        spawn_stream(Box::pin(s.map(move |msg| {
                            let decoded = image::load_from_memory(&msg.data).map(|i| i.to_rgb8());
                            with(&r, |r| {
                                let slot = &mut r.cameras[idx];
                                match decoded {
                                    Ok(img) => {
                                        slot.width = img.width();
                                        slot.height = img.height();
                                        slot.rgb = img.into_raw();
                                        slot.frame_count += 1;
                                        slot.last_frame_at = now();
                                        slot.error.clear();
                                    }
                                    Err(e) => slot.error = format!("couldn't decode: {e}"),
                                }
                            });
                        })))
                    })
                } else {
                    node.subscribe::<Image>(topic, QosProfile::sensor_data()).map(|s| {
                        spawn_stream(Box::pin(s.map(move |msg| {
                            let decoded = decode_image(&msg);
                            with(&r, |r| {
                                let slot = &mut r.cameras[idx];
                                match decoded {
                                    Ok((rgb, note)) => {
                                        slot.width = msg.width;
                                        slot.height = msg.height;
                                        slot.rgb = rgb;
                                        slot.frame_count += 1;
                                        slot.last_frame_at = now();
                                        slot.error.clear();
                                        slot.note = note;
                                    }
                                    Err(e) => {
                                        slot.error = e;
                                        slot.note.clear();
                                    }
                                }
                            });
                        })))
                    })
                };
                match result {
                    Ok(h) => camera_subs[idx] = Some((topic.clone(), Some(h))),
                    Err(e) => with(ros, |r| r.cameras[idx].error = format!("subscribe failed: {e}")),
                }
            }

            node.spin_once(Duration::from_millis(100));
            pool.run_until_stalled();

            // --- clock
            if let Ok(t) = lock(&clock).get_now() {
                with(ros, |r| {
                    r.sim_time = use_sim_time;
                    if !t.is_zero() {
                        r.time_ns = t.as_nanos() as i64;
                        r.last_time = now();
                    }
                });
            }

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
                    if let Ok(t) = lock(&clock).get_now() {
                        msg.header.stamp = r2r::Clock::to_builtin_time(&t);
                    }
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

            // --- once a second: rates and the publisher lists.
            let t = now();
            if t >= next_graph_poll {
                next_graph_poll = t + 1.0;
                let fire = publishers(&node, GPS_PUBLISH_TOPIC);
                let nav = publishers(&node, &navpath_topic);
                let dt = (t - rate_base.0).max(1e-3);
                let lc = lidar_count.load(Ordering::Relaxed);
                with(ros, |r| {
                    r.fire_publishers = fire;
                    r.navpath_publishers = nav;
                    r.lidar_rate_hz = (lc - rate_base.1) as f64 / dt;
                    for (i, slot) in r.cameras.iter_mut().enumerate() {
                        slot.fps = slot.frame_count.saturating_sub(rate_base.2[i]) as f64 / dt;
                        rate_base.2[i] = slot.frame_count;
                    }
                });
                rate_base.0 = t;
                rate_base.1 = lc;
            }
        }
        Ok(())
    }
}
