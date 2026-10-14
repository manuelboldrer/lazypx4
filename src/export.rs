//! `:export` - position data out to CSV, one file per message / topic, in
//! `<log>_csv/` next to the log.
//!
//! - `.ulg` (PX4's own flight log): pyulog's `ulog2csv`, as a background
//!   job. This is where external vision / mocap lands at full rate, since
//!   it reaches PX4 on its own port and never passes through lazypx4.
//! - `.tlog` (what lazypx4 heard): decoded here.

use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::io::Write as _;
use std::path::{Path, PathBuf};

use mavlink::dialects::development::MavMessage;

/// `<dir>/<stem>_csv` for `<dir>/<stem>.<ext>`.
pub fn out_dir(log: &Path) -> PathBuf {
    let stem = log.file_stem().unwrap_or_default().to_string_lossy();
    log.with_file_name(format!("{stem}_csv"))
}

/// Header and one row of a position-carrying message; None for the rest.
fn row(msg: &MavMessage) -> Option<(&'static str, &'static str, String)> {
    let mut r = String::new();
    let name = match msg {
        MavMessage::LOCAL_POSITION_NED(d) => {
            let _ = write!(r, "{},{},{},{},{},{},{}", d.time_boot_ms, d.x, d.y, d.z, d.vx, d.vy, d.vz);
            ("LOCAL_POSITION_NED", "time_boot_ms,x,y,z,vx,vy,vz")
        }
        MavMessage::ODOMETRY(d) => {
            let _ = write!(
                r,
                "{},{:?},{:?},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{:?},{}",
                d.time_usec,
                d.frame_id,
                d.child_frame_id,
                d.x,
                d.y,
                d.z,
                d.q[0],
                d.q[1],
                d.q[2],
                d.q[3],
                d.vx,
                d.vy,
                d.vz,
                d.rollspeed,
                d.pitchspeed,
                d.yawspeed,
                d.reset_counter,
                d.estimator_type,
                d.quality
            );
            (
                "ODOMETRY",
                "time_usec,frame_id,child_frame_id,x,y,z,q0,q1,q2,q3,vx,vy,vz,rollspeed,pitchspeed,yawspeed,reset_counter,estimator_type,quality",
            )
        }
        MavMessage::ATT_POS_MOCAP(d) => {
            let _ = write!(r, "{},{},{},{},{},{},{},{}", d.time_usec, d.x, d.y, d.z, d.q[0], d.q[1], d.q[2], d.q[3]);
            ("ATT_POS_MOCAP", "time_usec,x,y,z,q0,q1,q2,q3")
        }
        MavMessage::VISION_POSITION_ESTIMATE(d) => {
            let _ = write!(r, "{},{},{},{},{},{},{},{}", d.usec, d.x, d.y, d.z, d.roll, d.pitch, d.yaw, d.reset_counter);
            ("VISION_POSITION_ESTIMATE", "usec,x,y,z,roll,pitch,yaw,reset_counter")
        }
        _ => return None,
    };
    Some((name.0, name.1, r))
}

/// Every position message in `log` to `<log>_csv/<MESSAGE>.csv`, each row
/// led by the tlog's receive time (Unix seconds) and the sender's system
/// / component id (ODOMETRY from compid 1 is PX4's estimate, anything else
/// is forwarded external vision). Returns (file name, rows) per file written.
pub fn tlog_to_csv(log: &Path) -> Result<(PathBuf, Vec<(String, usize)>), String> {
    let mut player = crate::tlog::Player::open(&log.to_string_lossy(), 1.0).map_err(|e| format!("{}: {e}", log.display()))?;
    let mut tables: BTreeMap<&'static str, (&'static str, String, usize)> = BTreeMap::new();
    while let Some((ts, frame)) = player.read_record().map_err(|e| e.to_string())? {
        for (header, msg) in crate::link::decode(&frame) {
            let Some((name, columns, values)) = row(&msg) else { continue };
            let t = tables.entry(name).or_insert((columns, String::new(), 0));
            let _ = writeln!(t.1, "{:.6},{},{},{values}", ts as f64 / 1e6, header.system_id, header.component_id);
            t.2 += 1;
        }
    }
    if tables.is_empty() {
        return Err("no position messages (LOCAL_POSITION_NED / ODOMETRY / ATT_POS_MOCAP / VISION_POSITION_ESTIMATE) in it".into());
    }
    let dir = out_dir(log);
    std::fs::create_dir_all(&dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    let mut written = Vec::new();
    for (name, (columns, body, rows)) in tables {
        let path = dir.join(format!("{name}.csv"));
        std::fs::File::create(&path)
            .and_then(|mut f| f.write_all(format!("tlog_unix_s,sysid,compid,{columns}\n{body}").as_bytes()))
            .map_err(|e| format!("{}: {e}", path.display()))?;
        written.push((format!("{name}.csv"), rows));
    }
    Ok((dir, written))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn csv_dir_next_to_log() {
        assert_eq!(out_dir(Path::new("px4_logs/log_3.ulg")), PathBuf::from("px4_logs/log_3_csv"));
    }
}
