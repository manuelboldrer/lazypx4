//! Telemetry logs in QGC / MAVProxy `.tlog` format: each record is an
//! 8-byte big-endian Unix time in microseconds followed by one raw MAVLink
//! frame. Frames are cut by their own length header (never decoded and
//! re-encoded), so what is recorded is byte-for-byte what came off the wire,
//! including messages this build's dialect does not know.

use std::fs::File;
use std::io::{self, BufWriter, Write};
use std::path::PathBuf;
use std::time::Instant;

/// Length of the MAVLink frame starting at `b[0]`, if `b` starts with one
/// whose header is complete.
pub fn frame_len(b: &[u8]) -> Option<usize> {
    match *b.first()? {
        0xFD => {
            let (len, incompat) = (*b.get(1)? as usize, *b.get(2)?);
            Some(12 + len + if incompat & 0x01 != 0 { 13 } else { 0 })
        }
        0xFE => Some(8 + *b.get(1)? as usize),
        _ => None,
    }
}

fn unix_us() -> u64 {
    std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_micros() as u64).unwrap_or(0)
}

/// Appends frames to a .tlog, opened lazily on the first frame so a session
/// that never hears a vehicle leaves no empty file behind.
pub struct Recorder {
    path: PathBuf,
    out: Option<BufWriter<File>>,
    failed: bool,
    last_flush: Instant,
    /// Frames written so far.
    pub frames: u64,
}

impl Recorder {
    pub fn new(dir: &str, port: u16) -> Self {
        let name = format!("lazypx4_{}_p{port}.tlog", chrono::Local::now().format("%Y%m%d_%H%M%S"));
        Self::at(PathBuf::from(dir).join(name))
    }

    /// Record to exactly `path` (the REC pane's mavlink/telemetry.tlog).
    pub fn at(path: PathBuf) -> Self {
        Recorder { path, out: None, failed: false, last_flush: Instant::now(), frames: 0 }
    }

    pub fn path(&self) -> &std::path::Path {
        &self.path
    }

    pub fn is_open(&self) -> bool {
        self.out.is_some()
    }

    /// Every complete frame in `bytes` (one datagram, or one frame we sent).
    pub fn write(&mut self, bytes: &[u8]) {
        if self.failed {
            return;
        }
        if self.out.is_none() {
            let opened = self.path.parent().map(std::fs::create_dir_all).unwrap_or(Ok(())).and_then(|_| std::fs::OpenOptions::new().create(true).append(true).open(&self.path));
            match opened {
                Ok(f) => self.out = Some(BufWriter::new(f)),
                Err(_) => {
                    self.failed = true;
                    return;
                }
            }
        }
        let out = self.out.as_mut().unwrap();
        let ts = unix_us().to_be_bytes();
        let mut rest = bytes;
        while let Some(n) = frame_len(rest).filter(|&n| n <= rest.len()) {
            if out.write_all(&ts).and_then(|_| out.write_all(&rest[..n])).is_err() {
                self.failed = true;
                return;
            }
            rest = &rest[n..];
            self.frames += 1;
        }
        // A crash or power cut should lose at most about a second.
        if self.last_flush.elapsed().as_secs_f64() >= 1.0 {
            let _ = out.flush();
            self.last_flush = Instant::now();
        }
    }
}

impl Drop for Recorder {
    fn drop(&mut self) {
        if let Some(out) = self.out.as_mut() {
            let _ = out.flush();
        }
    }
}

/// Reads a .tlog back in (recorded) real time, scaled by `speed`.
pub struct Player {
    reader: mavlink_tlog::TlogReader<File>,
    /// Record read ahead but not yet due.
    pending: Option<(u64, Vec<u8>)>,
    /// (tlog time, wall clock) the schedule is anchored to.
    base: Option<(u64, Instant)>,
    pub first_ts: Option<u64>,
    pub last_ts: u64,
    pub speed: f64,
    pub paused: bool,
    pub done: bool,
}

impl Player {
    pub fn open(path: &str, speed: f64) -> io::Result<Self> {
        Ok(Player {
            reader: mavlink_tlog::TlogReader::new(File::open(path)?),
            pending: None,
            base: None,
            first_ts: None,
            last_ts: 0,
            speed: if speed > 0.0 { speed } else { 1.0 },
            paused: false,
            done: false,
        })
    }

    /// Seconds into the recording of the last frame delivered.
    pub fn position_s(&self) -> f64 {
        self.first_ts.map(|f| self.last_ts.saturating_sub(f) as f64 / 1e6).unwrap_or(0.0)
    }

    /// Re-anchor after a pause or speed change so time doesn't jump.
    pub fn rebase(&mut self) {
        self.base = self.first_ts.map(|_| (self.last_ts, Instant::now()));
    }

    /// The next record whose frame passes its CRC check. Damaged bytes are
    /// skipped and the reader resyncs on the next valid frame.
    fn read_record(&mut self) -> io::Result<Option<(u64, Vec<u8>)>> {
        Ok(self.reader.next_record()?.map(|r| (r.timestamp_us, r.frame.as_bytes().to_vec())))
    }

    /// The frames due now (at most a batch), or how long to wait for the
    /// next one.
    pub fn due(&mut self) -> io::Result<Result<Vec<Vec<u8>>, std::time::Duration>> {
        let idle = std::time::Duration::from_millis(100);
        if self.paused || self.done {
            return Ok(Err(idle));
        }
        let mut out = Vec::new();
        while out.len() < 200 {
            let record = match self.pending.take() {
                Some(r) => r,
                None => match self.read_record()? {
                    Some(r) => r,
                    None => {
                        self.done = true;
                        break;
                    }
                },
            };
            let (ts, frame) = record;
            let (base_ts, base_wall) = *self.base.get_or_insert((ts, Instant::now()));
            self.first_ts.get_or_insert(ts);
            let offset = ts.saturating_sub(base_ts) as f64 / 1e6 / self.speed;
            let due = base_wall + std::time::Duration::from_secs_f64(offset);
            let now = Instant::now();
            if due > now {
                self.pending = Some((ts, frame));
                if out.is_empty() {
                    return Ok(Err((due - now).min(idle)));
                }
                break;
            }
            self.last_ts = ts;
            out.push(frame);
        }
        Ok(Ok(out))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_lengths() {
        assert_eq!(frame_len(&[0xFD, 9, 0]), Some(21));
        assert_eq!(frame_len(&[0xFD, 9, 1]), Some(34));
        assert_eq!(frame_len(&[0xFE, 9]), Some(17));
        assert_eq!(frame_len(&[0x00, 9, 0]), None);
        assert_eq!(frame_len(&[0xFD]), None);
    }

    #[test]
    fn records_and_replays_frames() {
        let dir = std::env::temp_dir().join(format!("lazypx4_tlog_test_{}", std::process::id()));
        let mut rec = Recorder::new(dir.to_str().unwrap(), 1);
        // Two v2 frames in one datagram (payload lengths 1 and 2) + junk tail.
        let mut datagram = [v2(0, &[0xAA]), v2(1, &[0xBB, 0xCC])].concat();
        datagram.push(0xFD);
        rec.write(&datagram);
        let path = rec.path().to_path_buf();
        drop(rec);
        let mut p = Player::open(path.to_str().unwrap(), 1000.0).unwrap();
        let mut frames = Vec::new();
        while !p.done {
            if let Ok(batch) = p.due().unwrap() {
                frames.extend(batch);
            }
        }
        assert_eq!(frames, vec![datagram[..13].to_vec(), datagram[13..27].to_vec()]);
        let _ = std::fs::remove_dir_all(dir);
    }

    fn v2(seq: u8, payload: &[u8]) -> Vec<u8> {
        let mut out = [0u8; 280];
        let header = mavlink_tlog::Header { seq, sys_id: 1, comp_id: 1, msg_id: 0 };
        let n = mavlink_tlog::frame::encode(mavlink_tlog::frame::Version::V2, header, payload, None, &mut out).unwrap();
        out[..n].to_vec()
    }

    #[test]
    fn skips_damaged_records_and_keeps_replaying() {
        let good = [v2(0, &[0xAA]), v2(1, &[0xBB]), v2(2, &[0xCC])];
        let mut bad_crc = v2(3, &[0xDD]);
        *bad_crc.last_mut().unwrap() ^= 0xFF;
        let mut file = Vec::new();
        for (i, f) in [&good[0], &bad_crc, &vec![0x00, 0x13, 0x37], &good[1], &good[2]].iter().enumerate() {
            file.extend_from_slice(&(1_700_000_000_000_000u64 + i as u64).to_be_bytes());
            file.extend_from_slice(f);
        }
        let path = std::env::temp_dir().join(format!("lazypx4_tlog_damaged_{}.tlog", std::process::id()));
        std::fs::write(&path, &file).unwrap();
        let mut p = Player::open(path.to_str().unwrap(), 1000.0).unwrap();
        let mut frames = Vec::new();
        while !p.done {
            if let Ok(batch) = p.due().unwrap() {
                frames.extend(batch);
            }
        }
        // The corrupted frame is not replayed, and the junk record does not
        // end playback.
        assert_eq!(frames, good.to_vec());
        let _ = std::fs::remove_file(path);
    }
}
