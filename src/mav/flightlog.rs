//! PX4 flight-log (ULog) listing and download.
//!
//! The downloader asks for large LOG_REQUEST_DATA windows (PX4 streams many
//! 90-byte LOG_DATA packets back-to-back per request) and receives them over
//! a channel straight from the receiver thread - never through the shared
//! state lock, which the UI and receiver contend for. Mirrors
//! `mavlink/flightlog.py`.

use std::collections::HashMap;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::mpsc::{Receiver, RecvTimeoutError, Sender};
use std::time::Duration;

use mavlink::dialects::development::{self as mav, MavMessage};

use crate::config::*;
use crate::link::Link;
use crate::state::{self, FlightLogEntry, Shared, State, now};

/// (offset, data). Empty data = PX4's end-of-log marker.
pub type Chunk = (u32, Vec<u8>);

pub fn flight_log_filename(entry: &FlightLogEntry) -> String {
    let stamp = if entry.time_utc == 0 {
        "unknown_time".to_string()
    } else {
        chrono::DateTime::from_timestamp(entry.time_utc as i64, 0)
            .map(|t| t.format("%Y%m%d_%H%M%S").to_string())
            .unwrap_or_else(|| "unknown_time".into())
    };
    format!("px4_log_{:06}_{stamp}.ulg", entry.id)
}

fn send_log_request_end(link: &Link) {
    let (target_system, target_component) = link.target();
    link.send(&MavMessage::LOG_REQUEST_END(mav::LOG_REQUEST_END_DATA {
        target_system,
        target_component,
    }));
}

pub fn request_flight_logs(link: &Link, st: &mut State, allow_armed: bool) -> bool {
    if !link.has_peer() {
        st.error("No MAVLink connection");
        return false;
    }
    if st.armed && !allow_armed {
        st.warn("Flight-log retrieval is disabled while vehicle is ARMED");
        return false;
    }
    st.flight_logs.clear();
    st.flight_log_list_requested_at = now();
    st.flight_log_list_last_entry_at = 0.0;
    st.flight_log_list_complete = false;

    let (target_system, target_component) = link.target();
    if !link.send(&MavMessage::LOG_REQUEST_LIST(mav::LOG_REQUEST_LIST_DATA {
        start: 0,
        end: 0xFFFF,
        target_system,
        target_component,
    })) {
        st.flight_log_list_requested_at = 0.0;
        st.error("Failed to request PX4 flight-log list");
        return false;
    }
    st.command("LOG_REQUEST_LIST SENT");
    true
}

pub fn handle_log_entry(st: &mut State, d: &mav::LOG_ENTRY_DATA) {
    if st.flight_log_list_requested_at <= 0.0 {
        return;
    }
    let entry = FlightLogEntry { id: d.id, num_logs: d.num_logs, time_utc: d.time_utc, size: d.size };
    let num_logs = entry.num_logs as usize;
    st.flight_logs.insert(d.id, entry);
    st.flight_log_list_last_entry_at = now();
    if num_logs > 0 && st.flight_logs.len() >= num_logs {
        st.flight_log_list_complete = true;
    }
}

/// Hand a LOG_DATA packet to the active download, if it's for that log.
pub fn route_log_data(shared: &Shared, tx: &Sender<Chunk>, d: &mav::LOG_DATA_DATA) {
    let (active, id) = {
        let st = state::lock(shared);
        (st.dl_active, st.dl_id)
    };
    if !active || d.id as i32 != id {
        return;
    }
    let count = (d.count as usize).min(LOG_CHUNK_SIZE);
    let _ = tx.send((d.ofs, d.data[..count].to_vec()));
}

fn finish_list(link: &Link, st: &mut State) {
    send_log_request_end(link);
    st.flight_log_list_requested_at = 0.0;
    st.flight_log_list_complete = true;
    let n = st.flight_logs.len();
    st.info(format!("PX4 flight-log list loaded: {n} log(s)"));
}

pub fn check_flight_log_list(link: &Link, st: &mut State) {
    let requested_at = st.flight_log_list_requested_at;
    if requested_at == 0.0 {
        return;
    }
    let t = now();
    let received = st.flight_logs.len();
    if st.flight_log_list_complete {
        finish_list(link, st);
        return;
    }
    let last = st.flight_log_list_last_entry_at;
    if received > 0 && last > 0.0 && t - last >= LOG_LIST_QUIET_PERIOD {
        finish_list(link, st);
        return;
    }
    if t - requested_at >= LOG_LIST_TIMEOUT {
        send_log_request_end(link);
        st.flight_log_list_requested_at = 0.0;
        st.flight_log_list_complete = true;
        if received > 0 {
            st.warn(format!("Flight-log list request timed out; using {received} received log(s)"));
        } else {
            st.warn("PX4 returned no flight logs");
        }
    }
}

pub fn start_download(
    link: Arc<Link>,
    shared: Shared,
    rx: Arc<std::sync::Mutex<Receiver<Chunk>>>,
    entry: FlightLogEntry,
    log_dir: String,
    allow_armed: bool,
) -> bool {
    {
        let mut st = state::lock(&shared);
        if st.dl_active {
            st.warn("A flight-log download is already active");
            return false;
        }
        if st.armed && !allow_armed {
            st.warn("Flight-log download is disabled while vehicle is ARMED");
            return false;
        }
        if let Err(e) = std::fs::create_dir_all(&log_dir) {
            st.error(format!("Could not create log directory '{log_dir}': {e}"));
            return false;
        }
        st.dl_active = true;
        st.dl_cancel = false;
        st.dl_id = entry.id as i32;
        st.dl_size = entry.size;
        st.dl_received = 0;
        st.dl_started_at = now();
        st.dl_speed = 0.0;
        st.dl_path.clear();
        st.dl_status = "STARTING".into();
        st.dl_error.clear();
        st.command(format!("Flight-log download started: ID {}", entry.id));
    }

    std::thread::Builder::new()
        .name(format!("FlightLog-{}", entry.id))
        .spawn(move || {
            let rx = rx.lock().unwrap_or_else(|e| e.into_inner());
            download_worker(&link, &shared, &rx, &entry, &log_dir);
        })
        .is_ok()
}

pub fn cancel_download(st: &mut State) -> bool {
    if !st.dl_active {
        return false;
    }
    st.dl_cancel = true;
    st.dl_status = "CANCELLING".into();
    st.command("Flight-log download cancellation requested");
    true
}

fn cancelled(shared: &Shared) -> bool {
    state::lock(shared).dl_cancel
}

fn download_worker(link: &Link, shared: &Shared, rx: &Receiver<Chunk>, entry: &FlightLogEntry, log_dir: &str) {
    let final_path = PathBuf::from(log_dir).join(flight_log_filename(entry));
    let temp_path = final_path.with_extension("ulg.part");

    while rx.try_recv().is_ok() {}

    let result = download_into(link, shared, rx, entry, &temp_path);
    send_log_request_end(link);

    let result = result.and_then(|offset| {
        std::fs::rename(&temp_path, &final_path)
            .map(|_| offset)
            .map_err(|e| format!("Could not finalize downloaded log: {e}"))
    });
    if result.is_err() {
        let _ = std::fs::remove_file(&temp_path);
    }
    while rx.try_recv().is_ok() {}

    let mut st = state::lock(shared);
    st.dl_active = false;
    match result {
        Ok(offset) => {
            let elapsed = (now() - st.dl_started_at).max(0.001);
            st.dl_received = offset;
            st.dl_path = final_path.display().to_string();
            st.dl_status = "COMPLETE".into();
            st.dl_error.clear();
            st.info(format!(
                "Flight log {} downloaded: {} ({offset} bytes, {:.1} KiB/s)",
                entry.id,
                final_path.display(),
                offset as f64 / elapsed / 1024.0
            ));
        }
        Err(error) => {
            let was_cancel = error == "Cancelled by user";
            st.dl_status = if was_cancel { "CANCELLED".into() } else { "ERROR".into() };
            if was_cancel {
                st.warn(format!("Flight log {} download cancelled", entry.id));
            } else {
                st.error(format!("Flight log {} download failed: {error}", entry.id));
            }
            st.dl_error = error;
        }
    }
}

/// The windowed download loop. Returns the final byte count.
fn download_into(
    link: &Link,
    shared: &Shared,
    rx: &Receiver<Chunk>,
    entry: &FlightLogEntry,
    temp_path: &PathBuf,
) -> Result<u64, String> {
    let file = File::create(temp_path).map_err(|e| e.to_string())?;
    let mut out = BufWriter::with_capacity(1 << 20, file);
    let size = entry.size as u64;
    let mut offset: u64 = 0;
    let (target_system, target_component) = link.target();

    loop {
        if cancelled(shared) {
            return Err("Cancelled by user".into());
        }
        if size > 0 && offset >= size {
            break;
        }

        let request_start = offset;
        let mut request_count = if size > 0 {
            (LOG_REQUEST_SIZE as u64).min(size - request_start)
        } else {
            LOG_REQUEST_SIZE as u64
        };
        let mut request_end = request_start + request_count;
        let mut received: HashMap<u64, Vec<u8>> = HashMap::new();
        let mut last_contiguous = request_start;
        let mut end_seen = false;
        let mut range_complete = false;

        for retry in 1..=LOG_CHUNK_RETRIES {
            if cancelled(shared) {
                return Err("Cancelled by user".into());
            }
            state::lock(shared).dl_status =
                format!("DOWNLOADING {request_count} bytes (retry {retry}/{LOG_CHUNK_RETRIES})");

            link.send(&MavMessage::LOG_REQUEST_DATA(mav::LOG_REQUEST_DATA_DATA {
                ofs: request_start as u32,
                count: request_count as u32,
                id: entry.id,
                target_system,
                target_component,
            }));

            let deadline = now() + LOG_CHUNK_TIMEOUT;
            loop {
                let remaining = deadline - now();
                if remaining <= 0.0 {
                    break;
                }
                if cancelled(shared) {
                    return Err("Cancelled by user".into());
                }
                let (pkt_offset, mut data) = match rx.recv_timeout(Duration::from_secs_f64(remaining.min(0.2))) {
                    Ok(chunk) => chunk,
                    Err(RecvTimeoutError::Timeout) => continue,
                    Err(RecvTimeoutError::Disconnected) => return Err("receiver stopped".into()),
                };
                let pkt_offset = pkt_offset as u64;
                if data.is_empty() {
                    end_seen = true;
                } else if (request_start..request_end).contains(&pkt_offset) {
                    data.truncate((request_end - pkt_offset) as usize);
                    received.entry(pkt_offset).or_insert(data);
                }
                while let Some(packet) = received.get(&last_contiguous) {
                    last_contiguous += packet.len() as u64;
                }
                if last_contiguous >= request_end {
                    range_complete = true;
                    break;
                }
                // Unknown size: PX4's end marker closes the log.
                if end_seen && size == 0 && last_contiguous > request_start {
                    request_end = last_contiguous;
                    request_count = request_end - request_start;
                    range_complete = true;
                    break;
                }
            }
            if range_complete {
                break;
            }
        }

        if !range_complete {
            return Err(format!(
                "Range {request_start}-{request_end} incomplete: received {}/{request_count} bytes after {LOG_CHUNK_RETRIES} retries",
                last_contiguous.saturating_sub(request_start)
            ));
        }

        let mut write_offset = request_start;
        while write_offset < request_end {
            let data = received
                .get(&write_offset)
                .ok_or_else(|| format!("Missing packet at offset {write_offset}"))?;
            out.write_all(data).map_err(|e| e.to_string())?;
            write_offset += data.len() as u64;
        }
        offset = write_offset;

        {
            let mut st = state::lock(shared);
            st.dl_received = offset;
            let elapsed = (now() - st.dl_started_at).max(0.001);
            st.dl_speed = offset as f64 / elapsed;
        }

        if size == 0 && end_seen {
            break;
        }
    }

    out.flush().map_err(|e| e.to_string())?;
    Ok(offset)
}
