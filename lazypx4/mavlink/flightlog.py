"""PX4 flight-log (ULog) listing and download.

The downloader uses large LOG_REQUEST_DATA ranges and hands LOG_DATA packets
straight from the receiver thread to the worker thread through
:data:`lazypx4.state.flight_log_queue` - never through ``state`` or its lock.

A LOG_DATA packet only matters to one consumer (the active download), so it
does not need to go through the lock the UI / keyboard / receiver threads all
contend for. ``queue.Queue`` has its own internal lock and ``get(timeout=...)``
wakes immediately on ``put`` instead of polling. Routing every packet of a
multi-megabyte log through the shared lock is what made the previous
implementation slow.
"""

from __future__ import annotations

import os
import queue
import threading
import time
from datetime import datetime, timezone

from ..config import (
    LOG_CHUNK_RETRIES,
    LOG_CHUNK_SIZE,
    LOG_CHUNK_TIMEOUT,
    LOG_LIST_QUIET_PERIOD,
    LOG_LIST_TIMEOUT,
    LOG_REQUEST_SIZE,
    settings,
)
from ..eventlog import log_command, log_error, log_info, log_warn
from ..models import FlightLogEntry
from ..state import flight_log_queue, session, shutdown_event, state
from ..util import safe_int


def format_flight_log_id(log_id):
    return f"{safe_int(log_id):06d}"


def flight_log_filename(entry):
    if entry.time_utc:
        try:
            stamp = datetime.fromtimestamp(
                entry.time_utc, tz=timezone.utc
            ).strftime("%Y%m%d_%H%M%S")
        except (OverflowError, OSError, ValueError):
            stamp = "unknown_time"
    else:
        stamp = "unknown_time"

    return f"px4_log_{format_flight_log_id(entry.id)}_{stamp}.ulg"


def send_log_request_end(master):
    if master is None:
        return
    try:
        master.mav.log_request_end_send(
            master.target_system,
            master.target_component,
        )
    except Exception as exc:
        log_warn(f"LOG_REQUEST_END failed: {exc}")


def request_flight_logs(master):
    if master is None:
        log_error("No MAVLink connection")
        return False

    with state.lock:
        if state.armed and not settings.allow_log_download_while_armed:
            log_warn("Flight-log retrieval is disabled while vehicle is ARMED")
            return False

        state.flight_logs.clear()
        state.flight_log_order.clear()
        state.flight_log_list_requested_at = time.monotonic()
        state.flight_log_list_last_entry_at = 0.0
        state.flight_log_list_complete = False
        state.flight_log_index = 0

    try:
        # 0..0xFFFF requests the complete available log list.
        master.mav.log_request_list_send(
            master.target_system,
            master.target_component,
            0,
            0xFFFF,
        )
        log_command("LOG_REQUEST_LIST SENT")
        return True
    except Exception as exc:
        with state.lock:
            state.flight_log_list_requested_at = 0.0
        log_error(f"Failed to request PX4 flight-log list: {exc}")
        return False


def handle_log_entry(msg):
    now = time.monotonic()

    log_id = safe_int(getattr(msg, "id", -1), -1)
    if log_id < 0:
        return

    entry = FlightLogEntry(
        id=log_id,
        num_logs=safe_int(getattr(msg, "num_logs", 0)),
        last_log_num=safe_int(getattr(msg, "last_log_num", 0)),
        time_utc=safe_int(getattr(msg, "time_utc", 0)),
        size=safe_int(getattr(msg, "size", 0)),
    )

    with state.lock:
        if state.flight_log_list_requested_at <= 0:
            return

        state.flight_logs[log_id] = entry

        if log_id not in state.flight_log_order:
            state.flight_log_order.append(log_id)
            state.flight_log_order.sort()

        state.flight_log_list_last_entry_at = now

        if entry.num_logs > 0 and len(state.flight_logs) >= entry.num_logs:
            # Leave requested_at set so check_flight_log_list() sends the
            # terminating LOG_REQUEST_END exactly once.
            state.flight_log_list_complete = True


def handle_log_data(msg):
    """Hand a LOG_DATA packet to the active download, if any.

    Deliberately reads ``state.flight_log_download_active`` /
    ``state.flight_log_download_id`` WITHOUT the lock: these are simple
    attribute reads, atomic under the GIL, so this can only ever see the old
    value for a brief instant right at the start/end of a download (worst
    case: a couple of stray packets get ignored or queued and later discarded
    by the offset check in the worker) - never a torn value.
    """
    if not state.flight_log_download_active:
        return

    log_id = safe_int(getattr(msg, "id", -1), -1)
    if log_id != state.flight_log_download_id:
        return

    count = safe_int(getattr(msg, "count", 0), 0)
    offset = safe_int(getattr(msg, "ofs", -1), -1)

    if count <= 0:
        # Zero count is an EOF indication from PX4. Completion is still
        # primarily determined by LOG_ENTRY.size in the worker, because some
        # firmware doesn't send a trailing zero-count packet when the final
        # request exactly reaches the advertised size.
        flight_log_queue.put((offset, b""))
        return

    raw = getattr(msg, "data", [])
    try:
        data = bytes(raw[:min(count, LOG_CHUNK_SIZE)])
    except Exception:
        try:
            data = bytes(raw)[:min(count, LOG_CHUNK_SIZE)]
        except Exception:
            return

    if not data:
        return

    flight_log_queue.put((offset, data))


def _finish_list(received):
    send_log_request_end(session.link)
    with state.lock:
        state.flight_log_list_requested_at = 0.0
        state.flight_log_list_complete = True
        state.flight_log_order.sort()
    log_info(f"PX4 flight-log list loaded: {received} log(s)")


def check_flight_log_list():
    with state.lock:
        requested_at = state.flight_log_list_requested_at
        last_entry_at = state.flight_log_list_last_entry_at
        complete = state.flight_log_list_complete
        received = len(state.flight_log_order)

    if not requested_at:
        return

    now = time.monotonic()

    if complete:
        _finish_list(received)
        return

    # PX4 sends one LOG_ENTRY per log. Once entries stop arriving for a short
    # quiet period, the list is complete even if the firmware did not provide
    # a useful num_logs value.
    if received and last_entry_at and now - last_entry_at >= LOG_LIST_QUIET_PERIOD:
        _finish_list(received)
        return

    if now - requested_at >= LOG_LIST_TIMEOUT:
        send_log_request_end(session.link)
        with state.lock:
            state.flight_log_list_requested_at = 0.0
            state.flight_log_list_complete = True
            state.flight_log_order.sort()

        if received:
            log_warn(
                f"Flight-log list request timed out; using {received} received log(s)"
            )
        else:
            log_warn("PX4 returned no flight logs")


def start_flight_log_download(entry):
    if session.link is None:
        log_error("No MAVLink connection")
        return False

    with state.lock:
        if state.flight_log_download_active:
            log_warn("A flight-log download is already active")
            return False

        if state.armed and not settings.allow_log_download_while_armed:
            log_warn("Flight-log download is disabled while vehicle is ARMED")
            return False

        state.flight_log_download_active = True
        state.flight_log_download_cancel = False
        state.flight_log_download_id = entry.id
        state.flight_log_download_size = max(0, entry.size)
        state.flight_log_download_received = 0
        state.flight_log_download_offset = 0
        state.flight_log_download_retry = 0
        state.flight_log_download_started_at = time.monotonic()
        state.flight_log_download_speed = 0.0
        state.flight_log_download_path = ""
        state.flight_log_download_status = "STARTING"
        state.flight_log_download_error = ""

    try:
        os.makedirs(settings.log_dir, exist_ok=True)
    except Exception as exc:
        with state.lock:
            state.flight_log_download_active = False
            state.flight_log_download_status = "ERROR"
            state.flight_log_download_error = str(exc)
        log_error(f"Could not create log directory '{settings.log_dir}': {exc}")
        return False

    thread = threading.Thread(
        target=flight_log_download_worker,
        args=(entry,),
        daemon=True,
        name=f"FlightLog-{entry.id}",
    )
    thread.start()

    log_command(f"Flight-log download started: ID {entry.id}")
    return True


def cancel_flight_log_download():
    with state.lock:
        if not state.flight_log_download_active:
            return False
        state.flight_log_download_cancel = True
        state.flight_log_download_status = "CANCELLING"

    # Wake the worker immediately if it is blocked waiting on the queue.
    try:
        flight_log_queue.put_nowait(None)
    except Exception:
        pass

    log_command("Flight-log download cancellation requested")
    return True


def _drain_queue():
    while True:
        try:
            flight_log_queue.get_nowait()
        except queue.Empty:
            break


def flight_log_download_worker(entry):
    """Download one ULog using large LOG_REQUEST_DATA ranges.

    Everything below the queue read is local to this thread - no lock, no
    polling - which is what lets it keep pace with PX4 streaming packets
    back-to-back.
    """
    master = session.link
    final_path = os.path.join(settings.log_dir, flight_log_filename(entry))
    temp_path = final_path + ".part"
    success = False
    error_text = ""
    offset = 0

    # Drop anything left over from a previous download (e.g. very late
    # duplicate packets) so it can't be misread as belonging to this one.
    _drain_queue()

    try:
        with open(temp_path, "wb", buffering=1024 * 1024) as output:
            while not shutdown_event.is_set():
                with state.lock:
                    if state.flight_log_download_cancel:
                        error_text = "Cancelled by user"
                        break

                if entry.size > 0 and offset >= entry.size:
                    success = True
                    break

                request_start = offset
                request_count = (
                    min(LOG_REQUEST_SIZE, entry.size - request_start)
                    if entry.size > 0 else LOG_REQUEST_SIZE
                )
                request_end = request_start + request_count
                range_complete = False
                received = {}
                last_contiguous = request_start
                end_seen = False

                for retry in range(1, LOG_CHUNK_RETRIES + 1):
                    if shutdown_event.is_set():
                        break

                    with state.lock:
                        if state.flight_log_download_cancel:
                            error_text = "Cancelled by user"
                            break
                        state.flight_log_download_retry = retry
                        state.flight_log_download_status = (
                            f"DOWNLOADING {request_count:,} bytes "
                            f"(retry {retry}/{LOG_CHUNK_RETRIES})"
                        )

                    if error_text:
                        break

                    try:
                        master.mav.log_request_data_send(
                            master.target_system,
                            master.target_component,
                            entry.id,
                            request_start,
                            request_count,
                        )
                    except Exception as exc:
                        error_text = f"LOG_REQUEST_DATA failed: {exc}"
                        time.sleep(0.15)
                        continue

                    deadline = time.monotonic() + LOG_CHUNK_TIMEOUT

                    while True:
                        remaining = deadline - time.monotonic()

                        if remaining <= 0 or shutdown_event.is_set():
                            break

                        with state.lock:
                            cancelled = state.flight_log_download_cancel

                        if cancelled:
                            error_text = "Cancelled by user"
                            break

                        try:
                            item = flight_log_queue.get(timeout=min(remaining, 0.2))
                        except queue.Empty:
                            continue

                        if item is None:
                            # Wake-up nudge (e.g. from cancel); just re-check.
                            continue

                        pkt_offset, pkt_data = item

                        if not pkt_data:
                            end_seen = True
                        elif request_start <= pkt_offset < request_end:
                            if pkt_offset + len(pkt_data) > request_end:
                                pkt_data = pkt_data[:request_end - pkt_offset]
                            if pkt_data:
                                received.setdefault(pkt_offset, pkt_data)
                        # else: packet belongs to a different window
                        # (stale retry / previous chunk) - harmless, drop it.

                        progressed = True
                        while progressed:
                            progressed = False
                            packet = received.get(last_contiguous)
                            if packet:
                                last_contiguous += len(packet)
                                progressed = True

                        if last_contiguous >= request_end:
                            range_complete = True
                            break

                        if end_seen and entry.size <= 0 and last_contiguous > request_start:
                            request_end = last_contiguous
                            request_count = request_end - request_start
                            range_complete = True
                            break

                    if error_text:
                        break
                    if range_complete:
                        break

                if error_text:
                    break

                if not range_complete:
                    error_text = (
                        f"Range {request_start:,}-{request_end:,} incomplete: "
                        f"received {max(0, last_contiguous - request_start):,}/"
                        f"{request_count:,} bytes after {LOG_CHUNK_RETRIES} retries"
                    )
                    break

                write_offset = request_start
                while write_offset < request_end:
                    data = received.get(write_offset)
                    if not data:
                        error_text = f"Missing packet at offset {write_offset:,}"
                        break
                    output.write(data)
                    write_offset += len(data)

                if error_text:
                    break

                offset = write_offset
                with state.lock:
                    state.flight_log_download_received = offset
                    state.flight_log_download_offset = offset
                    elapsed = max(0.001, time.monotonic() - state.flight_log_download_started_at)
                    state.flight_log_download_speed = offset / elapsed

                if entry.size > 0 and offset >= entry.size:
                    success = True
                    break
                if entry.size <= 0 and end_seen:
                    success = True
                    break

        send_log_request_end(master)
        if success:
            try:
                os.replace(temp_path, final_path)
            except Exception as exc:
                success = False
                error_text = f"Could not finalize downloaded log: {exc}"
        if not success:
            try:
                if os.path.exists(temp_path):
                    os.remove(temp_path)
            except Exception:
                pass
    except Exception as exc:
        error_text = str(exc)
        send_log_request_end(master)
        try:
            if os.path.exists(temp_path):
                os.remove(temp_path)
        except Exception:
            pass

    with state.lock:
        state.flight_log_download_active = False
        if success:
            state.flight_log_download_received = offset
            state.flight_log_download_offset = offset
            state.flight_log_download_path = final_path
            state.flight_log_download_status = "COMPLETE"
            state.flight_log_download_error = ""
        else:
            state.flight_log_download_status = (
                "CANCELLED" if error_text == "Cancelled by user" else "ERROR"
            )
            state.flight_log_download_error = error_text

    # Whatever's left in the queue belonged to this download and is now
    # useless; drop it so a future download doesn't have to sift through it.
    _drain_queue()

    if success:
        elapsed = max(0.001, time.monotonic() - state.flight_log_download_started_at)
        speed = offset / elapsed
        log_info(
            f"Flight log {entry.id} downloaded: {final_path} "
            f"({offset} bytes, {speed / 1024:.1f} KiB/s)"
        )
    elif error_text == "Cancelled by user":
        log_warn(f"Flight log {entry.id} download cancelled")
    else:
        log_error(
            f"Flight log {entry.id} download failed at {offset} bytes: {error_text}"
        )
