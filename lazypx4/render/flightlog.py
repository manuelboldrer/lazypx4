"""The [l] flight-log list / download screen."""

from __future__ import annotations

import time

from ..ansi import BOLD, DIM, GREEN, RED, RESET, YELLOW
from ..config import FLIGHT_LOG_PAGE_SIZE, LOG_CHUNK_RETRIES, settings
from ..search import footer_line
from ..state import session, state
from ..util import clamp
from .chrome import highlight_matches
from .jobpanel import job_panel_lines


def draw_flight_log_screen():
    with state.lock:
        logs = [
            state.flight_logs[log_id]
            for log_id in state.flight_log_order
            if log_id in state.flight_logs
        ]
        selected_index = state.flight_log_index
        requested_at = state.flight_log_list_requested_at
        complete = state.flight_log_list_complete

        active = state.flight_log_download_active
        download_id = state.flight_log_download_id
        download_size = state.flight_log_download_size
        download_received = state.flight_log_download_received
        download_speed = state.flight_log_download_speed
        download_status = state.flight_log_download_status
        download_path = state.flight_log_download_path
        download_error = state.flight_log_download_error
        download_retry = state.flight_log_download_retry

    lines = []

    if active:
        if download_size > 0:
            percent = min(100.0, download_received * 100.0 / download_size)
            size_text = (
                f"{download_received / (1024 * 1024):.2f}/"
                f"{download_size / (1024 * 1024):.2f} MiB"
            )
        else:
            percent = 0.0
            size_text = f"{download_received / (1024 * 1024):.2f} MiB"

        if download_speed > 0:
            speed_text = f"{download_speed / 1024:.1f} KiB/s"
        else:
            speed_text = "--"

        lines.append(
            BOLD + YELLOW + f" Downloading log {download_id}: {percent:6.1f}%" + RESET
        )
        lines.append(
            f"   {size_text}    {speed_text}"
            f"    retry {download_retry}/{LOG_CHUNK_RETRIES}"
        )
        lines.append(f"   Status: {download_status}")
        lines.append("")
        lines.append(BOLD + "ESC = cancel download" + RESET)

        return lines

    if requested_at:
        elapsed = time.monotonic() - requested_at
        status = YELLOW + f"LOADING ({elapsed:.1f}s)" + RESET
    elif complete:
        status = GREEN + "READY" + RESET
    else:
        status = DIM + "NOT LOADED" + RESET

    lines.append(f" Status: {status}    Logs: {len(logs)}")

    if download_status == "COMPLETE" and download_path:
        lines.append(GREEN + f" Last download: {download_path}" + RESET)
    elif download_status == "CANCELLED":
        lines.append(YELLOW + " Last download: cancelled" + RESET)
    elif download_status == "ERROR" and download_error:
        lines.append(RED + f" Last download error: {download_error}" + RESET)
    else:
        lines.append("")

    lines.append("")
    lines.append(
        DIM + f" {'ID':>6}  {'UTC TIME':<22} {'SIZE':>12}  {'INDEX':>9}" + RESET
    )
    lines.append(DIM + " " + "─" * 58 + RESET)

    if not logs:
        if requested_at:
            lines.append(DIM + " Waiting for LOG_ENTRY messages..." + RESET)
        else:
            lines.append(DIM + " No PX4 flight logs reported." + RESET)
    else:
        selected_index = clamp(selected_index, 0, len(logs) - 1)
        page_start = (selected_index // FLIGHT_LOG_PAGE_SIZE) * FLIGHT_LOG_PAGE_SIZE
        page = logs[page_start:page_start + FLIGHT_LOG_PAGE_SIZE]

        query = session.search_query

        for local_index, entry in enumerate(page):
            absolute_index = page_start + local_index
            selected = absolute_index == selected_index
            marker = ">" if selected else " "

            line = (
                f" {marker} "
                + highlight_matches(f"{entry.id:06d}  {entry.time_string:<22}", query)
                + f" {entry.size_string:>12}"
                f"  {absolute_index + 1:>3}/{len(logs):<3}"
            )

            if selected:
                lines.append(BOLD + GREEN + line + RESET)
            else:
                lines.append(line)

    footer = footer_line()
    if footer:
        lines.append(footer)

    lines.extend(job_panel_lines("ulog_upload"))
    lines.extend(job_panel_lines("ecl_ekf"))

    lines.append("")
    lines.append("UP/DOWN = select    PGUP/PGDN = page    HOME/END    / search")
    lines.append("ENTER = download selected .ulg    r = refresh list    l = back    ESC = panels")
    lines.append("u = upload last download to web    a = EKF health-check on it")
    lines.append(DIM + f"Download directory: {settings.log_dir}" + RESET)

    return lines
