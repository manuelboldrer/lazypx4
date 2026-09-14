"""The [e] estimation / sensors screen: EKF health, altitude, GPS, rangefinder, baro."""

from __future__ import annotations

import math
import time

from ..ansi import BOLD, DIM, GREEN, RED, RESET, YELLOW, ui_section
from ..config import EKF2_HGT_REF_NAMES, EKF2_RNG_CTRL_NAMES
from ..mavlink.connection import get_param_value
from ..state import state
from .chrome import (
    distance_orientation_text,
    ekf_summary,
    estimator_flag_labels,
    gps_fix_display,
    rtk_text,
    test_ratio_cell,
    vibration_verdict,
)


def draw_estimation_screen():
    with state.lock:
        ekf_flags = state.ekf_flags
        is_est = state.est_is_estimator_status
        ekf_status = state.ekf_status
        last_ekf = state.last_ekf

        vel_ratio = state.est_vel_ratio
        ph_ratio = state.est_pos_horiz_ratio
        pv_ratio = state.est_pos_vert_ratio
        mag_ratio = state.est_mag_ratio
        hagl_ratio = state.est_hagl_ratio
        tas_ratio = state.est_tas_ratio
        pos_h_acc = state.est_pos_horiz_accuracy
        pos_v_acc = state.est_pos_vert_accuracy

        vel_var = state.ekf_vel_variance
        ph_var = state.ekf_pos_horiz_variance
        pv_var = state.ekf_pos_vert_variance
        mag_var = state.ekf_compass_variance
        terr_var = state.ekf_terrain_variance

        vx, vy, vz = state.vx, state.vy, state.vz

        amsl = state.alt_amsl
        arel = state.alt_relative
        aloc = state.alt_local
        amono = state.alt_monotonic
        aterr = state.alt_terrain
        abot = state.alt_bottom_clearance
        last_alt = state.last_altitude
        vfr_alt = state.vfr_alt
        climb = state.climb_rate
        groundspeed = state.groundspeed
        airspeed = state.airspeed

        gfix = state.gps_fix
        gsats = state.gps_sats
        ghdop = state.gps_hdop
        gvdop = state.gps_vdop
        ghacc = state.gps_h_acc
        gvacc = state.gps_v_acc
        gvelacc = state.gps_vel_acc
        galt = state.gps_alt
        gspeed = state.gps_speed
        last_gps = state.last_gps

        g2fix = state.gps2_fix
        g2sats = state.gps2_sats
        g2hdop = state.gps2_hdop
        last_gps2 = state.last_gps2

        origin_set = state.local_origin_set
        olat = state.local_origin_lat
        olon = state.local_origin_lon
        oalt = state.local_origin_alt

        rf_dist = state.rangefinder_distance
        rf_min = state.rangefinder_min
        rf_max = state.rangefinder_max
        rf_q = state.rangefinder_quality
        rf_or = state.rangefinder_orientation
        last_rf = state.last_rangefinder

        press = state.baro_pressure
        btemp = state.baro_temp
        last_baro = state.last_baro

        vib_x = state.vibration_x
        vib_y = state.vibration_y
        vib_z = state.vibration_z
        clip0 = state.clipping_0
        clip1 = state.clipping_1
        clip2 = state.clipping_2
        last_vib = state.last_vibration

    now = time.monotonic()

    def age(timestamp):
        if not timestamp:
            return DIM + "never" + RESET
        delta = now - timestamp
        color = GREEN if delta < 3.0 else (YELLOW if delta < 10.0 else RED)
        return f"{color}{delta:.1f}s{RESET}"

    speed_3d = math.sqrt(vx * vx + vy * vy + vz * vz)

    lines = []

    # --- EKF ---------------------------------------------------------
    verdict, verdict_color = ekf_summary(ekf_flags, is_est, ekf_status)
    source = "ESTIMATOR_STATUS" if is_est else "EKF_STATUS_REPORT"

    lines.append(ui_section(
        "EKF",
        f"{verdict_color}{verdict}{RESET}   {source}   updated {age(last_ekf)}",
    ))

    active_flags = estimator_flag_labels(ekf_flags, is_est)
    lines.append(
        "   fusing/valid: "
        + (", ".join(active_flags) if active_flags else DIM + "none" + RESET)
    )

    lines.append(
        "   innov test ratios  "
        + "  ".join([
            test_ratio_cell("vel", vel_ratio),
            test_ratio_cell("posH", ph_ratio),
            test_ratio_cell("posV", pv_ratio),
            test_ratio_cell("mag", mag_ratio),
            test_ratio_cell("hagl", hagl_ratio),
            test_ratio_cell("tas", tas_ratio),
        ])
        + DIM + "   (< 1.0 healthy)" + RESET
    )
    lines.append(
        f"   reported accuracy   horiz: {pos_h_acc:.2f} m   vert: {pos_v_acc:.2f} m"
    )

    if any(v > 0 for v in (vel_var, ph_var, pv_var, mag_var, terr_var)):
        lines.append(
            f"   variances   vel {vel_var:.3f}   posH {ph_var:.3f}   posV {pv_var:.3f}"
            f"   mag {mag_var:.3f}   terrain {terr_var:.3f}"
        )

    # --- Altitude / height reference -------------------------------
    lines.append("")
    lines.append(ui_section("ALTITUDE", f"updated {age(last_alt)}"))

    if last_alt:
        lines.append(
            f"   AMSL: {amsl:9.2f} m   relative: {arel:8.2f} m"
            f"   local: {aloc:8.2f} m   monotonic: {amono:9.2f} m"
        )
        terr_text = f"{aterr:.2f} m" if aterr else DIM + "n/a" + RESET
        bot_text = f"{abot:.2f} m" if abot else DIM + "n/a" + RESET
        lines.append(f"   terrain estimate: {terr_text}   bottom clearance (AGL): {bot_text}")
    else:
        lines.append(
            f"   AMSL (VFR_HUD): {vfr_alt:.2f} m   "
            + DIM + "(ALTITUDE message not received)" + RESET
        )

    lines.append(
        f"   climb: {climb:+.2f} m/s   ground speed: {groundspeed:.2f} m/s"
        f"   air speed: {airspeed:.2f} m/s   |V|: {speed_3d:.2f} m/s"
    )

    hgt_ref = get_param_value("EKF2_HGT_REF")
    baro_ctrl = get_param_value("EKF2_BARO_CTRL")
    gps_ctrl = get_param_value("EKF2_GPS_CTRL")
    rng_ctrl = get_param_value("EKF2_RNG_CTRL")
    ev_ctrl = get_param_value("EKF2_EV_CTRL")
    of_ctrl = get_param_value("EKF2_OF_CTRL")

    if hgt_ref is None:
        lines.append(
            "   height reference: " + DIM + "EKF2_* params not loaded yet (press [r])" + RESET
        )
    else:
        parts = [
            "primary=" + BOLD
            + EKF2_HGT_REF_NAMES.get(int(round(hgt_ref)), str(int(round(hgt_ref))))
            + RESET
        ]
        if baro_ctrl is not None:
            parts.append("baro=" + ("on" if int(round(baro_ctrl)) else "off"))
        if gps_ctrl is not None:
            parts.append(f"gps=0x{int(round(gps_ctrl)):x}")
        if rng_ctrl is not None:
            parts.append(
                "rng="
                + EKF2_RNG_CTRL_NAMES.get(int(round(rng_ctrl)), str(int(round(rng_ctrl))))
            )
        if ev_ctrl is not None and int(round(ev_ctrl)):
            parts.append(f"vision=0x{int(round(ev_ctrl)):x}")
        if of_ctrl is not None and int(round(of_ctrl)):
            parts.append("flow=on")
        lines.append("   height reference: " + "   ".join(parts))

    # --- GPS ------------------------------------------------------
    lines.append("")
    lines.append(ui_section("GPS", f"updated {age(last_gps)}"))

    fix_color, fix_name = gps_fix_display(gfix)
    lines.append(
        f"   fix: {fix_color}{fix_name}{RESET}   sats: {gsats}"
        f"   HDOP: {ghdop:.2f}   VDOP: {gvdop:.2f}   RTK: {rtk_text(gfix)}"
    )
    lines.append(
        f"   MSL alt: {galt:.1f} m   EPH: {ghacc:.2f} m   EPV: {gvacc:.2f} m"
        f"   speed: {gspeed:.2f} m/s   vel acc: {gvelacc:.2f} m/s"
    )

    if last_gps2:
        f2_color, f2_name = gps_fix_display(g2fix)
        lines.append(
            f"   GPS2  fix: {f2_color}{f2_name}{RESET}   sats: {g2sats}"
            f"   HDOP: {g2hdop:.2f}   updated: {age(last_gps2)}"
        )
    else:
        lines.append("   GPS2  " + DIM + "not detected" + RESET)

    if origin_set:
        lines.append(f"   local origin: {olat:.7f}, {olon:.7f}   alt {oalt:.2f} m")
    else:
        lines.append("   local origin: " + DIM + "not set" + RESET)

    # --- Rangefinder --------------------------------------------
    lines.append("")
    lines.append(ui_section("RANGEFINDER", f"updated {age(last_rf)}"))

    if last_rf:
        quality_text = f"{rf_q}%" if rf_q >= 0 else "n/a"
        lines.append(
            f"   distance: {rf_dist:.2f} m   range: {rf_min:.2f}-{rf_max:.2f} m"
            f"   facing: {distance_orientation_text(rf_or)}   signal: {quality_text}"
        )
    else:
        lines.append("   " + DIM + "no DISTANCE_SENSOR messages received" + RESET)

    # --- Barometer ---------------------------------------------
    lines.append("")
    lines.append(ui_section("BAROMETER", f"updated {age(last_baro)}"))

    if last_baro:
        lines.append(f"   pressure: {press:.2f} hPa   temperature: {btemp:.1f} C")
    else:
        lines.append("   " + DIM + "no SCALED_PRESSURE messages received" + RESET)

    # --- Vibration -----------------------------------------------
    lines.append("")
    lines.append(ui_section("VIBRATION", f"updated {age(last_vib)}"))

    if last_vib:
        verdict, verdict_color = vibration_verdict(vib_x, vib_y, vib_z, clip0, clip1, clip2)
        lines.append(
            f"   {verdict_color}{verdict}{RESET}"
            f"   X {vib_x:.2f}  Y {vib_y:.2f}  Z {vib_z:.2f} m/s² rms"
        )
        clip_color = RED if (clip0 or clip1 or clip2) else DIM
        lines.append(
            f"   clipping (accel saturation events)   "
            f"IMU0: {clip_color}{clip0}{RESET}   IMU1: {clip_color}{clip1}{RESET}"
            f"   IMU2: {clip_color}{clip2}{RESET}"
        )
    else:
        lines.append("   " + DIM + "no VIBRATION messages received" + RESET)

    lines.append("")
    lines.append("[r] re-request streams / EKF2 params    [e] back    [ESC] panels")

    return lines
