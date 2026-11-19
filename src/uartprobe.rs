//! Pre-flash check for flashing over a companion UART: PX4 must be heard on
//! that UART (a CRC-valid MAVLink HEARTBEAT) before px_uploader reboots it
//! into the bootloader. The bootloader waits forever after a commanded
//! reboot, so rebooting it through a port that isn't wired to the FC leaves
//! it stuck until someone power-cycles the vehicle.

use std::ffi::CString;
use std::time::{Duration, Instant};

/// Listen on `port` at each baud in turn for up to `per_baud`; the first baud
/// a HEARTBEAT arrives at, or an error describing why none did.
pub fn find_heartbeat_baud(port: &str, bauds: &[u32], per_baud: Duration) -> Result<u32, String> {
    for &baud in bauds {
        if listen(port, baud, per_baud)? {
            return Ok(baud);
        }
    }
    Err(format!(
        "no MAVLink heartbeat from PX4 on {port} at {} baud",
        bauds.iter().map(u32::to_string).collect::<Vec<_>>().join("/")
    ))
}

fn speed(baud: u32) -> Option<libc::speed_t> {
    Some(match baud {
        57600 => libc::B57600,
        115200 => libc::B115200,
        230400 => libc::B230400,
        460800 => libc::B460800,
        500000 => libc::B500000,
        921600 => libc::B921600,
        1000000 => libc::B1000000,
        1500000 => libc::B1500000,
        2000000 => libc::B2000000,
        3000000 => libc::B3000000,
        _ => return None,
    })
}

fn listen(port: &str, baud: u32, dur: Duration) -> Result<bool, String> {
    let speed = speed(baud).ok_or_else(|| format!("unsupported baud {baud}"))?;
    let path = CString::new(port).map_err(|_| format!("bad port name {port}"))?;
    // SAFETY: plain POSIX calls on a descriptor we own and close below.
    unsafe {
        let fd = libc::open(path.as_ptr(), libc::O_RDWR | libc::O_NOCTTY | libc::O_NONBLOCK);
        if fd < 0 {
            return Err(format!("{port}: {}", std::io::Error::last_os_error()));
        }
        let mut tio: libc::termios = std::mem::zeroed();
        if libc::tcgetattr(fd, &mut tio) != 0 {
            let e = std::io::Error::last_os_error();
            libc::close(fd);
            return Err(format!("{port}: {e}"));
        }
        libc::cfmakeraw(&mut tio);
        tio.c_cflag |= libc::CLOCAL | libc::CREAD;
        tio.c_cflag &= !libc::CRTSCTS;
        libc::cfsetispeed(&mut tio, speed);
        libc::cfsetospeed(&mut tio, speed);
        if libc::tcsetattr(fd, libc::TCSANOW, &tio) != 0 {
            let e = std::io::Error::last_os_error();
            libc::close(fd);
            return Err(format!("{port}: {e}"));
        }
        libc::tcflush(fd, libc::TCIFLUSH);

        let mut data = Vec::new();
        let mut buf = [0u8; 512];
        let end = Instant::now() + dur;
        let mut found = false;
        while Instant::now() < end && !found {
            let n = libc::read(fd, buf.as_mut_ptr().cast(), buf.len());
            if n > 0 {
                data.extend_from_slice(&buf[..n as usize]);
                found = contains_heartbeat(&data);
            } else {
                std::thread::sleep(Duration::from_millis(20));
            }
        }
        libc::close(fd);
        Ok(found)
    }
}

/// MAVLink X.25 CRC, seeded and finished with HEARTBEAT's crc_extra (50).
fn crc(bytes: &[u8]) -> u16 {
    let mut acc: u16 = 0xFFFF;
    for &b in bytes.iter().chain(std::iter::once(&50u8)) {
        let mut tmp = b ^ (acc & 0xFF) as u8;
        tmp ^= tmp << 4;
        acc = (acc >> 8) ^ ((tmp as u16) << 8) ^ ((tmp as u16) << 3) ^ ((tmp as u16) >> 4);
    }
    acc
}

/// True if `data` holds a CRC-valid MAVLink v1 or v2 HEARTBEAT frame.
fn contains_heartbeat(data: &[u8]) -> bool {
    (0..data.len()).any(|i| match data[i] {
        // v2: STX len incompat compat seq sys comp msgid(3) payload crc(2)
        0xFD => {
            let Some(f) = data.get(i..) else { return false };
            if f.len() < 12 {
                return false;
            }
            let len = f[1] as usize;
            let total = 10 + len + 2 + if f[2] & 0x01 != 0 { 13 } else { 0 };
            f[7] == 0 && f[8] == 0 && f[9] == 0 && f.len() >= total && {
                let c = crc(&f[1..10 + len]);
                f[10 + len] == (c & 0xFF) as u8 && f[11 + len] == (c >> 8) as u8
            }
        }
        // v1: STX len seq sys comp msgid payload(9) crc(2)
        0xFE => {
            let f = &data[i..];
            f.len() >= 17 && f[1] == 9 && f[5] == 0 && {
                let c = crc(&f[1..15]);
                f[15] == (c & 0xFF) as u8 && f[16] == (c >> 8) as u8
            }
        }
        _ => false,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink::dialects::development::{HEARTBEAT_DATA, MavMessage};
    use mavlink::{MavHeader, MavlinkVersion};

    fn frame(version: MavlinkVersion) -> Vec<u8> {
        let mut out = Vec::new();
        let header = MavHeader { system_id: 1, component_id: 1, sequence: 7 };
        mavlink::write_versioned_msg(&mut out, version, header, &MavMessage::HEARTBEAT(HEARTBEAT_DATA::default())).unwrap();
        out
    }

    #[test]
    fn finds_v1_and_v2_heartbeats_amid_noise() {
        for v in [MavlinkVersion::V1, MavlinkVersion::V2] {
            let mut data = vec![0x00, 0xFD, 0x12, 0xFE, 0x33];
            data.extend(frame(v));
            data.extend([0xAA, 0x55]);
            assert!(contains_heartbeat(&data), "{v:?}");
        }
    }

    #[test]
    fn rejects_corrupt_or_truncated_frames() {
        let mut f = frame(MavlinkVersion::V2);
        let last = f.len() - 1;
        assert!(!contains_heartbeat(&f[..last]));
        f[last] ^= 0xFF;
        assert!(!contains_heartbeat(&f));
        assert!(!contains_heartbeat(&[0xFD; 64]));
        assert!(!contains_heartbeat(&[]));
    }
}
