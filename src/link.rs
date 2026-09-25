//! The MAVLink transport: a `udpin` socket, like pymavlink's
//! `mavlink_connection("udpin:0.0.0.0:PORT")`.
//!
//! We listen on the port, remember the address the vehicle last talked to
//! us from, and send everything back there. Our own socket (rather than
//! rust-mavlink's `connect()`) gives us a read timeout for a clean shutdown,
//! and lets one datagram carry several MAVLink frames.

use std::io::{self, ErrorKind};
use std::net::{SocketAddr, UdpSocket};
use std::sync::Mutex;
use std::sync::atomic::{AtomicU8, Ordering};
use std::time::Duration;

use mavlink::dialects::development::{self as mav, MavMessage};
use mavlink::peek_reader::PeekReader;
use mavlink::{MavHeader, MavlinkVersion};

/// We announce ourselves as a ground station: system 255 (the pymavlink /
/// QGC convention) and MAV_COMP_ID_MISSIONPLANNER.
const GCS_SYSTEM_ID: u8 = 255;
const GCS_COMPONENT_ID: u8 = 190;

pub struct Link {
    socket: UdpSocket,
    peer: Mutex<Option<SocketAddr>>,
    sequence: AtomicU8,
    /// Locked vehicle identity (system, component), copied here so a sender
    /// never needs the state lock just to address a packet.
    target: Mutex<(u8, u8)>,
}

impl Link {
    pub fn bind(port: u16) -> io::Result<Self> {
        let socket = UdpSocket::bind(("0.0.0.0", port))?;
        socket.set_read_timeout(Some(Duration::from_millis(500)))?;
        Ok(Link {
            socket,
            peer: Mutex::new(None),
            sequence: AtomicU8::new(0),
            target: Mutex::new((0, 0)),
        })
    }

    pub fn set_target(&self, system: u8, component: u8) {
        *self.target.lock().unwrap_or_else(|e| e.into_inner()) = (system, component);
    }

    pub fn target(&self) -> (u8, u8) {
        *self.target.lock().unwrap_or_else(|e| e.into_inner())
    }

    pub fn has_peer(&self) -> bool {
        self.peer.lock().unwrap_or_else(|e| e.into_inner()).is_some()
    }

    /// Block (up to the read timeout) for one datagram and decode every
    /// frame in it. `Ok(vec![])` on timeout; frames of messages the
    /// dialect doesn't know (or with out-of-range enum values) are skipped.
    pub fn recv(&self) -> io::Result<Vec<(MavHeader, MavMessage)>> {
        let mut buf = [0u8; 2048];
        let (n, from) = match self.socket.recv_from(&mut buf) {
            Ok(v) => v,
            Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => {
                return Ok(Vec::new());
            }
            Err(e) => return Err(e),
        };

        *self.peer.lock().unwrap_or_else(|e| e.into_inner()) = Some(from);

        let mut reader = PeekReader::new(&buf[..n]);
        let mut out = Vec::new();
        loop {
            match mavlink::read_any_msg::<MavMessage, _>(&mut reader) {
                Ok(frame) => out.push(frame),
                Err(mavlink::error::MessageReadError::Io(_)) => break,
                // Parse errors consume the bad frame; keep going.
                Err(_) => continue,
            }
        }
        Ok(out)
    }

    /// Send one message to the vehicle. Returns false (never panics) when
    /// no peer is known yet or the socket write fails.
    pub fn send(&self, msg: &MavMessage) -> bool {
        let Some(peer) = *self.peer.lock().unwrap_or_else(|e| e.into_inner()) else {
            return false;
        };
        let header = MavHeader {
            system_id: GCS_SYSTEM_ID,
            component_id: GCS_COMPONENT_ID,
            sequence: self.sequence.fetch_add(1, Ordering::Relaxed),
        };
        let mut bytes = Vec::with_capacity(280);
        if mavlink::write_versioned_msg(&mut bytes, MavlinkVersion::V2, header, msg).is_err() {
            return false;
        }
        self.socket.send_to(&bytes, peer).is_ok()
    }

    /// COMMAND_LONG to the locked vehicle.
    pub fn command_long(&self, command: mav::MavCmd, params: [f32; 7]) -> bool {
        let (target_system, target_component) = self.target();
        self.send(&MavMessage::COMMAND_LONG(mav::COMMAND_LONG_DATA {
            param1: params[0],
            param2: params[1],
            param3: params[2],
            param4: params[3],
            param5: params[4],
            param6: params[5],
            param7: params[6],
            command,
            target_system,
            target_component,
            confirmation: 0,
        }))
    }
}

/// Encode a parameter / text id into MAVLink's fixed, NUL-padded char array.
pub fn char_array<const N: usize>(text: &str) -> mavlink::types::CharArray<N> {
    let mut raw = [0u8; N];
    for (dst, src) in raw.iter_mut().zip(text.bytes()) {
        *dst = src;
    }
    raw.into()
}

/// Decode a NUL-padded MAVLink char array.
pub fn from_char_array(bytes: &[u8]) -> String {
    let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    String::from_utf8_lossy(&bytes[..end]).trim().to_string()
}
