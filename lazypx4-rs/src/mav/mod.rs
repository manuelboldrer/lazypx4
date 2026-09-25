//! Everything that speaks MAVLink: the receiver thread and handlers, plus
//! the command senders for each protocol the console uses.

pub mod calibration;
pub mod commands;
pub mod flightlog;
pub mod guided;
pub mod handlers;
pub mod modes;
pub mod params;
pub mod shell;
