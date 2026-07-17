//! Remote diagnostics: a small request/response command channel over MQTT.
//!
//! - [`transport`]: the MQTT plumbing — subscribe to the command topic, pull one
//!   queued command per loop, and publish a correlated reply on the ack topic.
//! - [`commands`]: the catalog of commands the tower answers. New `get_*`
//!   commands are added there; `transport` and `main.rs` stay untouched.

pub mod commands;
pub mod telemetry;
pub mod transport;
