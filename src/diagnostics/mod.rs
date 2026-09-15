//! Remote diagnostics: a small request/response command channel over MQTT.
//!
//! - [`transport`]: the MQTT plumbing — subscribe to the command topic, pull one
//!   queued command per loop, and publish a correlated reply on the ack topic.
//! - [`commands`]: the catalog of read-only commands the tower answers. New
//!   `get_*` commands are added there; `transport` and `main.rs` stay untouched.
//! - [`motion_commands`]: the catalog of commands that move the tower, used
//!   during installation. Separate from [`commands`] so the read-only handlers
//!   never gain mutable access to the motor.

pub mod commands;
pub mod motion_commands;
pub mod transport;
