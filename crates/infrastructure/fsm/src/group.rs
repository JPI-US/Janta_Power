//! Shared-thread scheduler for one or more [`crate::Runnable`] machines.

use std::{
    io,
    thread::{sleep, JoinHandle},
    time::{Duration, Instant},
};

use watchdog::UserWatchdog;

use crate::{FsmStatus, Runnable};

/// Co-schedules several machines on a single OS thread.
///
/// Each loop iteration steps every machine once, then sleeps so the iteration
/// lasts at least `min_thread_period`.
pub struct Group {
    name: String,
    thread_stack_size: usize,
    machines: Vec<Box<dyn Runnable>>,
    min_thread_period: Duration,
}

impl Group {
    /// Creates an empty group that will run under `name`.
    ///
    /// # Arguments
    ///
    /// * `name` - Thread name used when the group is spawned.
    /// * `thread_stack_size` - Stack size in bytes for the spawned thread.
    /// * `min_thread_period` - Minimum duration of each step-loop iteration.
    pub fn new(
        name: impl Into<String>,
        thread_stack_size: usize,
        min_thread_period: Duration,
    ) -> Group {
        Group {
            name: name.into(),
            thread_stack_size,
            machines: vec![],
            min_thread_period,
        }
    }

    /// Spawns the group's thread and runs the step loop until the process exits.
    ///
    /// [`FsmStatus::Running`] transitions are logged. [`FsmStatus::Hold`] is
    /// silent. [`FsmStatus::Stopped`] or a step error aborts the remainder of
    /// the current round-robin pass; the outer loop continues.
    ///
    /// # Errors
    ///
    /// Returns an [`io::Error`] if the OS thread cannot be spawned.
    pub fn spawn(mut self) -> Result<JoinHandle<()>, io::Error> {
        std::thread::Builder::new()
            .name(self.name.clone())
            .stack_size(self.thread_stack_size)
            .spawn(move || {
                log::info!("Thread \"{}\" started", self.name);

                let watchdog_name = format!("fsm_{}", self.name);
                let watchdog = match UserWatchdog::new(&watchdog_name) {
                    Ok(wd) => Some(wd),
                    Err(e) => {
                        log::error!("Failed to register watchdog: {e}");
                        None
                    }
                };

                loop {
                    let start = Instant::now();

                    for machine in &mut self.machines {
                        match machine.step() {
                            Ok(FsmStatus::Running) => {
                                log::info!(
                                    "{} transitioned to {}",
                                    machine.name(),
                                    machine.state()
                                );
                            }
                            Ok(FsmStatus::Hold) => {}
                            Ok(FsmStatus::Stopped) => break,
                            Err(e) => {
                                log::error!("FSM error: {e:?}");
                                break;
                            }
                        }
                    }

                    if let Some(wd) = &watchdog {
                        if let Err(e) = wd.feed() {
                            log::error!("Watchdog feed failed: {e}");
                        }
                    }

                    sleep(self.min_thread_period.saturating_sub(start.elapsed()));
                }
            })
    }

    /// Appends a machine to the round-robin set.
    ///
    /// # Arguments
    ///
    /// * `machine` - Runnable to step on each loop iteration.
    pub fn add(&mut self, machine: Box<dyn Runnable>) {
        self.machines.push(machine);
    }
}
