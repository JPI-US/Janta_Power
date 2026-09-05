//! Shared-thread scheduler for one or more [`crate::Runnable`] machines.

use std::{
    io,
    thread::{sleep, JoinHandle},
    time::{Duration, Instant},
};

use crate::{FsmStatus, Runnable};

/// Co-schedules several machines on a single OS thread.
///
/// Each loop iteration steps every machine once, then sleeps so the iteration
/// lasts at least `min_thread_period`. Machines that return
/// [`FsmStatus::Stopped`] are removed from the group. The group's thread
/// exits when no machines remain.
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

    /// Spawns the group's thread and runs the step loop until no machines remain.
    ///
    /// [`FsmStatus::Running`] transitions are logged. [`FsmStatus::Hold`] is
    /// silent. Machines returning [`FsmStatus::Stopped`] are removed from the
    /// group and will not be stepped again. Step errors are logged, but the
    /// machine remains in the group and will be stepped again on the next
    /// iteration.
    ///
    /// The spawned thread exits when all machines have returned
    /// [`FsmStatus::Stopped`].
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

                loop {
                    let start = Instant::now();

                    self.machines.retain_mut(|machine| match machine.step() {
                        Ok(FsmStatus::Running) => {
                            log::info!("{} Transitioned to {}", machine.name(), machine.state());
                            true
                        }
                        Ok(FsmStatus::Hold) => true,
                        Ok(FsmStatus::Stopped) => {
                            log::info!("{} Stopped", machine.name());
                            false
                        }
                        Err(e) => {
                            log::error!("{} returned error: {e:?}", machine.name());
                            true
                        }
                    });

                    if self.machines.is_empty() {
                        log::info!("Thread \"{}\" stopped: no machines remaining", self.name);
                        break;
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
