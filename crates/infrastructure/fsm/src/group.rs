use std::{
    io,
    thread::{sleep, JoinHandle},
    time::{Duration, Instant},
};

use crate::{FsmStatus, Runnable};

pub struct Group {
    name: String,
    thread_stack_size: usize,
    machines: Vec<Box<dyn Runnable>>,
    min_thread_period: Duration,
}

impl Group {
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

    pub fn spawn(mut self) -> Result<JoinHandle<()>, io::Error> {
        std::thread::Builder::new()
            .name(self.name.clone())
            .stack_size(self.thread_stack_size)
            .spawn(move || {
                log::info!("Thread \"{}\" started", self.name);

                loop {
                    let start = Instant::now();

                    for machine in &mut self.machines {
                        match machine.step() {
                            Ok(FsmStatus::Running) => {
                                log::info!(
                                    "{} Transitioned to {}",
                                    machine.name(),
                                    machine.state()
                                );
                            }
                            Ok(FsmStatus::Hold) => {}
                            Ok(FsmStatus::Stopped) => break,
                            Err(e) => {
                                log::error!("FSM thread exited: {e:?}");
                                break;
                            }
                        }
                    }

                    sleep(self.min_thread_period.saturating_sub(start.elapsed()));
                }
            })
    }

    pub fn add(&mut self, machine: Box<dyn Runnable>) {
        self.machines.push(machine);
    }
}
