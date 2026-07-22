use crossbeam_channel::{Receiver, Sender, TryRecvError, TrySendError};

/// Communication channel used by an FSM.
///
/// The channel wraps a sender and receiver and provides non-blocking send and
/// receive operations.
///
/// Incoming commands can be automatically pruned so that stale queued commands
/// are discarded, allowing states to process only the most recent activity when
/// appropriate.
pub struct Channel<Cmd> {
    pub(crate) tx: Sender<Cmd>,
    pub(crate) rx: Receiver<Cmd>,
    pub(crate) max_pending: usize,
}

impl<Cmd> Channel<Cmd> {
    /// Creates a new communication channel.
    ///
    /// `max_pending` specifies the maximum number of queued received commands
    /// that may be retained before older commands are discarded.
    pub fn new(tx: Sender<Cmd>, rx: Receiver<Cmd>, max_pending: usize) -> Self {
        Self {
            tx,
            rx,
            max_pending,
        }
    }

    pub fn drain(&self) {
        while self.rx.len() > self.max_pending {
            let _ = self.rx.try_recv();
        }
    }

    pub fn drain_to(&self, to: usize) {
        while self.rx.len() > to {
            let _ = self.rx.try_recv();
        }
    }

    /// Receives the next pending command.
    ///
    /// Before attempting to receive, older queued commands are discarded until
    /// at most `max_pending` commands remain.
    ///
    /// Returns `TryRecvError::Empty` if no command is available.
    pub fn recv(&self) -> Result<Cmd, TryRecvError> {
        self.drain();
        self.rx.try_recv()
    }

    /// Receives only the most recently queued command.
    ///
    /// Any older queued commands are discarded before attempting to receive,
    /// ensuring that at most one pending command remains.
    ///
    /// Returns `TryRecvError::Empty` if no command is available.
    pub fn recv_latest(&self) -> Result<Cmd, TryRecvError> {
        self.drain_to(1);
        self.rx.try_recv()
    }

    /// Attempts to send a command without blocking.
    ///
    /// Returns `TrySendError` if the command cannot be queued.
    pub fn send(&self, cmd: Cmd) -> Result<(), TrySendError<Cmd>> {
        self.tx.try_send(cmd)
    }
}
