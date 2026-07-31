//! Point-to-point, non-blocking message queue keyed by [`crate::postal::Address`].

use core::marker::PhantomData;
use std::sync::Arc;

use crossbeam_channel::{Receiver, Sender, TryRecvError, TrySendError};

use crate::postal::Address;

/// Send/receive endpoint for one FSM.
///
/// Each mailbox owns its inbox receiver and a shared table of senders so it
/// can address any peer created from the same [`super::Postal`].
pub struct Mailbox<A, M>
where
    A: Address,
{
    receiver: Receiver<M>,
    routes: Arc<Vec<Sender<M>>>,
    capacity: usize,
    _address: PhantomData<A>,
}

impl<A, M> Mailbox<A, M>
where
    A: Address,
    M: Send + 'static,
{
    /// Creates a mailbox bound to `receiver` with shared `routes`.
    ///
    /// # Arguments
    ///
    /// * `receiver` - Inbox for this mailbox.
    /// * `routes` - Shared sender table indexed by [`Address::index`].
    /// * `capacity` - Soft depth enforced by [`Self::drain`].
    pub(crate) fn new(receiver: Receiver<M>, routes: Arc<Vec<Sender<M>>>, capacity: usize) -> Self {
        Self {
            receiver,
            routes,
            capacity,
            _address: PhantomData,
        }
    }

    /// Tries to enqueue `message` for `address` without blocking.
    ///
    /// Channels are unbounded; failure means the receiver was dropped.
    ///
    /// # Arguments
    ///
    /// * `address` - Destination FSM address.
    /// * `message` - Message to deliver.
    ///
    /// # Errors
    ///
    /// Returns [`TrySendError`] if the destination channel is disconnected.
    pub fn send(&self, address: A, message: M) -> Result<(), TrySendError<M>> {
        self.routes[address.index()].try_send(message)
    }

    /// Tries to dequeue the next pending message without blocking.
    ///
    /// # Errors
    ///
    /// Returns [`TryRecvError`] if the inbox is empty or disconnected.
    pub fn receive(&self) -> Result<M, TryRecvError> {
        self.receiver.try_recv()
    }

    /// Discards all but the newest queued message, then returns that message.
    ///
    /// Useful when only the latest command matters (e.g. button events).
    ///
    /// # Errors
    ///
    /// Returns [`TryRecvError`] if the inbox is empty or disconnected.
    pub fn receive_latest(&self) -> Result<M, TryRecvError> {
        while self.receiver.len() > 1 {
            let _ = self.receiver.try_recv();
        }

        self.receiver.try_recv()
    }

    /// Drops oldest messages until at most the configured capacity remain.
    pub fn drain(&self) {
        while self.receiver.len() > self.capacity {
            let _ = self.receiver.try_recv();
        }
    }
}
