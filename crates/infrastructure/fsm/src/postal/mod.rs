//! Inter-FSM messaging: addresses, mailboxes, and a shared bulletin board.

use std::sync::Arc;

use crossbeam_channel::unbounded;

use crate::postal::{bulletin::Bulletin, mailbox::Mailbox};

pub mod bulletin;
pub mod mailbox;

/// Routing key for [`Mailbox`] delivery.
///
/// Implementors map each variant to a dense `0..count()` index used as a
/// slot into the postal route table.
pub trait Address: Copy + Send + 'static {
    /// Index of this address in the postal route table.
    fn index(self) -> usize;

    /// Number of distinct addresses (size of the route table).
    fn count() -> usize;
}

/// Factory for per-address [`Mailbox`]es plus one shared [`Bulletin`].
///
/// Create with [`Postal::new`], then [`Postal::take`] each address exactly
/// once when constructing FSMs.
pub struct Postal<A, M, B>
where
    A: Address,
{
    mailboxes: Vec<Option<Mailbox<A, M>>>,
    bulletin: Arc<Bulletin<B>>,
}

impl<A, M, B> Postal<A, M, B>
where
    A: Address,
    M: Send + 'static,
    B: Send + 'static,
{
    /// Builds a mailbox for every [`Address`] and an empty shared bulletin.
    ///
    /// # Arguments
    ///
    /// * `capacity` - Soft inbox depth enforced by [`Mailbox::drain`]: older
    ///   messages are dropped until at most this many remain.
    pub fn new(capacity: usize) -> Self {
        let mut senders = Vec::with_capacity(A::count());
        let mut receivers = Vec::with_capacity(A::count());

        for _ in 0..A::count() {
            let (tx, rx) = unbounded();
            senders.push(tx);
            receivers.push(rx);
        }

        let routes = Arc::new(senders);

        let mailboxes = receivers
            .into_iter()
            .map(|rx| Some(Mailbox::new(rx, Arc::clone(&routes), capacity)))
            .collect();

        Self {
            mailboxes,
            bulletin: Arc::new(Bulletin::new()),
        }
    }

    /// Takes the mailbox for `address` and a clone of the shared bulletin.
    ///
    /// # Arguments
    ///
    /// * `address` - Address whose mailbox should be claimed.
    ///
    /// # Returns
    ///
    /// The mailbox for `address` and a clone of the shared [`Bulletin`].
    ///
    /// # Panics
    ///
    /// Panics if the mailbox for `address` was already taken.
    pub fn take(&mut self, address: A) -> (Mailbox<A, M>, Arc<Bulletin<B>>) {
        let mailbox = self.mailboxes[address.index()]
            .take()
            // we intentionally panic here. Mailboxes should only be gotten at startup,
            // and it's never acceptable to get the same mailbox twice.
            .expect("Mailbox already taken");

        (mailbox, Arc::clone(&self.bulletin))
    }
}
