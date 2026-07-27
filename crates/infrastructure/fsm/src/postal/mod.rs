use std::sync::Arc;

use crossbeam_channel::unbounded;

use crate::postal::{bulletin::Bulletin, mailbox::Mailbox};

pub mod bulletin;
pub mod mailbox;

pub trait Address: Copy + Send + 'static {
    fn index(self) -> usize;
    fn count() -> usize;
}

pub struct Postal<A, M, B>
where
    A: Address,
{
    mailboxes: Vec<Option<Mailbox<A, M>>>,
    bulletins: Vec<Option<Bulletin<B>>>,
}

impl<A, M, B> Postal<A, M, B>
where
    A: Address,
    M: Send + 'static,
    B: Send + 'static,
{
    pub fn new(capacity: usize) -> Self {
        let mut senders = Vec::with_capacity(A::count());
        let mut receivers = Vec::with_capacity(A::count());

        for _ in 0..A::count() {
            let (tx, rx) = unbounded();
            senders.push(tx);
            receivers.push(rx);
        }

        let routes = Arc::new(senders);

        let bulletins = (0..A::count()).map(|_| Some(Bulletin::new())).collect();

        let mailboxes = receivers
            .into_iter()
            .map(|rx| Some(Mailbox::new(rx, Arc::clone(&routes), capacity)))
            .collect();

        Self {
            mailboxes,
            bulletins,
        }
    }

    pub fn take(&mut self, address: A) -> (Mailbox<A, M>, Bulletin<B>) {
        let mailbox = self.mailboxes[address.index()]
            .take()
            .expect("Mailbox already taken");

        let bulletin = self.bulletins[address.index()]
            .take()
            .expect("Bulletin already taken");

        (mailbox, bulletin)
    }
}
