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
    bulletin: Arc<Bulletin<B>>,
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

        let mailboxes = receivers
            .into_iter()
            .map(|rx| Some(Mailbox::new(rx, Arc::clone(&routes), capacity)))
            .collect();

        Self {
            mailboxes,
            bulletin: Arc::new(Bulletin::new()),
        }
    }

    pub fn take(&mut self, address: A) -> (Mailbox<A, M>, Arc<Bulletin<B>>) {
        let mailbox = self.mailboxes[address.index()]
            .take()
            .expect("Mailbox already taken");

        (mailbox, Arc::clone(&self.bulletin))
    }
}
