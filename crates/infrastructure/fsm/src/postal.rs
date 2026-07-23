use core::marker::PhantomData;
use std::sync::Arc;

use crossbeam_channel::{bounded, Receiver, Sender, TryRecvError, TrySendError};

pub trait Address: Copy + Send + 'static {
    fn index(self) -> usize;
    fn count() -> usize;
}

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
    fn new(receiver: Receiver<M>, routes: Arc<Vec<Sender<M>>>, capacity: usize) -> Self {
        Self {
            receiver,
            routes,
            capacity,
            _address: PhantomData,
        }
    }

    pub fn send(&self, address: A, message: M) -> Result<(), TrySendError<M>> {
        self.routes[address.index()].try_send(message)
    }

    pub fn receive(&self) -> Result<M, TryRecvError> {
        self.drain();
        self.receiver.try_recv()
    }

    pub fn receive_latest(&self) -> Result<M, TryRecvError> {
        while self.receiver.len() > 1 {
            let _ = self.receiver.try_recv();
        }

        self.receiver.try_recv()
    }

    fn drain(&self) {
        while self.receiver.len() > self.capacity {
            let _ = self.receiver.try_recv();
        }
    }
}

pub struct Postal<A, M>
where
    A: Address,
{
    mailboxes: Vec<Option<Mailbox<A, M>>>,
}

impl<A, M> Postal<A, M>
where
    A: Address,
    M: Send + 'static,
{
    pub fn new(capacity: usize) -> Self {
        let mut senders = Vec::with_capacity(A::count());
        let mut receivers = Vec::with_capacity(A::count());

        for _ in 0..A::count() {
            let (tx, rx) = bounded(capacity);
            senders.push(tx);
            receivers.push(rx);
        }

        let routes = Arc::new(senders);

        let mailboxes = receivers
            .into_iter()
            .map(|rx| Some(Mailbox::new(rx, Arc::clone(&routes), capacity)))
            .collect();

        Self { mailboxes }
    }

    pub fn take(&mut self, address: A) -> Mailbox<A, M> {
        self.mailboxes[address.index()]
            .take()
            .expect("Mailbox already taken")
    }
}
