use core::marker::PhantomData;
use std::sync::Arc;

use crossbeam_channel::{Receiver, Sender, TryRecvError, TrySendError};

use crate::postal::Address;

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
    pub(crate) fn new(receiver: Receiver<M>, routes: Arc<Vec<Sender<M>>>, capacity: usize) -> Self {
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
