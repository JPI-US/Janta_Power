//! Shared, replaceable notice board readable by every FSM from one [`super::Postal`].

use core::default::Default;
use std::sync::Mutex;

/// Thread-safe slot for a single shared value of type `T`.
///
/// Unlike a [`super::mailbox::Mailbox`], a bulletin is broadcast-style: any
/// machine holding the `Arc` can post, clear, or read the same value.
#[derive(Default, Debug)]
pub struct Bulletin<T> {
    value: Mutex<Option<T>>,
}

impl<T> Bulletin<T> {
    /// Creates an empty bulletin (`None`).
    pub fn new() -> Self {
        Self {
            value: Mutex::new(None),
        }
    }

    /// Mutates the posted value in place, inserting `T::default()` if empty.
    ///
    /// # Arguments
    ///
    /// * `f` - Closure that receives a mutable reference to the posted value.
    pub fn update<F>(&self, f: F)
    where
        F: FnOnce(&mut T),
        T: Default,
    {
        let mut guard = self.value.lock().unwrap_or_else(|e| e.into_inner());

        let value = guard.get_or_insert_with(T::default);
        f(value);
    }
}

impl<T: Clone> Bulletin<T> {
    /// Returns a clone of the posted value without clearing it.
    pub fn read(&self) -> Option<T> {
        let guard = self.value.lock().unwrap_or_else(|e| e.into_inner());

        guard.clone()
    }
}
