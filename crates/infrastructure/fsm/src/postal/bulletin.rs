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

    /// Replaces the posted value with `value`.
    ///
    /// # Arguments
    ///
    /// * `value` - Value to post, replacing any previous contents.
    pub fn post(&self, value: T) {
        *self.value.lock().unwrap() = Some(value);
    }

    /// Clears the bulletin back to `None`.
    pub fn clear(&self) {
        *self.value.lock().unwrap() = None;
    }

    /// Removes and returns the posted value, leaving `None`.
    pub fn take(&self) -> Option<T> {
        self.value.lock().unwrap().take()
    }

    /// Returns whether a value is currently posted.
    pub fn is_posted(&self) -> bool {
        self.value.lock().unwrap().is_some()
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
        let mut guard = self.value.lock().unwrap();

        let value = guard.get_or_insert_with(T::default);
        f(value);
    }
}

impl<T: Clone> Bulletin<T> {
    /// Returns a clone of the posted value without clearing it.
    pub fn read(&self) -> Option<T> {
        self.value.lock().unwrap().clone()
    }
}
