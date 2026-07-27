use core::default::Default;
use std::sync::Mutex;

#[derive(Default, Debug)]
pub struct Bulletin<T> {
    value: Mutex<Option<T>>,
}

impl<T> Bulletin<T> {
    pub fn new() -> Self {
        Self {
            value: Mutex::new(None),
        }
    }

    pub fn post(&self, value: T) {
        *self.value.lock().unwrap() = Some(value);
    }

    pub fn clear(&self) {
        *self.value.lock().unwrap() = None;
    }

    pub fn take(&self) -> Option<T> {
        self.value.lock().unwrap().take()
    }

    pub fn is_posted(&self) -> bool {
        self.value.lock().unwrap().is_some()
    }

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
    pub fn read(&self) -> Option<T> {
        self.value.lock().unwrap().clone()
    }
}
