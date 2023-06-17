use std::collections::VecDeque;

pub struct HandleQueue<T> {
    handles: VecDeque<T>
}

impl<T: From<usize>> HandleQueue<T> {
    pub fn new(capacity: usize) -> Self {
        let mut handles = VecDeque::with_capacity(capacity);
        for i in 0..capacity {
            handles.push_back(i.into());
        }

        HandleQueue {
            handles
        }
    }

    pub fn pop(&mut self) -> T {
        self.handles.pop_front()
            .expect("Failed to get handle.")
    }

    pub fn push(&mut self, handle: T) {
        self.handles.push_back(handle);
    }
}