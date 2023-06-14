use std::rc::Rc;
use crate::Model;

mod raytracer_cpu;
pub use raytracer_cpu::*;

pub trait Renderer: private::Renderer {
    fn add_model(&mut self, model: Rc<Model>);
}

pub(crate) mod private {
    use crate::{Camera, Window};

    pub trait Renderer {
        fn resize(&mut self, width: u32, height: u32);
        fn render(&mut self, window: &Window, camera: &Camera);
    }
}