use std::rc::Rc;
use crate::Model;
use crate::Camera;

mod raytracer_cpu;
pub use raytracer_cpu::*;

pub trait Renderer: private::Renderer {
    fn add_model(&mut self, model: Rc<Model>);
}

pub(crate) mod private {
    pub trait Renderer {
        fn resize(&mut self, width: u32, height: u32);
        fn render(&mut self, camera: &crate::Camera);
    }
}