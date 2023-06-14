mod raytracer_cpu;
pub use raytracer_cpu::*;

pub trait Renderer {
    fn new() -> Self;
}