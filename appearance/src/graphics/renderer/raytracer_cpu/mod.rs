use super::*;
mod framebuffer;
use framebuffer::*;
use crate::{Window, Camera};

use glam::*;

pub struct RaytracerCPU {
    framebuffer: Framebuffer
}

impl RaytracerCPU {
    pub fn new(window: &Window) -> Self {
        gl_helper::gl_init(window.internal_context());
        let framebuffer = Framebuffer::new(window.get_width(), window.get_height());

        RaytracerCPU {
            framebuffer
        }
    }
}

impl Renderer for RaytracerCPU {
    fn add_model(&mut self, model: Rc<Model>) {

    }
}

impl private::Renderer for RaytracerCPU {
    fn resize(&mut self, width: u32, height: u32) {
        self.framebuffer = Framebuffer::new(width, height);
    }

    fn render(&mut self, window: &Window, camera: &Camera) {
        let width = self.framebuffer.width();
        let height = self.framebuffer.height();

        for x in 0..width {
            for y in 0..height {
                self.framebuffer.set_pixel(x, y, &Vec3::new(x as f32 / width as f32, y as f32 / height as f32, 0.0));
            }
        }

        self.framebuffer.display(window);
    }
}