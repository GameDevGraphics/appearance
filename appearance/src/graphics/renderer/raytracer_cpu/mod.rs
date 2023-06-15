use glam::*;
use super::*;
use crate::{Window, Camera};

mod framebuffer;
use framebuffer::*;
mod ray;
use ray::*;
mod mesh;
use mesh::*;
mod acceleration_structure;
use acceleration_structure::*;

pub struct RaytracerCPU {
    framebuffer: Framebuffer,
    meshes: Vec<Mesh>
}

impl RaytracerCPU {
    pub fn new(window: &Window) -> Self {
        gl_helper::gl_init(window.internal_context());
        let framebuffer = Framebuffer::new(window.get_width(), window.get_height());

        RaytracerCPU {
            framebuffer,
            meshes: Vec::new()
        }
    }
}

impl Renderer for RaytracerCPU {
    fn add_model(&mut self, model: Rc<Model>) {
        self.meshes.push(Mesh::new(model.root_nodes[0].mesh.as_ref().unwrap()));
    }
}

impl private::Renderer for RaytracerCPU {
    fn resize(&mut self, width: u32, height: u32) {
        self.framebuffer = Framebuffer::new(width, height);
    }

    fn render(&mut self, window: &Window, camera: &mut Camera) {
        let width = self.framebuffer.width();
        let height = self.framebuffer.height();

        let origin = *camera.view_inv_matrix() * Vec4::new(0.0, 0.0, 0.0, 1.0);

        for x in 0..width {
            for y in 0..height {
                let pixel_center = Vec2::new(x as f32, y as f32) + Vec2::splat(0.5);
                let uv = (pixel_center / Vec2::new(width as f32, height as f32)) * 2.0 - 1.0;
                let target = *camera.proj_inv_matrix() * Vec4::new(uv.x, uv.y, 1.0, 1.0);
                let direction = *camera.view_inv_matrix() * Vec4::from((target.xyz().normalize() * Vec3::new(-1.0, -1.0, 1.0), 0.0));

                let ray = Ray::new(&origin.xyz(), &direction.xyz());

                if let Some(hit) = self.meshes[0].intersect(&ray, 0.01, 100.0) {
                    self.framebuffer.set_pixel(x, y, &Vec3::new(1.0, 1.0, hit.t * 0.2));
                } else {
                    self.framebuffer.set_pixel(x, y, &Vec3::new(0.0, 0.0, 0.0));
                }
            }
        }

        self.framebuffer.display(window);
    }
}