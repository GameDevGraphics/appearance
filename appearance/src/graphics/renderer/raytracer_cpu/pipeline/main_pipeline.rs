use glam::*;
use super::*;

#[derive(Clone, Copy, Debug, Default)]
pub struct Payload {
    pub color: Vec3
}

#[derive(Default)]
pub struct MainPipeline;

impl PipelineLayout<Payload, Framebuffer> for MainPipeline {
    fn ray_gen(&self, camera: &CameraMatrices, uv: &Vec2, payload: &mut Payload) -> Ray {
        let target = camera.proj_inv_matrix * Vec4::new(uv.x, uv.y, 1.0, 1.0);

        let origin = (camera.view_inv_matrix * Vec4::new(0.0, 0.0, 0.0, 1.0)).xyz();
        let direction = (camera.view_inv_matrix * Vec4::from((target.xyz().normalize(), 0.0))).xyz();

        Ray::new(&origin, &direction)
    }

    fn chit(&self, ray: Ray, intersection: Intersection, payload: &mut Payload) {
        let color = Vec3::ONE.lerp(Vec3::ZERO, intersection.t * 0.03);
        payload.color = color;
    }

    fn miss(&self, ray: Ray, payload: &mut Payload) {
        let t = ray.direction().y * 0.5 + 0.5;
        payload.color = Vec3::new(1.0, 1.0, 1.0).lerp(Vec3::new(0.5, 0.7, 1.0), t);
    }

    fn present(&self, payload: Payload, uv: &IVec2, render_target: &mut Framebuffer) {
        render_target.set_pixel(uv.x as u32, uv.y as u32, &payload.color);
    }
}