use glam::*;
use super::*;

#[derive(Clone, Copy, Debug, Default)]
pub struct Payload {
    pub color: Vec3
}

#[derive(Default)]
pub struct MainPipeline {
    pub meshes: Vec<Arc<Mesh>>,
    pub transforms: Vec<Mat4>
}

impl PipelineLayout<Payload, Framebuffer> for MainPipeline {
    fn ray_gen(&self, camera: &CameraMatrices, uv: &Vec2, payload: &mut Payload) -> Ray {
        let target = camera.proj_inv_matrix * Vec4::new(uv.x, uv.y, 1.0, 1.0);

        let origin = (camera.view_inv_matrix * Vec4::new(0.0, 0.0, 0.0, 1.0)).xyz();
        let direction = (camera.view_inv_matrix * Vec4::from((target.xyz().normalize(), 0.0))).xyz();

        Ray::new(&origin, &direction)
    }

    fn chit(&self, ray: Ray, intersection: Intersection, payload: &mut Payload) {
        //let color = Vec3::ONE.lerp(Vec3::ZERO, intersection.t * 0.03);
        // let n = intersection.normal;
        // let l = Vec3::new(0.3, -1.0, 0.1);
        // let attenuation = n.dot(-l);

        // let color = Vec3::ONE.lerp(Vec3::ZERO, attenuation);
        let mesh = &self.meshes[intersection.blas as usize];
        let material = mesh.material();
        let vertices = &mesh.mesh_data().vertices;

        let v0 = &vertices[intersection.indices.x as usize];
        let v1 = &vertices[intersection.indices.y as usize];
        let v2 = &vertices[intersection.indices.z as usize];

        let bary_coords = Vec3::new(1.0 - intersection.uv.x - intersection.uv.y, intersection.uv.x, intersection.uv.y);
        let n = v0.normal * bary_coords.x + v1.normal * bary_coords.y + v2.normal * bary_coords.z;
        let tex_coord = v0.tex_coord * bary_coords.x + v1.tex_coord * bary_coords.y + v2.tex_coord * bary_coords.z;
        
        let mut base_color = material.base_color_factor.xyz();
        if let Some(base_color_texture) = &material.base_color_texture {
            base_color *= base_color_texture.sample_pixel(&tex_coord).xyz();
        }

        let l = Vec3::new(0.3, -1.0, 0.1);
        let attenuation = n.dot(-l).clamp(0.0, 1.0);
        payload.color = base_color * attenuation;
    }

    fn miss(&self, ray: Ray, payload: &mut Payload) {
        let t = ray.direction().y * 0.5 + 0.5;
        payload.color = Vec3::new(1.0, 1.0, 1.0).lerp(Vec3::new(0.5, 0.7, 1.0), t);
    }

    fn present(&self, payload: Payload, uv: &IVec2, render_target: &mut Framebuffer) {
        render_target.set_pixel(uv.x as u32, uv.y as u32, &payload.color);
    }
}