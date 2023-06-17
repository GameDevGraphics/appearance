use std::collections::HashMap;
use uuid::Uuid;

use glam::*;
use super::*;
use crate::{Window, Camera};

mod framebuffer;
use framebuffer::*;
mod ray;
use ray::*;
mod mesh;
use mesh::*;
mod bvh;
use bvh::*;

pub struct RaytracerCPU {
    framebuffer: Framebuffer,

    mesh_handles: HandleQueue<MeshRendererID>,                  // Handle queue to hand out handles
    mesh_instances: HashMap<Rc<MeshRendererID>, MeshRenderer>,  // Every handle pointing to its instance
    meshes: HashMap<Uuid, (Mesh, Vec<Rc<MeshRendererID>>)>      // Every mesh id with all its instances
}

impl RaytracerCPU {
    pub(crate) fn new(window: &Window) -> Self {
        gl_helper::gl_init(window.internal_context());
        let framebuffer = Framebuffer::new(window.get_width(), window.get_height());

        RaytracerCPU {
            framebuffer,
            mesh_handles: HandleQueue::new(100_000),
            mesh_instances: HashMap::new(),
            meshes: HashMap::new(),
        }
    }

    fn update(&mut self) {
        let mut mesh_ids_to_remove = Vec::new();
        for mesh in &self.mesh_instances {
            if Rc::strong_count(mesh.0) <= 2 {
                mesh_ids_to_remove.push((
                    mesh.0.clone(), mesh.1.get_id()
                ));
            }
        }
        for mesh_id in mesh_ids_to_remove {
            self.mesh_instances.remove(&mesh_id.0);
            let meshes = &mut self.meshes.get_mut(&mesh_id.1).unwrap().1;
            meshes.remove(meshes.iter().position(|x| *x == mesh_id.0).unwrap());

            self.mesh_handles.push(*mesh_id.0.as_ref());
        }
    }
}

impl Renderer for RaytracerCPU {
    fn add_mesh(&mut self, mesh: &crate::Mesh) -> Rc<MeshRendererID> {
        // Add mesh if this is the first instance
        let mesh_renderer_ids = match self.meshes.get_mut(&mesh.get_id()) {
            Some(mesh_renderer_ids) => {
                mesh_renderer_ids
            },
            None => {
                let mesh_internal = Mesh::new(mesh);
                self.meshes.insert(mesh.get_id(), (mesh_internal, vec![]));
                self.meshes.get_mut(&mesh.get_id()).unwrap()
            }
        };

        // Add mesh renderer instance to the mesh
        let mesh_id = Rc::new(self.mesh_handles.pop());
        let mesh_renderer = MeshRenderer::new(mesh.get_id());
        mesh_renderer_ids.1.push(mesh_id.clone());
        self.mesh_instances.insert(mesh_id.clone(), mesh_renderer);
        
        mesh_id
    }

    fn mesh_renderer(&mut self, id: Rc<MeshRendererID>) -> &mut MeshRenderer {
        self.mesh_instances.get_mut(&id)
            .expect("Failed to get mesh renderer.")
    }
}

impl private::Renderer for RaytracerCPU {
    fn resize(&mut self, width: u32, height: u32) {
        self.framebuffer = Framebuffer::new(width, height);
    }

    fn render(&mut self, window: &Window, camera: &mut Camera) {
        self.update();

        let width = self.framebuffer.width();
        let height = self.framebuffer.height();

        let origin = *camera.view_inv_matrix() * Vec4::new(0.0, 0.0, 0.0, 1.0);

        let mut blases = Vec::new();
        let mut blas_instances = Vec::new();
        for mesh in &mut self.meshes {
            let instances = &mesh.1.1;
            let mesh = &mut mesh.1.0;

            mesh.animate();

            blases.push(mesh.blas());
            let blas_idx = (blases.len() - 1) as u32;
            
            for instance in instances {
                let instance_transform = &mut self.mesh_instances.get_mut(instance).unwrap().transform;
                blas_instances.push(BLASInstace::new(
                    *instance_transform.get_inv_model_matrix(),
                    blas_idx,
                    &blases
                ));
            }
        }
        let mut tlas = TLAS::new(blas_instances);
        tlas.rebuild(BVHBuildMode::FastBuild);

        for x in 0..width {
            for y in 0..height {
                let pixel_center = Vec2::new(x as f32, y as f32) + Vec2::splat(0.5);
                let uv = (pixel_center / Vec2::new(width as f32, height as f32)) * 2.0 - 1.0;
                let target = *camera.proj_inv_matrix() * Vec4::new(uv.x, uv.y, 1.0, 1.0);
                let direction = *camera.view_inv_matrix() * Vec4::from((target.xyz().normalize() * Vec3::new(-1.0, -1.0, 1.0), 0.0));

                let ray = Ray::new(&origin.xyz(), &direction.xyz());

                // let mut closest_hit: Option<Intersection> = None;

                // for mesh in &mut self.meshes {
                //     let instances = &mesh.1.1;
                //     let mesh = &mut mesh.1.0;
                    
                //     for instance in instances {
                //         let instance_transform = &self.mesh_instances.get(instance).unwrap().transform;
                //         let instance_ray = Ray::new(&(*ray.origin() - *instance_transform.get_position()), ray.direction());

                //         if let Some(hit) = mesh.intersect(&instance_ray, 0.01, 100.0) {
                //             if let Some(closest) = &closest_hit {
                //                 if hit.t < closest.t {
                //                     closest_hit = Some(hit);
                //                 }
                //             } else {
                //                 closest_hit = Some(hit);
                //             }
                //         }
                //     }
                // }

                if let Some(closest_hit) = tlas.intersect(&ray, 0.01, 100.0, &mut blases) {
                    let cold = Vec3::new(0.0, 1.0, 0.0);
                    let hot = Vec3::new(1.0, 0.0, 0.0);
                    let t = (closest_hit.heat as f32 / 50.0).clamp(0.0, 1.0);
                    let color = cold.lerp(hot, t);

                    self.framebuffer.set_pixel(x, y, &color);//&Vec3::new(1.0, 1.0, hit.t * 0.2));
                } else {
                    self.framebuffer.set_pixel(x, y, &Vec3::new(0.0, 0.0, 0.0));
                }
            }
        }

        self.framebuffer.display(window);
    }
}