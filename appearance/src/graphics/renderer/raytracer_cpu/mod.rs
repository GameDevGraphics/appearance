use std::collections::HashMap;
use uuid::Uuid;
use rayon::prelude::*;
use std::sync::{Arc, Mutex};

use glam::*;
use super::*;
use crate::{Window, Camera};

mod framebuffer;
use framebuffer::*;
mod ray;
use ray::*;
mod simd_ray;
use simd_ray::*;
mod mesh;
use mesh::*;
mod acc_structures;
use acc_structures::*;

pub struct RaytracerCPU {
    framebuffer: Arc<Mutex<Framebuffer>>,

    mesh_handles: HandleQueue<MeshRendererID>,                  // Handle queue to hand out handles
    mesh_instances: HashMap<Rc<MeshRendererID>, MeshRenderer>,  // Every handle pointing to its instance
    meshes: HashMap<Uuid, (Mesh, Vec<Rc<MeshRendererID>>)>      // Every mesh id with all its instances
}

impl RaytracerCPU {
    pub(crate) fn new(window: &Window) -> Self {
        gl_helper::gl_init(window.internal_context());
        let framebuffer = Arc::new(
            Mutex::new(
                Framebuffer::new(window.get_width(), window.get_height())
            )
        );

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
        if let Ok(mut framebuffer) = self.framebuffer.lock() {
            *framebuffer = Framebuffer::new(width, height);
        }
    }

    fn render(&mut self, window: &Window, camera: &mut Camera) {
        self.update();

        let (width, height) =
        if let Ok(framebuffer) = self.framebuffer.lock() {
            (framebuffer.width(), framebuffer.height())
        } else {
            panic!("Failed to get framebuffer mutex.")
        };

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
                blas_instances.push(BLASInstance::new(
                    *instance_transform.get_model_matrix(),
                    *instance_transform.get_inv_model_matrix(),
                    blas_idx,
                    &blases
                ));
            }
        }
        let mut tlas = TLAS::new(blas_instances);
        tlas.rebuild();

        Self::dispatch_rays(
            self.framebuffer.clone(),
            width,
            height,
            *camera.proj_inv_matrix(),
            *camera.view_inv_matrix(),
            &origin.xyz(),
            &blases,
            &tlas
        );
        
        if let Ok(mut framebuffer) = self.framebuffer.lock() {
            framebuffer.display(window);
        }
    }
}

impl RaytracerCPU {
    #[allow(clippy::too_many_arguments)]
    fn dispatch_rays(
        framebuffer: Arc<Mutex<Framebuffer>>,
        width: u32, height: u32,
        proj_inv_matrix: Mat4, view_inv_matrix: Mat4, origin: &Vec3,
        blases: &[&BLAS<Triangle>], tlas: &TLAS
    ) {
        let chunk_size = 32;
        let chunk_count_x = (width as f32 / chunk_size as f32).ceil() as usize;
        let chunk_count_y = (height as f32 / chunk_size as f32).ceil() as usize;
        let chunk_count = chunk_count_x * chunk_count_y;

        (0..chunk_count).into_par_iter().for_each(|chunk_idx| {
            let chunk_x = chunk_idx % chunk_count_x;
            let chunk_y = chunk_idx / chunk_count_x;
            // let range_x = (chunk_x * chunk_size)..(((chunk_x + 1) * chunk_size).clamp(0, width as usize));
            // let range_y = (chunk_y * chunk_size)..(((chunk_y + 1) * chunk_size).clamp(0, height as usize));

            // for x in range_x {
            //     for y in range_y.clone() {
            //         let pixel_center = Vec2::new(x as f32, y as f32) + Vec2::splat(0.5);
            //         let uv = (pixel_center / Vec2::new(width as f32, height as f32)) * 2.0 - 1.0;
            //         let target = proj_inv_matrix * Vec4::new(uv.x, uv.y, 1.0, 1.0);
            //         let direction = view_inv_matrix * Vec4::from((target.xyz().normalize() * Vec3::new(-1.0, -1.0, 1.0), 0.0));
                
            //         let ray = Ray::new(origin, &direction.xyz());

            //         let closest_hit = tlas.intersect(&ray, 0.01, 100.0, blases);
            //         if closest_hit.hit() {
            //             let cold = Vec3::new(0.0, 1.0, 0.0);
            //             let hot = Vec3::new(1.0, 0.0, 0.0);
            //             let t = ((closest_hit.heat as f32 - 20.0) / 80.0).clamp(0.0, 1.0);
            //             let color = cold.lerp(hot, t);
                    
            //             if let Ok(mut framebuffer) = framebuffer.lock() {
            //                 framebuffer.set_pixel(x as u32, y as u32, &color);
            //             }
            //         } else {
            //             if let Ok(mut framebuffer) = framebuffer.lock() {
            //                 framebuffer.set_pixel(x as u32, y as u32, &Vec3::new(0.0, 0.0, 0.0));
            //             }
            //         }
            //     }
            // }

            let range_x = ((chunk_x * chunk_size) / 2)..(((chunk_x + 1) * chunk_size).clamp(0, width as usize)  / 2);
            let range_y = ((chunk_y * chunk_size) / 2)..(((chunk_y + 1) * chunk_size).clamp(0, height as usize) / 2);

            for x in range_x {
                for y in range_y.clone() {
                    let mut origins = [Vec3::ZERO; 4];
                    let mut directions = [Vec3::ZERO; 4];
                    for px in 0..2 {
                        for py in 0..2 {
                            let pixel_center = Vec2::new((x * 2 + px) as f32, (y * 2 + py) as f32) + Vec2::splat(0.5);
                            let uv = (pixel_center / Vec2::new(width as f32, height as f32)) * 2.0 - 1.0;
                            let target = proj_inv_matrix * Vec4::new(uv.x, uv.y, 1.0, 1.0);
                            let direction = view_inv_matrix * Vec4::from((target.xyz().normalize() * Vec3::new(-1.0, -1.0, 1.0), 0.0));

                            origins[px + py * 2] = *origin;
                            directions[px + py * 2] = direction.xyz();
                        }
                    }
                    
                    let ray = SIMDRay::new(&origins, &directions);
                    let aabb = AABB::new(&Vec3::ZERO, &Vec3::ONE);
                    //let intersection = aabb.intersect_simd(&ray, 0.01, 100.0);
                    let intersection = tlas.intersect_simd(&ray, 0.01, 100.0, blases);

                    for px in 0..2 {
                        for py in 0..2 {
                            if intersection.hit(px + py * 2) {
                                if let Ok(mut framebuffer) = framebuffer.lock() {
                                    let cold = Vec3::new(0.0, 1.0, 0.0);
                                    let hot = Vec3::new(1.0, 0.0, 0.0);
                                    let t = ((intersection.heat(px + py * 2) as f32 - 20.0) / 80.0).clamp(0.0, 1.0);
                                    let color = cold.lerp(hot, t);

                                    framebuffer.set_pixel((x * 2 + px) as u32, (y * 2 + py) as u32, &color);
                                }
                            } else {
                                if let Ok(mut framebuffer) = framebuffer.lock() {
                                    framebuffer.set_pixel((x * 2 + px) as u32, (y * 2 + py) as u32, &Vec3::ZERO);
                                }
                            }
                        }
                    }
                }
            }
        });
    }
}