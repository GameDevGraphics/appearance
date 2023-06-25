use glam::*;
use std::sync::{Arc, Mutex};
use rayon::prelude::*;

use super::{Camera, CameraMatrices, Framebuffer};
use super::primitives::*;
use super::acc_structures::*;

pub mod main_pipeline;

const CHUNK_SIZE: usize = 32;
const PACKET_WIDTH: usize = 16;
const PACKET_HEIGHT: usize = 16;
const SIMD_LANE_WIDTH: usize = 2;
const SIMD_LANE_HEIGHT: usize = 2;

pub trait PipelineTarget {
    fn set_pixel(&mut self, x: u32, y: u32, value: &Vec3);
}

pub trait PipelineLayout<P: Default + Copy, T: PipelineTarget + Send>: Send + Sync {
    fn ray_gen(&self, camera: &CameraMatrices, uv: &Vec2, payload: &mut P) -> Ray;
    fn chit(&self, ray: Ray, intersection: Intersection, payload: &mut P);
    fn miss(&self, ray: Ray, payload: &mut P);
    fn present(&self, payload: P, uv: &IVec2, render_target: &mut T);
}

pub struct Pipeline<P: Default + Copy, T: PipelineTarget + Send> {
    pipeline_layout: Box<dyn PipelineLayout<P, T>>
}

impl<P: Default + Copy, T: PipelineTarget + Send> Pipeline<P, T> {
    pub fn new(pipeline_layout: Box<dyn PipelineLayout<P, T>>) -> Self {
        Pipeline {
            pipeline_layout
        }
    }

    pub fn dispatch(&mut self,
        width: u32, height: u32,
        camera: &Camera,
        tlas: &TLAS, blases: &[&BLAS<Triangle>],
        render_target: Arc<Mutex<T>>
    ) {
        // let origin = (camera.view_inv_matrix() * Vec4::new(0.0, 0.0, 0.0, 1.0)).xyz();
        // let proj_inv_matrix = camera.proj_inv_matrix();
        // let view_inv_matrix = camera.view_inv_matrix();
        let camera_matrices = camera.get_matrices();

        let chunk_count_x = (width as f32 / CHUNK_SIZE as f32).ceil() as usize;
        let chunk_count_y = (height as f32 / CHUNK_SIZE as f32).ceil() as usize;
        let chunk_count = chunk_count_x * chunk_count_y;

        (0..chunk_count).into_par_iter().for_each(|chunk_idx| {
            let chunk_x = chunk_idx % chunk_count_x;
            let chunk_y = chunk_idx / chunk_count_x;

            const REAL_PACKET_WIDTH: usize = PACKET_WIDTH / SIMD_LANE_WIDTH;
            const REAL_PACKET_HEIGHT: usize = PACKET_HEIGHT / SIMD_LANE_HEIGHT;
            let range_x = ((chunk_x * CHUNK_SIZE) / PACKET_WIDTH)..(((chunk_x + 1) * CHUNK_SIZE / PACKET_WIDTH).clamp(0, width as usize));
            let range_y = ((chunk_y * CHUNK_SIZE) / PACKET_HEIGHT)..(((chunk_y + 1) * CHUNK_SIZE / PACKET_HEIGHT).clamp(0, height as usize));

            let mut rays = [SIMDRay::default(); REAL_PACKET_WIDTH * REAL_PACKET_HEIGHT];
            let mut origins = [Vec3::ZERO; SIMD_LANE_WIDTH * SIMD_LANE_HEIGHT];
            let mut directions = [Vec3::ZERO; SIMD_LANE_WIDTH * SIMD_LANE_HEIGHT];
            for x in range_x {
                for y in range_y.clone() {
                    let mut payloads = [P::default(); PACKET_WIDTH * PACKET_HEIGHT];

                    // Raygen
                    for px in 0..REAL_PACKET_WIDTH {
                        for py in 0..REAL_PACKET_HEIGHT {
                            for sx in 0..SIMD_LANE_WIDTH {
                                for sy in 0..SIMD_LANE_HEIGHT {
                                    let global_x = x * PACKET_WIDTH + px * SIMD_LANE_WIDTH + sx;
                                    let global_y = y * PACKET_HEIGHT + py * SIMD_LANE_HEIGHT + sy;
                                    let local_x = px * SIMD_LANE_WIDTH + sx;
                                    let local_y = py * SIMD_LANE_HEIGHT + sy;

                                    let pixel_center = Vec2::new(
                                        global_x as f32,
                                        global_y as f32
                                    ) + Vec2::splat(0.5);
                                    let uv = (pixel_center / Vec2::new(width as f32, height as f32)) * 2.0 - 1.0;
                                    let payload = &mut payloads[local_x + local_y * PACKET_WIDTH];

                                    let ray = self.pipeline_layout.ray_gen(&camera_matrices, &uv, payload);
                                    origins[sx + sy * SIMD_LANE_WIDTH] = *ray.origin();
                                    directions[sx + sy * SIMD_LANE_WIDTH] = *ray.direction();
                                }
                            }

                            rays[px + py * REAL_PACKET_WIDTH] = SIMDRay::new(&origins, &directions);
                        }
                    }
                    
                    let ray_packet = SIMDRayPacket::from_cohorent(rays);
                    let intersection = tlas.intersect_simd_packet(&ray_packet, 0.01, 100.0, blases);

                    for px in 0..REAL_PACKET_WIDTH {
                        for py in 0..REAL_PACKET_HEIGHT {
                            let intersection = intersection.intersection(px + py * REAL_PACKET_WIDTH);
                            let ray = rays[px + py * REAL_PACKET_WIDTH];

                            for sx in 0..SIMD_LANE_WIDTH {
                                for sy in 0..SIMD_LANE_HEIGHT {
                                    let local_x = px * SIMD_LANE_WIDTH + sx;
                                    let local_y = py * SIMD_LANE_HEIGHT + sy;
                                    let payload = &mut payloads[local_x + local_y * PACKET_WIDTH];

                                    let intersection = intersection.intersection(sx + sy * SIMD_LANE_WIDTH);
                                    let ray = ray.ray(sx + sy * SIMD_LANE_WIDTH);
                                    if intersection.hit() {
                                        self.pipeline_layout.chit(ray, intersection, payload);
                                    } else {
                                        self.pipeline_layout.miss(ray, payload);
                                    }
                                }
                            }
                        }
                    }

                    // Present
                    for px in 0..REAL_PACKET_WIDTH {
                        for py in 0..REAL_PACKET_HEIGHT {
                            for sx in 0..SIMD_LANE_WIDTH {
                                for sy in 0..SIMD_LANE_HEIGHT {
                                    let local_x = px * SIMD_LANE_WIDTH + sx;
                                    let local_y = py * SIMD_LANE_HEIGHT + sy;

                                    let payload = payloads[local_x + local_y * PACKET_WIDTH];
                                    let uv = IVec2::new(
                                        (x * PACKET_WIDTH + px * SIMD_LANE_WIDTH + sx) as i32,
                                        (y * PACKET_HEIGHT + py * SIMD_LANE_HEIGHT + sy) as i32,
                                    );

                                    if let Ok(mut render_target) = render_target.lock() {
                                        self.pipeline_layout.present(payload, &uv, &mut render_target);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        });
    }
}