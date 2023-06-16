use crate::Timer;

use super::{Ray, Triangle, Intersection, BVH, BVHBuildMode};

pub struct Mesh {
    bvh: BVH,
    timer: Timer,
    triangles_original: Vec<Triangle>
}

impl Mesh {
    pub fn new(mesh: &crate::Mesh) -> Self {
        let mut triangles = Vec::new();
        for i in 0..mesh.indices.len() / 3 {
            triangles.push(Triangle::new(
                &mesh.vertices[mesh.indices[i * 3] as usize].position,
                &mesh.vertices[mesh.indices[i * 3 + 1] as usize].position,
                &mesh.vertices[mesh.indices[i * 3 + 2] as usize].position
            ))
        }

        let mut bvh = BVH::new(triangles.clone());
        bvh.rebuild(BVHBuildMode::FastTrace);

        Mesh {
            bvh,
            timer: Timer::new(),
            triangles_original: triangles
        }
    }

    pub fn animate(&mut self) {
        let time = self.timer.elapsed() as f32;
        let triangles = self.bvh.triangles();

        for (i, triangle) in triangles.iter_mut().enumerate() {
            triangle.p0.y = self.triangles_original[i].p0.y + self.triangles_original[i].p0.y * (time).sin() * 0.1;
            triangle.p1.y = self.triangles_original[i].p1.y + self.triangles_original[i].p1.y * (time).sin() * 0.1;
            triangle.p2.y = self.triangles_original[i].p2.y + self.triangles_original[i].p2.y * (time).sin() * 0.1;
        }

        self.bvh.refit();
    }

    pub fn intersect(&mut self, ray: &Ray, tmin: f32, tmax: f32) -> Option<Intersection> {
        self.bvh.intersect(ray, tmin, tmax)
    }
}