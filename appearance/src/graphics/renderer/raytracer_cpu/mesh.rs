use super::{Ray, Triangle, Intersection, BVH, BVHBuildMode};

pub struct Mesh {
    bvh: BVH
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

        let bvh = BVH::new(triangles, BVHBuildMode::FastTrace);

        Mesh {
            bvh
        }
    }

    pub fn intersect(&mut self, ray: &Ray, tmin: f32, tmax: f32) -> Option<Intersection> {
        self.bvh.intersect(ray, tmin, tmax)
    }
}