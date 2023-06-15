use super::{Ray, Triangle, Intersection, BVH};

pub struct Mesh {
    //triangles: Vec<Triangle>,
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

        let bvh = BVH::new(triangles);

        Mesh {
            bvh
        }
    }

    pub fn intersect(&self, ray: &Ray, tmin: f32, tmax: f32) -> Option<Intersection> {
        // let mut closest_intersection: Option<Intersection> = None;

        // for triangle in &self.triangles {
        //     let intersection = ray.intersect_triangle(triangle, tmin, tmax);
        //     if let Some(intersection) = intersection {
        //         if let Some(closest) = &closest_intersection {
        //             if intersection.t < closest.t {
        //                 closest_intersection = Some(intersection);
        //             }
        //         } else {
        //             closest_intersection = Some(intersection);
        //         }
        //     }
        // }

        // closest_intersection
        self.bvh.intersect(ray, tmin, tmax)
    }
}