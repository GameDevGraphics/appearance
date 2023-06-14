use super::{Ray, Triangle, Intersection};

pub struct Mesh {
    triangles: Vec<Triangle>
}

impl Mesh {
    pub fn new(mesh: &crate::Mesh) -> Self {
        let mut triangles = Vec::new();
        for i in 0..mesh.indices.len() / 3 {
            triangles.push(Triangle::new(
                &mesh.vertices[mesh.indices[i] as usize].position,
                &mesh.vertices[mesh.indices[i + 1] as usize].position,
                &mesh.vertices[mesh.indices[i + 2] as usize].position
            ))
        }

        Mesh {
            triangles
        }
    }

    pub fn intersect(&self, ray: &Ray) -> Option<Intersection> {
        let mut closest_intersection: Option<Intersection> = None;

        for triangle in &self.triangles {
            let intersection = ray.intersect_triangle(triangle);
            if let Some(intersection) = intersection {
                if let Some(closest) = &closest_intersection {
                    if intersection.t < closest.t {
                        closest_intersection = Some(intersection);
                    }
                } else {
                    closest_intersection = Some(intersection);
                }
            }
        }

        closest_intersection
    }
}