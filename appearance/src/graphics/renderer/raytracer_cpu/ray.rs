use glam::*;

#[derive(Clone, Debug)]
pub struct Ray {
    origin: Vec3,
    direction: Vec3,
    inv_direction: Vec3,
    signs: [bool; 3]
}

#[derive(Clone, Copy, Debug)]
pub struct Triangle {
    p0: Vec3,
    p1: Vec3,
    p2: Vec3
}

#[derive(Clone, Debug)]
pub struct Intersection {
    pub t: f32,
    pub uv: Vec2
}

impl Default for Intersection {
    fn default() -> Self {
        Intersection {
            t: f32::MAX,
            uv: Vec2::ZERO
        }
    }
}

#[allow(clippy::upper_case_acronyms)]
#[derive(Clone, Copy, Debug)]
pub struct AABB {
    bounds: [Vec3; 2]
}

impl Default for AABB {
    fn default() -> Self {
        AABB {
            bounds: [
                Vec3::splat(f32::MAX),
                Vec3::splat(f32::MIN)
            ]
        }
    }
}

impl Triangle {
    pub fn new(p0: &Vec3, p1: &Vec3, p2: &Vec3) -> Self {
        Triangle {
            p0: *p0,
            p1: *p1,
            p2: *p2
        }
    }

    pub fn min(&self) -> Vec3 {
        self.p0.min(self.p1.min(self.p2))
    }

    pub fn max(&self) -> Vec3 {
        self.p0.max(self.p1.max(self.p2))
    }

    pub fn centroid(&self) -> Vec3 {
        (self.p0 + self.p1 + self.p2) * 0.33333333
    }

    #[allow(clippy::manual_range_contains)]
    pub fn intersect(&self, ray: &Ray, cull: bool, tmin: f32, tmax: f32) -> Option<Intersection> {
        let edge1 = self.p1 - self.p0;
        let edge2 = self.p2 - self.p0;
        let pvec = ray.direction.cross(edge2);
        let det = edge1.dot(pvec);

        let (mut t, mut u, mut v);
        if cull {
            if det < 0.00000001 {
                return None;
            }

            let tvec = ray.origin - self.p0;
            u = tvec.dot(pvec);
            if u < 0.0 || u > det {
                return None;
            }

            let qvec = tvec.cross(edge1);
            v = ray.direction.dot(qvec);
            if v < 0.0 || u + v > det {
                return None;
            }

            t = edge2.dot(qvec);
            if t < tmin || t > tmax {
                return None;
            }

            let inv_det = 1.0 / det;
            t *= inv_det;
            u *= inv_det;
            v *= inv_det;
        } else {
            if det > -0.00000001 && det < 0.00000001 {
                return None;
            }
            let inv_det = 1.0 / det;

            let tvec = ray.origin - self.p0;
            u = tvec.dot(pvec) * inv_det;
            if u < 0.0 || u > 1.0 {
                return None;
            }

            let qvec = tvec.cross(edge1);
            v = ray.direction.dot(qvec) * inv_det;
            if v < 0.0 || u + v > 1.0 {
                return None;
            }

            t = edge2.dot(qvec) * inv_det;
        }

        Some(Intersection {
            t,
            uv: Vec2::new(u, v)
        })
    }
}

impl AABB {
    pub fn new(min: &Vec3, max: &Vec3) -> Self {
        AABB {
            bounds: [*min, *max]
        }
    }

    pub fn grow(&mut self, triangle: &Triangle) {
        self.bounds[0] = triangle.p0.min(self.bounds[0]);
        self.bounds[1] = triangle.p0.max(self.bounds[1]);
        self.bounds[0] = triangle.p1.min(self.bounds[0]);
        self.bounds[1] = triangle.p1.max(self.bounds[1]);
        self.bounds[0] = triangle.p2.min(self.bounds[0]);
        self.bounds[1] = triangle.p2.max(self.bounds[1]);
    }

    pub fn extent(&self) -> Vec3 {
        *self.max() - *self.min()
    }

    pub fn min(&self) -> &Vec3 {
        &self.bounds[0]
    }

    pub fn max(&self) -> &Vec3 {
        &self.bounds[1]
    }

    pub fn surface_area(&self) -> f32 {
        let extent = self.extent();
        (extent.x * extent.y + extent.x * extent.z + extent.y * extent.z) * 2.0
    }

    pub fn intersect(&self, ray: &Ray, tmin: f32, tmax: f32) -> Option<Intersection> {
        let mut txmin = (self.bounds[ray.signs[0] as usize].x - ray.origin.x) * ray.inv_direction.x;
        let mut txmax = (self.bounds[1 - ray.signs[0] as usize].x - ray.origin.x) * ray.inv_direction.x;
        let tymin = (self.bounds[ray.signs[1] as usize].y - ray.origin.y) * ray.inv_direction.y;
        let tymax = (self.bounds[1 - ray.signs[1] as usize].y - ray.origin.y) * ray.inv_direction.y;

        if txmin > tymax || tymin > txmax {
            return None;
        }

        if tymin > txmin {
            txmin = tymin;
        }
        if tymax < txmax {
            txmax = tymax;
        }

        let tzmin = (self.bounds[ray.signs[2] as usize].z - ray.origin.z) * ray.inv_direction.z;
        let tzmax = (self.bounds[1 - ray.signs[2] as usize].z - ray.origin.z) * ray.inv_direction.z;

        if txmin > tzmax || tzmin > txmax {
            return None;
        }

        if tzmin > txmin {
            txmin = tzmin;
        }
        // if tzmax < txmax {
        //     txmax = tzmax;
        // }

        if txmin < tmin || txmin > tmax {
            return None;
        }

        Some(Intersection {
            t: txmin,
            ..Default::default()
        })
    }
}

impl Ray {
    pub fn new(origin: &Vec3, direction: &Vec3) -> Self {
        let inv_direction = 1.0 / *direction;
        let signs = [
            inv_direction.x < 0.0,
            inv_direction.y < 0.0,
            inv_direction.z < 0.0
        ];
        
        Ray {
            origin: *origin,
            direction: *direction,
            inv_direction,
            signs
        }
    }
}