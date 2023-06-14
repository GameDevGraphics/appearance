use glam::*;

pub struct Ray {
    origin: Vec3,
    direction: Vec3,
    inv_direction: Vec3,
    signs: [bool; 3]
}

pub struct Triangle {
    p0: Vec3,
    p1: Vec3,
    p2: Vec3
}

pub struct Intersection {
    pub t: f32,
    pub uv: Vec2
}

pub struct AABB {
    bounds: [Vec3; 2]
}

impl Triangle {
    pub fn new(p0: &Vec3, p1: &Vec3, p2: &Vec3) -> Self {
        Triangle {
            p0: *p0,
            p1: *p1,
            p2: *p2
        }
    }
}

impl AABB {
    pub fn new(min: &Vec3, max: &Vec3) -> Self {
        AABB {
            bounds: [*min, *max]
        }
    }
}

impl Default for Intersection {
     fn default() -> Self {
         Intersection {
            t: 0.0,
            uv: Vec2::ZERO
        }
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

    pub fn intersect_aabb(&self, aabb: &AABB) -> Option<Intersection> {
        let mut tmin = (aabb.bounds[self.signs[0] as usize].x - self.origin.x) * self.inv_direction.x;
        let mut tmax = (aabb.bounds[1 - self.signs[0] as usize].x - self.origin.x) * self.inv_direction.x;
        let tymin = (aabb.bounds[self.signs[1] as usize].y - self.origin.y) * self.inv_direction.y;
        let tymax = (aabb.bounds[1 - self.signs[1] as usize].y - self.origin.y) * self.inv_direction.y;

        if tmin > tymax || tymin > tmax {
            return None;
        }

        if tymin > tmin {
            tmin = tymin;
        }
        if tymax < tmax {
            tmax = tymax;
        }

        let tzmin = (aabb.bounds[self.signs[2] as usize].z - self.origin.z) * self.inv_direction.z;
        let tzmax = (aabb.bounds[1 - self.signs[2] as usize].z - self.origin.z) * self.inv_direction.z;

        if tmin > tzmax || tzmin > tmax {
            return None;
        }

        if tzmin > tmin {
            tmin = tzmin;
        }
        if tzmax < tmax {
            tmax = tzmax;
        }

        Some(Intersection {
            t: tmin,
            ..Default::default()
        })
    }

    pub fn intersect_triangle(&self, triangle: &Triangle) -> Option<Intersection> {
        let edge1 = triangle.p1 - triangle.p0;
        let edge2 = triangle.p2 - triangle.p0;
        let pvec = self.direction.cross(edge2);
        let det = edge1.dot(pvec);

        if det < 0.00001 {
            return None;
        }

        let tvec = self.origin - triangle.p0;
        let mut u = tvec.dot(pvec);
        if u < 0.0 || u > det {
            return None;
        }

        let qvec = tvec.cross(edge1);
        let mut v = self.direction.dot(qvec);
        if v < 0.0 || v > det {
            return None;
        }

        let mut t = edge2.dot(qvec);
        let inv_det = 1.0 / det;
        t *= inv_det;
        u *= inv_det;
        v *= inv_det;

        Some(Intersection {
            t,
            uv: Vec2::new(u, v)
        })
    }
}