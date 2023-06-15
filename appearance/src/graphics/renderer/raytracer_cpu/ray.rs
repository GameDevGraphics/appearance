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

#[derive(Clone, Debug)]
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

    pub fn min(&self) -> Vec3 {
        self.p0.min(self.p1.min(self.p2))
    }

    pub fn max(&self) -> Vec3 {
        self.p0.max(self.p1.max(self.p2))
    }

    pub fn centroid(&self) -> Vec3 {
        (self.p0 + self.p1 + self.p2) * 0.33333333
    }
}

impl AABB {
    pub fn new(min: &Vec3, max: &Vec3) -> Self {
        AABB {
            bounds: [*min, *max]
        }
    }

    pub fn extent(&self) -> Vec3 {
        self.bounds[1] - self.bounds[0]
    }

    pub fn min(&self) -> &Vec3 {
        &self.bounds[0]
    }

    pub fn max(&self) -> &Vec3 {
        &self.bounds[1]
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

    pub fn intersect_aabb(&self, aabb: &AABB, tmin: f32, tmax: f32) -> Option<Intersection> {
        let mut txmin = (aabb.bounds[self.signs[0] as usize].x - self.origin.x) * self.inv_direction.x;
        let mut txmax = (aabb.bounds[1 - self.signs[0] as usize].x - self.origin.x) * self.inv_direction.x;
        let tymin = (aabb.bounds[self.signs[1] as usize].y - self.origin.y) * self.inv_direction.y;
        let tymax = (aabb.bounds[1 - self.signs[1] as usize].y - self.origin.y) * self.inv_direction.y;

        if txmin > tymax || tymin > txmax {
            return None;
        }

        if tymin > txmin {
            txmin = tymin;
        }
        if tymax < txmax {
            txmax = tymax;
        }

        let tzmin = (aabb.bounds[self.signs[2] as usize].z - self.origin.z) * self.inv_direction.z;
        let tzmax = (aabb.bounds[1 - self.signs[2] as usize].z - self.origin.z) * self.inv_direction.z;

        if txmin > tzmax || tzmin > txmax {
            return None;
        }

        if tzmin > txmin {
            txmin = tzmin;
        }
        if tzmax < txmax {
            txmax = tzmax;
        }

        if txmin < tmin || txmin > tmax {
            return None;
        }

        Some(Intersection {
            t: txmin,
            ..Default::default()
        })
    }

    pub fn intersect_triangle(&self, triangle: &Triangle, tmin: f32, tmax: f32) -> Option<Intersection> {
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
        if v < 0.0 || u + v > det {
            return None;
        }

        let mut t = edge2.dot(qvec);
        if t < tmin || t > tmax {
            return None;
        }

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