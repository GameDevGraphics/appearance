use glam::*;
use super::{Ray, AABB, Triangle};

#[derive(Clone, Copy, Debug)]
pub struct Frustum {
    pub n1: Vec4,
    pub n2: Vec4,
    pub n3: Vec4,
    pub n4: Vec4
}

impl Default for Frustum {
    fn default() -> Self {
        Frustum {
            n1: Vec4::ZERO,
            n2: Vec4::ZERO,
            n3: Vec4::ZERO,
            n4: Vec4::ZERO
        }
    }
}

impl Frustum {
    // r0: top right
    // r1: top left
    // r2: bottom left
    // r2: bottom right
    pub fn new(r0: &Ray, r1: &Ray, r2: &Ray, r3: &Ray) -> Self {
        let n1 = (*r0.direction()).cross(*r1.direction() - *r0.direction());
        let n2 = (*r1.direction()).cross(*r2.direction() - *r1.direction());
        let n3 = (*r2.direction()).cross(*r3.direction() - *r2.direction());
        let n4 = (*r3.direction()).cross(*r0.direction() - *r3.direction());
        
        let e = *r0.origin();
        let d1 = n1.dot(e);
        let d2 = n2.dot(e);
        let d3 = n3.dot(e);
        let d4 = n4.dot(e);

        Frustum {
            n1: Vec4::from((n1, d1)),
            n2: Vec4::from((n2, d2)),
            n3: Vec4::from((n3, d3)),
            n4: Vec4::from((n4, d4))
        }
    }
}