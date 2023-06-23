use glam::*;
use super::Ray;

/*****************************************************************************
*                               PUB STRUCTS
******************************************************************************/

#[derive(Clone, Copy, Debug)]
pub struct Frustum {
    pub n1: Vec4,
    pub n2: Vec4,
    pub n3: Vec4,
    pub n4: Vec4
}

/*****************************************************************************
*                               IMPLEMENTATIONS
******************************************************************************/

impl Frustum {
    pub fn new(top_right: &Ray, top_left: &Ray, bottom_left: &Ray, bottom_right: &Ray) -> Self {
        let n1 = (*top_right.direction()).cross(*top_left.direction() - *top_right.direction());
        let n2 = (*top_left.direction()).cross(*bottom_left.direction() - *top_left.direction());
        let n3 = (*bottom_left.direction()).cross(*bottom_right.direction() - *bottom_left.direction());
        let n4 = (*bottom_right.direction()).cross(*top_right.direction() - *bottom_right.direction());
        
        let e = *top_right.origin();
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