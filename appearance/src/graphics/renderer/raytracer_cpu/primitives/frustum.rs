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
    pub fn new(top_right: &Vec3, top_left: &Vec3, bottom_left: &Vec3, bottom_right: &Vec3, origin: &Vec3) -> Self {
        let n1 = (*top_right).cross(*top_left - *top_right);
        let n2 = (*top_left).cross(*bottom_left - *top_left);
        let n3 = (*bottom_left).cross(*bottom_right - *bottom_left);
        let n4 = (*bottom_right).cross(*top_right - *bottom_right);
        
        let d1 = n1.dot(*origin);
        let d2 = n2.dot(*origin);
        let d3 = n3.dot(*origin);
        let d4 = n4.dot(*origin);

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