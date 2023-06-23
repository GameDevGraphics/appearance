use glam::*;
use super::Ray;
use std::simd::*;

/*****************************************************************************
*                               PUB STRUCTS
******************************************************************************/

#[derive(Clone, Copy, Debug)]
pub struct Frustum {
    pub n1: Vec4,
    pub n2: Vec4,
    pub n3: Vec4,
    pub n4: Vec4,
    pub n_x: f32x4,
    pub n_y: f32x4,
    pub n_z: f32x4,
    pub n_w: f32x4
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
            n4: Vec4::from((n4, d4)),
            n_x: f32x4::from_array([n1.x, n2.x, n3.x, n4.x]),
            n_y: f32x4::from_array([n1.y, n2.y, n3.y, n4.y]),
            n_z: f32x4::from_array([n1.z, n2.z, n3.z, n4.z]),
            n_w: f32x4::from_array([d1, d2, d3, d4])
        }
    }
}

impl Default for Frustum {
    fn default() -> Self {
        Frustum {
            n1: Vec4::ZERO,
            n2: Vec4::ZERO,
            n3: Vec4::ZERO,
            n4: Vec4::ZERO,
            n_x: f32x4::splat(0.0),
            n_y: f32x4::splat(0.0),
            n_z: f32x4::splat(0.0),
            n_w: f32x4::splat(0.0)
        }
    }
}