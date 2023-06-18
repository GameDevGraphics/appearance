use glam::*;
use std::simd::*;

use super::{AABB, Triangle};

/*****************************************************************************
*                               PUB STRUCTS
******************************************************************************/

#[derive(Clone, Debug)]
pub struct SIMDRay {
    origin_x: f32x4,
    origin_y: f32x4,
    origin_z: f32x4,
    direction_x: f32x4,
    direction_y: f32x4,
    direction_z: f32x4,
    inv_direction_x: f32x4,
    inv_direction_y: f32x4,
    inv_direction_z: f32x4,
    signs: [mask32x4; 3]
}

#[derive(Clone, Debug)]
pub struct SIMDIntersection {
    pub t: f32x4,
    pub u: f32x4,
    pub v: f32x4,
    pub heat: u32x4
}

/*****************************************************************************
*                               IMPLEMENTATIONS
******************************************************************************/

impl SIMDRay {
    #[inline]
    pub fn new(origin: &[Vec3; 4], direction: &[Vec3; 4]) -> Self {
        let origin_x = f32x4::from_array([origin[0].x, origin[1].x, origin[2].x, origin[3].x]);
        let origin_y = f32x4::from_array([origin[0].y, origin[1].y, origin[2].y, origin[3].y]);
        let origin_z = f32x4::from_array([origin[0].z, origin[1].z, origin[2].z, origin[3].z]);
        let direction_x = f32x4::from_array([direction[0].x, direction[1].x, direction[2].x, direction[3].x]);
        let direction_y = f32x4::from_array([direction[0].y, direction[1].y, direction[2].y, direction[3].y]);
        let direction_z = f32x4::from_array([direction[0].z, direction[1].z, direction[2].z, direction[3].z]);

        let inv_direction_x = direction_x.recip();
        let inv_direction_y = direction_y.recip();
        let inv_direction_z = direction_z.recip();

        let signs = [
            f32x4::is_sign_negative(inv_direction_x),
            f32x4::is_sign_negative(inv_direction_y),
            f32x4::is_sign_negative(inv_direction_z)
        ];

        SIMDRay {
            origin_x,
            origin_y,
            origin_z,
            direction_x,
            direction_y,
            direction_z,
            inv_direction_x,
            inv_direction_y,
            inv_direction_z,
            signs
        }
    }
}

impl Default for SIMDIntersection {
    fn default() -> Self {
        SIMDIntersection {
            t: f32x4::splat(f32::MAX),
            u: f32x4::splat(0.0),
            v: f32x4::splat(0.0),
            heat: u32x4::splat(0)
        }
    }
}

impl SIMDIntersection {
    pub fn hit(&self, i: usize) -> bool {
        self.t.as_array()[i] != f32::MAX
    }
}

impl AABB {
    pub fn intesect_simd(&self, ray: &SIMDRay, tmin: f32, tmax: f32) -> SIMDIntersection {
        // let mut txmin = (self.bounds[ray.signs[0] as usize].x - ray.origin.x) * ray.inv_direction.x;
        let signs0 = ray.signs[0].to_array();
        let bounds_x = f32x4::from_array([
            self.bounds[signs0[0] as usize].x,
            self.bounds[signs0[1] as usize].x,
            self.bounds[signs0[2] as usize].x,
            self.bounds[signs0[3] as usize].x
        ]);
        let mut txmin = (bounds_x - ray.origin_x) * ray.inv_direction_x;

        // let mut txmax = (self.bounds[1 - ray.signs[0] as usize].x - ray.origin.x) * ray.inv_direction.x;
        let bounds_x = f32x4::from_array([
            self.bounds[1 - signs0[0] as usize].x,
            self.bounds[1 - signs0[1] as usize].x,
            self.bounds[1 - signs0[2] as usize].x,
            self.bounds[1 - signs0[3] as usize].x
        ]);
        let mut txmax = (bounds_x - ray.origin_x) * ray.inv_direction_x;

        // let tymin = (self.bounds[ray.signs[1] as usize].y - ray.origin.y) * ray.inv_direction.y;
        let signs1 = ray.signs[1].to_array();
        let bounds_y = f32x4::from_array([
            self.bounds[signs1[0] as usize].y,
            self.bounds[signs1[1] as usize].y,
            self.bounds[signs1[2] as usize].y,
            self.bounds[signs1[3] as usize].y
        ]);
        let tymin = (bounds_y - ray.origin_y) * ray.inv_direction_y;
        
        // let tymax = (self.bounds[1 - ray.signs[1] as usize].y - ray.origin.y) * ray.inv_direction.y;
        let bounds_y = f32x4::from_array([
            self.bounds[1 - signs1[0] as usize].y,
            self.bounds[1 - signs1[1] as usize].y,
            self.bounds[1 - signs1[2] as usize].y,
            self.bounds[1 - signs1[3] as usize].y
        ]);
        let tymax = (bounds_y - ray.origin_y) * ray.inv_direction_y;

        // if txmin > tymax || tymin > txmax {
        //     return Intersection::default();
        // }
        let mut dead_rays = txmin.simd_gt(tymax);
        dead_rays |= tymin.simd_gt(txmax);
        if dead_rays.all() {
            return SIMDIntersection::default();
        }

        // if tymin > txmin {
        //     txmin = tymin;
        // }
        let cmp = tymin.simd_gt(txmin);
        let diff = tymin - txmin;
        txmin += diff * f32x4::from_array(cmp.to_array().map(|x| x as i32 as f32));

        // if tymax < txmax {
        //     txmax = tymax;
        // }
        let cmp = tymax.simd_lt(txmax);
        let diff = tymax - txmax;
        txmax += diff * f32x4::from_array(cmp.to_array().map(|x| x as i32 as f32));

        // let tzmin = (self.bounds[ray.signs[2] as usize].z - ray.origin.z) * ray.inv_direction.z;
        let signs2 = ray.signs[2].to_array();
        let bounds_z = f32x4::from_array([
            self.bounds[signs2[0] as usize].z,
            self.bounds[signs2[1] as usize].z,
            self.bounds[signs2[2] as usize].z,
            self.bounds[signs2[3] as usize].z
        ]);
        let tzmin = (bounds_z - ray.origin_z) * ray.inv_direction_z;

        // let tzmax = (self.bounds[1 - ray.signs[2] as usize].z - ray.origin.z) * ray.inv_direction.z;
        let bounds_z = f32x4::from_array([
            self.bounds[1 - signs2[0] as usize].z,
            self.bounds[1 - signs2[1] as usize].z,
            self.bounds[1 - signs2[2] as usize].z,
            self.bounds[1 - signs2[3] as usize].z
        ]);
        let tzmax = (bounds_z - ray.origin_z) * ray.inv_direction_z;

        // if txmin > tzmax || tzmin > txmax {
        //     return Intersection::default();
        // }
        dead_rays |= txmin.simd_gt(tzmax) | tzmin.simd_gt(txmax);
        if dead_rays.all() {
            return SIMDIntersection::default();
        }

        // if tzmin > txmin {
        //     txmin = tzmin;
        // }
        let cmp = tzmin.simd_gt(txmin);
        let diff = tzmin - txmin;
        txmin += diff * f32x4::from_array(cmp.to_array().map(|x| x as i32 as f32));

        // if txmin < tmin || txmin > tmax {
        //     return Intersection::default();
        // }
        dead_rays |= txmin.simd_lt(f32x4::splat(tmin)) | txmin.simd_gt(f32x4::splat(tmax));
        if dead_rays.all() {
            return SIMDIntersection::default();
        }

        // Replace dead rays t with f32::MAX
        let diff = f32x4::splat(f32::MAX) - txmin;
        let txmin = txmin + diff * f32x4::from_array(dead_rays.to_array().map(|x| x as i32 as f32));
        
        SIMDIntersection {
            t: txmin,
            ..Default::default()
        }
    }
}

impl Triangle {
    fn cross_simd(
        a_x: f32x4,
        a_y: f32x4,
        a_z: f32x4,
        b: Vec3
    ) -> (f32x4, f32x4, f32x4) {
        let b_x = f32x4::splat(b.x);
        let b_y = f32x4::splat(b.y);
        let b_z = f32x4::splat(b.z);
        let c_x = (a_y * b_z) - (b_y * a_z);
        let c_y = (a_z * b_x) - (b_z * a_x);
        let c_z = (a_x * b_y) - (b_x * a_y);

        (c_x, c_y, c_z)
    }

    fn dot_simd(
        a_x: f32x4,
        a_y: f32x4,
        a_z: f32x4,
        b_x: f32x4,
        b_y: f32x4,
        b_z: f32x4,
    ) -> f32x4 {
        (a_x * b_x) + (a_y * b_y) + (a_z * b_z)
    }

    fn dot_simd_single(
        a_x: f32x4,
        a_y: f32x4,
        a_z: f32x4,
        b: Vec3
    ) -> f32x4 {
        let b_x = f32x4::splat(b.x);
        let b_y = f32x4::splat(b.y);
        let b_z = f32x4::splat(b.z);
        Self::dot_simd(a_x, a_y, a_z, b_x, b_y, b_z)
    }

    pub fn intersect_simd(&self, ray: &SIMDRay, tmin: f32, tmax: f32) -> SIMDIntersection {
        let edge1 = self.p1 - self.p0;
        let edge2 = self.p2 - self.p0;

        // let pvec = ray.direction.cross(edge2);
        let (pvec_x, pvec_y, pvec_z) = Self::cross_simd(
            ray.direction_x,
            ray.direction_y,
            ray.direction_z,
            edge2
        );
        // let det = edge1.dot(pvec);
        let det = Self::dot_simd_single(
            pvec_x,
            pvec_y,
            pvec_z,
            edge1
        );

        // if det > -0.00000001 && det < 0.00000001 {
        //     return Intersection::default();
        // }
        // let inv_det = 1.0 / det;
        let mut dead_rays = det.simd_gt(f32x4::splat(-0.0000001));
        dead_rays &= det.simd_lt(f32x4::splat(0.0000001));
        if dead_rays.all() {
            return SIMDIntersection::default();
        }
        let inv_det = det.recip();

        // let tvec = ray.origin - self.p0;
        let tvec_x = ray.origin_x - f32x4::splat(self.p0.x);
        let tvec_y = ray.origin_y - f32x4::splat(self.p0.y);
        let tvec_z = ray.origin_z - f32x4::splat(self.p0.z);
        // u = tvec.dot(pvec) * inv_det;
        let u = Self::dot_simd(
            pvec_x,
            pvec_y,
            pvec_z,
            tvec_x,
            tvec_y,
            tvec_z
        ) * inv_det;

        // if u < 0.0 || u > 1.0 {
        //     return Intersection::default();
        // }
        dead_rays |= u.simd_lt(f32x4::splat(0.0)) | u.simd_gt(f32x4::splat(1.0));
        if dead_rays.all() {
            return SIMDIntersection::default();
        }

        // let qvec = tvec.cross(edge1);
        let (qvec_x, qvec_y, qvec_z) = Self::cross_simd(
            tvec_x,
            tvec_y,
            tvec_z,
            edge1
        );
        // v = ray.direction.dot(qvec) * inv_det;
        let v = Self::dot_simd(
            ray.direction_x,
            ray.direction_y,
            ray.direction_z,
            qvec_x,
            qvec_y,
            qvec_z
        ) * inv_det;

        // if v < 0.0 || u + v > 1.0 {
        //     return Intersection::default();
        // }
        dead_rays |= v.simd_lt(f32x4::splat(0.0)) | (u + v).simd_gt(f32x4::splat(1.0));
        if dead_rays.all() {
            return SIMDIntersection::default();
        }

        // t = edge2.dot(qvec) * inv_det;
        let mut t = Self::dot_simd_single(
            qvec_x,
            qvec_y,
            qvec_z,
            edge2
        ) * inv_det;

        // Replace dead rays t with f32::MAX
        let diff = f32x4::splat(f32::MAX) - t;
        t += diff * f32x4::from_array(dead_rays.to_array().map(|x| x as i32 as f32));
        
        SIMDIntersection {
            t,
            u,
            v,
            ..Default::default()
        }
    }
}