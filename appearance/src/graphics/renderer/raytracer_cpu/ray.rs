use glam::*;
use std::simd::*;
use super::acc_structures::BLASPrimitive;
use super::{SIMDRay, SIMDIntersection};

#[derive(Clone, Debug)]
pub struct Ray {
    origin: Vec3,
    direction: Vec3,
    inv_direction: Vec3,
    signs: [bool; 3]
}

#[derive(Clone, Copy, Debug)]
pub struct Triangle {
    pub p0: Vec3,
    pub p1: Vec3,
    pub p2: Vec3
}

#[derive(Clone, Debug)]
pub struct Intersection {
    pub t: f32,
    pub uv: Vec2,
    pub heat: u32
}

impl Default for Intersection {
    #[inline]
    fn default() -> Self {
        Intersection {
            t: f32::MAX,
            uv: Vec2::ZERO,
            heat: 0
        }
    }
}

impl Intersection {
    pub fn hit(&self) -> bool {
        self.t != f32::MAX
    }
}

#[allow(clippy::upper_case_acronyms)]
#[derive(Clone, Copy, Debug)]
pub struct AABB {
    pub bounds: [Vec3; 2]
}

impl Default for AABB {
    #[inline]
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
    #[inline]
    pub fn new(p0: &Vec3, p1: &Vec3, p2: &Vec3) -> Self {
        Triangle {
            p0: *p0,
            p1: *p1,
            p2: *p2
        }
    }

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
}

impl BLASPrimitive for Triangle {
    #[inline]
    fn centroid(&self) -> Vec3 {
        (self.p0 + self.p1 + self.p2) * 0.33333333
    }

    #[inline]
    fn expand_aabb(&self, aabb: &mut AABB) {
        aabb.grow_triangle(self);
    }

    #[allow(clippy::manual_range_contains)]
    fn intersect(&self, ray: &Ray, tmin: f32, tmax: f32) -> Intersection {
        let edge1 = self.p1 - self.p0;
        let edge2 = self.p2 - self.p0;
        let pvec = ray.direction.cross(edge2);
        let det = edge1.dot(pvec);

        let (mut t, mut u, mut v);
        if false {
            if det < 0.00000001 {
                return Intersection::default();
            }

            let tvec = ray.origin - self.p0;
            u = tvec.dot(pvec);
            if u < 0.0 || u > det {
                return Intersection::default();
            }

            let qvec = tvec.cross(edge1);
            v = ray.direction.dot(qvec);
            if v < 0.0 || u + v > det {
                return Intersection::default();
            }

            t = edge2.dot(qvec);
            if t < tmin || t > tmax {
                return Intersection::default();
            }

            let inv_det = 1.0 / det;
            t *= inv_det;
            u *= inv_det;
            v *= inv_det;
        } else {
            if det > -0.00000001 && det < 0.00000001 {
                return Intersection::default();
            }
            let inv_det = 1.0 / det;

            let tvec = ray.origin - self.p0;
            u = tvec.dot(pvec) * inv_det;
            if u < 0.0 || u > 1.0 {
                return Intersection::default();
            }

            let qvec = tvec.cross(edge1);
            v = ray.direction.dot(qvec) * inv_det;
            if v < 0.0 || u + v > 1.0 {
                return Intersection::default();
            }

            t = edge2.dot(qvec) * inv_det;
        }

        Intersection {
            t,
            uv: Vec2::new(u, v),
            ..Default::default()
        }
    }

    fn intersect_simd(&self, ray: &SIMDRay, tmin: f32, tmax: f32) -> SIMDIntersection {
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

impl AABB {
    #[inline]
    pub fn new(min: &Vec3, max: &Vec3) -> Self {
        AABB {
            bounds: [*min, *max]
        }
    }

    #[inline]
    pub fn grow_aabb(&mut self, aabb: &AABB) {
        if aabb.min().x == f32::MAX {
            return;
        }
        
        self.grow_vec3(&aabb.bounds[0]);
        self.grow_vec3(&aabb.bounds[1]);
    }

    #[inline]
    pub fn grow_triangle(&mut self, triangle: &Triangle) {
        self.grow_vec3(&triangle.p0);
        self.grow_vec3(&triangle.p1);
        self.grow_vec3(&triangle.p2);
    }

    #[inline]
    pub fn grow_vec3(&mut self, p: &Vec3) {
        self.bounds[0] = p.min(self.bounds[0]);
        self.bounds[1] = p.max(self.bounds[1]);
    }

    #[inline]
    pub fn extent(&self) -> Vec3 {
        *self.max() - *self.min()
    }

    #[inline]
    pub fn min(&self) -> &Vec3 {
        &self.bounds[0]
    }

    #[inline]
    pub fn max(&self) -> &Vec3 {
        &self.bounds[1]
    }

    #[inline]
    pub fn surface_area(&self) -> f32 {
        let extent = self.extent();
        extent.x * extent.y + extent.y * extent.z + extent.z * extent.x
    }

    #[inline]
    pub fn corners(&self) -> [Vec3; 8] {
        [
            Vec3::new(self.min().x, self.min().y, self.min().z),
            Vec3::new(self.max().x, self.min().y, self.min().z),
            Vec3::new(self.max().x, self.min().y, self.max().z),
            Vec3::new(self.min().x, self.min().y, self.max().z),

            Vec3::new(self.min().x, self.max().y, self.min().z),
            Vec3::new(self.max().x, self.max().y, self.min().z),
            Vec3::new(self.max().x, self.max().y, self.max().z),
            Vec3::new(self.min().x, self.max().y, self.max().z)
        ]
    }

    // use fused mul-add, fix traversal, min/max

    pub fn intersect(&self, ray: &Ray, tmin: f32, tmax: f32) -> Intersection {
        let mut txmin = (self.bounds[ray.signs[0] as usize].x - ray.origin.x) * ray.inv_direction.x;
        let mut txmax = (self.bounds[1 - ray.signs[0] as usize].x - ray.origin.x) * ray.inv_direction.x;
        let tymin = (self.bounds[ray.signs[1] as usize].y - ray.origin.y) * ray.inv_direction.y;
        let tymax = (self.bounds[1 - ray.signs[1] as usize].y - ray.origin.y) * ray.inv_direction.y;

        if txmin > tymax || tymin > txmax {
            return Intersection::default();
        }

        // if tymin > txmin {
        //     txmin = tymin;
        // }
        txmin = txmin.max(tymin);
        // if tymax < txmax {
        //     txmax = tymax;
        // }
        txmax = txmax.min(tymax);

        let tzmin = (self.bounds[ray.signs[2] as usize].z - ray.origin.z) * ray.inv_direction.z;
        let tzmax = (self.bounds[1 - ray.signs[2] as usize].z - ray.origin.z) * ray.inv_direction.z;

        if txmin > tzmax || tzmin > txmax {
            return Intersection::default();
        }

        // if tzmin > txmin {
        //     txmin = tzmin;
        // }
        txmin = txmin.max(tzmin);
        // if tzmax < txmax {
        //     txmax = tzmax;
        // }
        // NEW!!!!
        txmax = txmax.min(tzmax);
        if txmin < 0.0 {
            txmin = txmax;
        }

        if txmin < tmin || txmin > tmax {
            return Intersection::default();
        }

        Intersection {
            t: txmin,
            ..Default::default()
        }
    }

    pub fn intersect_simd(&self, ray: &SIMDRay, tmin: f32, tmax: f32) -> SIMDIntersection {
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

impl Ray {
    #[inline]
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

    #[inline]
    pub fn origin(&self) -> &Vec3 {
        &self.origin
    }

    #[inline]
    pub fn direction(&self) -> &Vec3 {
        &self.direction
    }
}