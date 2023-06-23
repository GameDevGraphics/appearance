use glam::*;
use std::simd::*;

use super::{Triangle, Frustum, Ray, Intersection, SIMDRayGeneric, SIMDIntersectionGeneric, StrideableLaneCount};

/*****************************************************************************
*                               PUB STRUCTS
******************************************************************************/

#[allow(clippy::upper_case_acronyms)]
#[derive(Clone, Copy, Debug)]
pub struct AABB {
    pub min: Vec3,
    pub max: Vec3
}

/*****************************************************************************
*                               IMPLEMENTATIONS
******************************************************************************/

impl AABB {
    #[inline]
    pub fn new(min: &Vec3, max: &Vec3) -> Self {
        AABB {
            min: *min,
            max: *max
        }
    }

    #[inline]
    pub fn grow_aabb(&mut self, aabb: &AABB) {
        if aabb.min.x == f32::MAX {
            return;
        }
        
        self.grow_vec3(&aabb.min);
        self.grow_vec3(&aabb.max);
    }

    #[inline]
    pub fn grow_triangle(&mut self, triangle: &Triangle) {
        self.grow_vec3(&triangle.p0);
        self.grow_vec3(&triangle.p1);
        self.grow_vec3(&triangle.p2);
    }

    #[inline]
    pub fn grow_vec3(&mut self, p: &Vec3) {
        self.min = p.min(self.min);
        self.max = p.max(self.max);
    }

    #[inline]
    pub fn extent(&self) -> Vec3 {
        self.max - self.min
    }

    #[inline]
    pub fn center(&self) -> Vec3 {
        (self.max + self.min) * 0.5
    }

    #[inline]
    pub fn surface_area(&self) -> f32 {
        let extent = self.extent();
        extent.x * extent.y + extent.y * extent.z + extent.z * extent.x
    }

    #[inline]
    pub fn corners(&self) -> [Vec3; 8] {
        [
            Vec3::new(self.min.x, self.min.y, self.min.z),
            Vec3::new(self.max.x, self.min.y, self.min.z),
            Vec3::new(self.max.x, self.min.y, self.max.z),
            Vec3::new(self.min.x, self.min.y, self.max.z),

            Vec3::new(self.min.x, self.max.y, self.min.z),
            Vec3::new(self.max.x, self.max.y, self.min.z),
            Vec3::new(self.max.x, self.max.y, self.max.z),
            Vec3::new(self.min.x, self.max.y, self.max.z)
        ]
    }

    #[inline]
    fn inv_corners_vec4(&self) -> [Vec4; 8] {
        [
            Vec4::new(-self.min.x, -self.min.y, -self.min.z, 1.0),
            Vec4::new(-self.max.x, -self.min.y, -self.min.z, 1.0),
            Vec4::new(-self.max.x, -self.min.y, -self.max.z, 1.0),
            Vec4::new(-self.min.x, -self.min.y, -self.max.z, 1.0),

            Vec4::new(-self.min.x, -self.max.y, -self.min.z, 1.0),
            Vec4::new(-self.max.x, -self.max.y, -self.min.z, 1.0),
            Vec4::new(-self.max.x, -self.max.y, -self.max.z, 1.0),
            Vec4::new(-self.min.x, -self.max.y, -self.max.z, 1.0)
        ]
    }

    pub fn intersect(&self, ray: &Ray, tmin: f32, tmax: f32) -> Intersection {
        let tx1 = (self.min.x - ray.origin().x) * ray.inv_direction().x;
        let tx2 = (self.max.x - ray.origin().x) * ray.inv_direction().x;
        let mut tmin = tx1.min(tx2);
        let mut tmax = tx1.max(tx2);

        let ty1 = (self.min.y - ray.origin().y) * ray.inv_direction().y;
        let ty2 = (self.max.y - ray.origin().y) * ray.inv_direction().y;
        tmin = ty1.min(ty2).max(tmin);
        tmax = ty1.max(ty2).min(tmax);

        let tz1 = (self.min.z - ray.origin().z) * ray.inv_direction().z;
        let tz2 = (self.max.z - ray.origin().z) * ray.inv_direction().z;
        tmin = tz1.min(tz2).max(tmin);
        tmax = tz1.max(tz2).min(tmax);

        if tmax >= tmin && tmax > 0.0 {
            Intersection {
                t: tmin,
                ..Default::default()
            }
        } else {
            Intersection::default()
        }
    }

    pub fn intersect_simd<const LANES: usize>(&self,
        ray: &SIMDRayGeneric<LANES>,
        tmin: f32, tmax: f32
    ) -> SIMDIntersectionGeneric<LANES>
    where LaneCount<LANES>: SupportedLaneCount + StrideableLaneCount {
        let tx1 = (Simd::<f32, LANES>::splat(self.min.x) - ray.origin_x) * ray.inv_direction_x;
        let tx2 = (Simd::<f32, LANES>::splat(self.max.x) - ray.origin_x) * ray.inv_direction_x;
        let mut tmin = tx1.simd_min(tx2);
        let mut tmax = tx1.simd_max(tx2);

        let ty1 = (Simd::<f32, LANES>::splat(self.min.y) - ray.origin_y) * ray.inv_direction_y;
        let ty2 = (Simd::<f32, LANES>::splat(self.max.y) - ray.origin_y) * ray.inv_direction_y;
        tmin = ty1.simd_min(ty2).simd_max(tmin);
        tmax = ty1.simd_max(ty2).simd_min(tmax);

        let tz1 = (Simd::<f32, LANES>::splat(self.min.z) - ray.origin_z) * ray.inv_direction_z;
        let tz2 = (Simd::<f32, LANES>::splat(self.max.z) - ray.origin_z) * ray.inv_direction_z;
        tmin = tz1.simd_min(tz2).simd_max(tmin);
        tmax = tz1.simd_max(tz2).simd_min(tmax);

        let cmp = tmax.simd_ge(tmin) & tmax.simd_ge(Simd::<f32, LANES>::splat(0.0));
        let diff = Simd::<f32, LANES>::splat(f32::MAX) - tmin;
        let tmin = tmin + diff * Simd::<f32, LANES>::from_array(cmp.to_array().map(|x| (!x) as i32 as f32));
        SIMDIntersectionGeneric {
            t: tmin,
            ..Default::default()
        }
    }

    pub fn intersect_frustum(&self, frustum: &Frustum) -> bool {
        let corners = self.inv_corners_vec4();

        let mut outside_n1 = 0;
        let mut outside_n2 = 0;
        let mut outside_n3 = 0;
        let mut outside_n4 = 0;
        for corner in corners {
            if frustum.n1.dot(corner) > 0.0 { outside_n1 += 1; }
            if frustum.n2.dot(corner) > 0.0 { outside_n2 += 1; }
            if frustum.n3.dot(corner) > 0.0 { outside_n3 += 1; }
            if frustum.n4.dot(corner) > 0.0 { outside_n4 += 1; }
        }

        !(outside_n1 == 8 || outside_n2 == 8 || outside_n3 == 8 || outside_n4 == 8)
    }
}

impl Default for AABB {
    #[inline]
    fn default() -> Self {
        AABB {
            min: Vec3::splat(f32::MAX),
            max: Vec3::splat(f32::MIN)
        }
    }
}