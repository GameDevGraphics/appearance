use glam::*;
use std::simd::*;

use super::{Ray, AABB, Intersection, Frustum, BLASPrimitive};
use super::{SIMDRayGeneric, SIMDIntersectionGeneric};

/*****************************************************************************
*                               PUB STRUCTS
******************************************************************************/

#[derive(Clone, Copy, Debug)]
pub struct Triangle {
    pub p0: Vec3,
    pub p1: Vec3,
    pub p2: Vec3
}

/*****************************************************************************
*                               IMPLEMENTATIONS
******************************************************************************/

impl Triangle {
    #[inline]
    pub fn new(p0: &Vec3, p1: &Vec3, p2: &Vec3) -> Self {
        Triangle {
            p0: *p0,
            p1: *p1,
            p2: *p2
        }
    }

    fn cross_simd<const LANES: usize>(
        a_x: Simd<f32, LANES>,
        a_y: Simd<f32, LANES>,
        a_z: Simd<f32, LANES>,
        b: Vec3
    ) -> (Simd<f32, LANES>, Simd<f32, LANES>, Simd<f32, LANES>)
    where LaneCount<LANES>: SupportedLaneCount {
        let b_x = Simd::<f32, LANES>::splat(b.x);
        let b_y = Simd::<f32, LANES>::splat(b.y);
        let b_z = Simd::<f32, LANES>::splat(b.z);
        let c_x = (a_y * b_z) - (b_y * a_z);
        let c_y = (a_z * b_x) - (b_z * a_x);
        let c_z = (a_x * b_y) - (b_x * a_y);

        (c_x, c_y, c_z)
    }

    fn dot_simd<const LANES: usize>(
        a_x: Simd<f32, LANES>,
        a_y: Simd<f32, LANES>,
        a_z: Simd<f32, LANES>,
        b_x: Simd<f32, LANES>,
        b_y: Simd<f32, LANES>,
        b_z: Simd<f32, LANES>,
    ) -> Simd<f32, LANES>
    where LaneCount<LANES>: SupportedLaneCount {
        (a_x * b_x) + (a_y * b_y) + (a_z * b_z)
    }

    fn dot_simd_single<const LANES: usize>(
        a_x: Simd<f32, LANES>,
        a_y: Simd<f32, LANES>,
        a_z: Simd<f32, LANES>,
        b: Vec3
    ) -> Simd<f32, LANES>
    where LaneCount<LANES>: SupportedLaneCount {
        let b_x = Simd::<f32, LANES>::splat(b.x);
        let b_y = Simd::<f32, LANES>::splat(b.y);
        let b_z = Simd::<f32, LANES>::splat(b.z);
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
        let pvec = ray.direction().cross(edge2);
        let det = edge1.dot(pvec);

        let (mut t, mut u, mut v);
        if false {
            if det < 0.00000001 {
                return Intersection::default();
            }

            let tvec = *ray.origin() - self.p0;
            u = tvec.dot(pvec);
            if u < 0.0 || u > det {
                return Intersection::default();
            }

            let qvec = tvec.cross(edge1);
            v = ray.direction().dot(qvec);
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

            let tvec = *ray.origin() - self.p0;
            u = tvec.dot(pvec) * inv_det;
            if u < 0.0 || u > 1.0 {
                return Intersection::default();
            }

            let qvec = tvec.cross(edge1);
            v = ray.direction().dot(qvec) * inv_det;
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

    fn intersect_simd<const LANES: usize>(&self,
        ray: &SIMDRayGeneric<LANES>,
        tmin: f32, tmax: f32
    ) -> SIMDIntersectionGeneric<LANES>
    where LaneCount<LANES>: SupportedLaneCount {
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
        let mut dead_rays = det.simd_gt(Simd::<f32, LANES>::splat(-0.00000001));
        dead_rays &= det.simd_lt(Simd::<f32, LANES>::splat(0.00000001));
        if dead_rays.all() {
            return SIMDIntersectionGeneric::default();
        }
        let inv_det = det.recip();

        // let tvec = ray.origin - self.p0;
        let tvec_x = ray.origin_x - Simd::<f32, LANES>::splat(self.p0.x);
        let tvec_y = ray.origin_y - Simd::<f32, LANES>::splat(self.p0.y);
        let tvec_z = ray.origin_z - Simd::<f32, LANES>::splat(self.p0.z);
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
        dead_rays |= u.simd_lt(Simd::<f32, LANES>::splat(0.0)) | u.simd_gt(Simd::<f32, LANES>::splat(1.0));
        if dead_rays.all() {
            return SIMDIntersectionGeneric::default();
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
        dead_rays |= v.simd_lt(Simd::<f32, LANES>::splat(0.0)) | (u + v).simd_gt(Simd::<f32, LANES>::splat(1.0));
        if dead_rays.all() {
            return SIMDIntersectionGeneric::default();
        }

        // t = edge2.dot(qvec) * inv_det;
        let mut t = Self::dot_simd_single(
            qvec_x,
            qvec_y,
            qvec_z,
            edge2
        ) * inv_det;

        // Replace dead rays t with f32::MAX
        let diff = Simd::<f32, LANES>::splat(f32::MAX) - t;
        t += diff * Simd::<f32, LANES>::from_array(dead_rays.to_array().map(|x| x as i32 as f32));
        
        SIMDIntersectionGeneric {
            t,
            u,
            v,
            ..Default::default()
        }
    }

    fn intersect_frustum(&self, frustum: &Frustum) -> bool {
        let corners = [
            Vec4::from((-self.p0, 1.0)),
            Vec4::from((-self.p1, 1.0)),
            Vec4::from((-self.p2, 1.0))
        ];

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

        !(outside_n1 == 3 || outside_n2 == 3 || outside_n3 == 3 || outside_n4 == 3)
    }
}