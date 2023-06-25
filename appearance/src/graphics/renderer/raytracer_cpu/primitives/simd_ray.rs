use glam::*;
use std::simd::*;

use super::{RayPacketSize, SupportedRayPacketSize, Frustum, Intersection, Ray};

pub type SIMDRay = SIMDRayGeneric<4>;
pub type SIMDIntersection = SIMDIntersectionGeneric<4>;

/*****************************************************************************
*                               PUB STRUCTS
******************************************************************************/

pub trait StrideableLaneCount {
    fn stride() -> usize;
}

impl StrideableLaneCount for LaneCount<4> { fn stride() -> usize { 2 } }
impl StrideableLaneCount for LaneCount<8> { fn stride() -> usize { 4 } }
impl StrideableLaneCount for LaneCount<16> { fn stride() -> usize { 4 } }
impl StrideableLaneCount for LaneCount<32> { fn stride() -> usize { 8 } }
impl StrideableLaneCount for LaneCount<64> { fn stride() -> usize { 8 } }

#[derive(Clone, Copy, Debug)]
pub struct SIMDRayGeneric<const LANES: usize>
where LaneCount<LANES>: SupportedLaneCount + StrideableLaneCount {
    pub origin_x:           Simd<f32, LANES>,
    pub origin_y:           Simd<f32, LANES>,
    pub origin_z:           Simd<f32, LANES>,
    pub direction_x:        Simd<f32, LANES>,
    pub direction_y:        Simd<f32, LANES>,
    pub direction_z:        Simd<f32, LANES>,
    pub inv_direction_x:    Simd<f32, LANES>,
    pub inv_direction_y:    Simd<f32, LANES>,
    pub inv_direction_z:    Simd<f32, LANES>
}

#[derive(Clone, Copy, Debug)]
pub struct SIMDIntersectionGeneric<const LANES: usize>
where LaneCount<LANES>: SupportedLaneCount + StrideableLaneCount {
    pub t:          Simd<f32, LANES>,
    pub u:          Simd<f32, LANES>,
    pub v:          Simd<f32, LANES>,
    pub blas:       Simd<i32, LANES>,
    pub instance:   Simd<i32, LANES>,
    pub indices_x:  Simd<i32, LANES>,
    pub indices_y:  Simd<i32, LANES>,
    pub indices_z:  Simd<i32, LANES>,
}

#[derive(Clone, Debug)]
pub struct SIMDRayPacket<const SIZE: usize, const LANES: usize>
where RayPacketSize<SIZE>: SupportedRayPacketSize,
    LaneCount<LANES>: SupportedLaneCount + StrideableLaneCount {
    rays: [SIMDRayGeneric<LANES>; SIZE],
    frustum: Frustum
}

#[derive(Clone, Debug)]
pub struct SIMDRayPacketIntersection<const SIZE: usize, const LANES: usize>
where RayPacketSize<SIZE>: SupportedRayPacketSize,
    LaneCount<LANES>: SupportedLaneCount + StrideableLaneCount {
    intersections: [SIMDIntersectionGeneric<LANES>; SIZE]
}

/*****************************************************************************
*                               IMPLEMENTATIONS
******************************************************************************/

impl<const LANES: usize> SIMDRayGeneric<LANES>
where LaneCount<LANES>: SupportedLaneCount + StrideableLaneCount {
    #[inline]
    pub fn new(origin: &[Vec3; LANES], direction: &[Vec3; LANES]) -> Self {
        let origin_x = Simd::<f32, LANES>::from_array(origin.map(|o| o.x));
        let origin_y = Simd::<f32, LANES>::from_array(origin.map(|o| o.y));
        let origin_z = Simd::<f32, LANES>::from_array(origin.map(|o| o.z));
        let direction_x = Simd::<f32, LANES>::from_array(direction.map(|d| d.x));
        let direction_y = Simd::<f32, LANES>::from_array(direction.map(|d| d.y));
        let direction_z = Simd::<f32, LANES>::from_array(direction.map(|d| d.z));

        let inv_direction_x = direction_x.recip();
        let inv_direction_y = direction_y.recip();
        let inv_direction_z = direction_z.recip();

        SIMDRayGeneric {
            origin_x,
            origin_y,
            origin_z,
            direction_x,
            direction_y,
            direction_z,
            inv_direction_x,
            inv_direction_y,
            inv_direction_z
        }
    }

    pub fn apply_transform(&self, inv_transform: &Mat4) -> SIMDRayGeneric<LANES> {
        let mut origins = self.origins();
        let mut directions = self.directions();
        for i in 0..LANES {
            origins[i] = (*inv_transform * Vec4::from((origins[i], 1.0))).xyz();
            directions[i] = (*inv_transform * Vec4::from((directions[i], 0.0))).xyz();
        }

        Self::new(&origins, &directions)
    }

    pub fn origins(&self) -> [Vec3; LANES] {
        let mut result = [Vec3::default(); LANES];
        for i in 0..LANES {
            result[i].x = self.origin_x.as_array()[i];
            result[i].y = self.origin_y.as_array()[i];
            result[i].z = self.origin_z.as_array()[i];
        }
        result
    }

    pub fn directions(&self) -> [Vec3; LANES] {
        let mut result = [Vec3::default(); LANES];
        for i in 0..LANES {
            result[i].x = self.direction_x.as_array()[i];
            result[i].y = self.direction_y.as_array()[i];
            result[i].z = self.direction_z.as_array()[i];
        }
        result
    }

    pub fn ray(&self, i: usize) -> Ray {
        Ray {
            origin: Vec3::new(self.origin_x.as_array()[i], self.origin_y.as_array()[i], self.origin_z.as_array()[i]),
            direction: Vec3::new(self.direction_x.as_array()[i], self.direction_y.as_array()[i], self.direction_z.as_array()[i]),
            inv_direction: Vec3::new(self.inv_direction_x.as_array()[i], self.inv_direction_y.as_array()[i], self.inv_direction_z.as_array()[i])
        }
    }
}

impl<const LANES: usize> Default for SIMDRayGeneric<LANES>
where LaneCount<LANES>: SupportedLaneCount + StrideableLaneCount {
    fn default() -> Self {
        SIMDRayGeneric {
            origin_x:           Simd::splat(0.0),
            origin_y:           Simd::splat(0.0),
            origin_z:           Simd::splat(0.0),
            direction_x:        Simd::splat(0.0),
            direction_y:        Simd::splat(0.0),
            direction_z:        Simd::splat(0.0),
            inv_direction_x:    Simd::splat(0.0),
            inv_direction_y:    Simd::splat(0.0),
            inv_direction_z:    Simd::splat(0.0)
        }
    }
}

impl<const LANES: usize> Default for SIMDIntersectionGeneric<LANES>
where LaneCount<LANES>: SupportedLaneCount + StrideableLaneCount {
    fn default() -> Self {
        SIMDIntersectionGeneric {
            t: Simd::<f32, LANES>::splat(f32::MAX),
            u: Simd::<f32, LANES>::splat(0.0),
            v: Simd::<f32, LANES>::splat(0.0),
            blas: Simd::<i32, LANES>::splat(0),
            instance: Simd::<i32, LANES>::splat(0),
            indices_x: Simd::<i32, LANES>::splat(0),
            indices_y: Simd::<i32, LANES>::splat(0),
            indices_z: Simd::<i32, LANES>::splat(0)
        }
    }
}

impl<const LANES: usize> SIMDIntersectionGeneric<LANES>
where LaneCount<LANES>: SupportedLaneCount + StrideableLaneCount {
    pub fn intersection(&self, i: usize) -> Intersection {
        Intersection {
            t: self.t.as_array()[i],
            uv: Vec2::new(self.u.as_array()[i], self.v.as_array()[i]),
            blas: self.blas.as_array()[i],
            instance: self.instance.as_array()[i],
            indices: IVec3::new(self.indices_x.as_array()[i], self.indices_y.as_array()[i], self.indices_z.as_array()[i])
        }
    }

    pub fn store_closest(&mut self, other: &SIMDIntersectionGeneric<LANES>) {
        let cmp = other.t.simd_lt(self.t);
        self.t = self.t.simd_min(other.t);

        let diff = other.u - self.u;
        self.u += diff * Simd::<f32, LANES>::from_array(cmp.to_array().map(|x| x as i32 as f32));
        let diff = other.v - self.v;
        self.v += diff * Simd::<f32, LANES>::from_array(cmp.to_array().map(|x| x as i32 as f32));

        let diff = other.blas - self.blas;
        self.blas += diff * Simd::<i32, LANES>::from_array(cmp.to_array().map(|x| x as i32));
        let diff = other.instance - self.instance;
        self.instance += diff * Simd::<i32, LANES>::from_array(cmp.to_array().map(|x| x as i32));

        let diff = other.indices_x - self.indices_x;
        self.indices_x += diff * Simd::<i32, LANES>::from_array(cmp.to_array().map(|x| x as i32));
        let diff = other.indices_y - self.indices_y;
        self.indices_y += diff * Simd::<i32, LANES>::from_array(cmp.to_array().map(|x| x as i32));
        let diff = other.indices_z - self.indices_z;
        self.indices_z += diff * Simd::<i32, LANES>::from_array(cmp.to_array().map(|x| x as i32));
    }
}

impl<const SIZE: usize, const LANES: usize> SIMDRayPacket<SIZE, LANES>
where RayPacketSize<SIZE>: SupportedRayPacketSize,
    LaneCount<LANES>: SupportedLaneCount + StrideableLaneCount {
    pub fn from_cohorent(rays: [SIMDRayGeneric<LANES>; SIZE]) -> Self {
        let stride = RayPacketSize::<SIZE>::stride();
        let simd_stride = LaneCount::<LANES>::stride();
        let frustum = Frustum::new(
            &rays[stride - 1].directions()[simd_stride - 1],
            &rays[0].directions()[0],
            &rays[SIZE - stride].directions()[LANES - simd_stride],
            &rays[SIZE - 1].directions()[LANES - 1],
            &rays[0].origins()[0]
        );

        SIMDRayPacket {
            rays,
            frustum
        }
    }

    pub fn ray(&self, i: usize) -> &SIMDRayGeneric<LANES> {
        &self.rays[i]
    }

    pub fn frustum(&self) -> &Frustum {
        &self.frustum
    }

    pub fn apply_transform(&self, inv_transform: &Mat4) -> Self {
        let mut transformed_rays = self.rays;
        for i in 0..SIZE {
            transformed_rays[i] = self.rays[i].apply_transform(inv_transform);
        }
        Self::from_cohorent(transformed_rays)
    }
}

impl<const SIZE: usize, const LANES: usize> SIMDRayPacketIntersection<SIZE, LANES>
where RayPacketSize<SIZE>: SupportedRayPacketSize,
    LaneCount<LANES>: SupportedLaneCount + StrideableLaneCount {
    pub fn intersection(&self, i: usize) -> &SIMDIntersectionGeneric<LANES> {
        &self.intersections[i]
    }

    pub fn intersection_mut(&mut self, i: usize) -> &mut SIMDIntersectionGeneric<LANES> {
        &mut self.intersections[i]
    }
}

impl<const SIZE: usize, const LANES: usize> Default for SIMDRayPacketIntersection<SIZE, LANES>
where RayPacketSize<SIZE>: SupportedRayPacketSize,
    LaneCount<LANES>: SupportedLaneCount + StrideableLaneCount {
   fn default() -> Self {
        SIMDRayPacketIntersection {
            intersections: [SIMDIntersectionGeneric::default(); SIZE]
        }
   }
}