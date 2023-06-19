use glam::*;
use std::simd::*;

pub type SIMDRay = SIMDRayGeneric<16>;
pub type SIMDIntersection = SIMDIntersectionGeneric<16>;

/*****************************************************************************
*                               PUB STRUCTS
******************************************************************************/

#[derive(Clone, Debug)]
pub struct SIMDRayGeneric<const LANES: usize>
where LaneCount<LANES>: SupportedLaneCount {
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

#[derive(Clone, Debug)]
pub struct SIMDIntersectionGeneric<const LANES: usize>
where LaneCount<LANES>: SupportedLaneCount {
    pub t:      Simd<f32, LANES>,
    pub u:      Simd<f32, LANES>,
    pub v:      Simd<f32, LANES>,
    pub heat:   Simd<i32, LANES>
}

/*****************************************************************************
*                               IMPLEMENTATIONS
******************************************************************************/

impl<const LANES: usize> SIMDRayGeneric<LANES>
where LaneCount<LANES>: SupportedLaneCount {
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
}

impl<const LANES: usize> Default for SIMDIntersectionGeneric<LANES>
where LaneCount<LANES>: SupportedLaneCount {
    fn default() -> Self {
        SIMDIntersectionGeneric {
            t: Simd::<f32, LANES>::splat(f32::MAX),
            u: Simd::<f32, LANES>::splat(0.0),
            v: Simd::<f32, LANES>::splat(0.0),
            heat: Simd::<i32, LANES>::splat(0)
        }
    }
}

impl<const LANES: usize> SIMDIntersectionGeneric<LANES>
where LaneCount<LANES>: SupportedLaneCount {
    pub fn hit(&self, i: usize) -> bool {
        self.t.as_array()[i] != f32::MAX
    }

    pub fn heat(&self, i: usize) -> i32 {
        self.heat.as_array()[i]
    }

    pub fn store_closest(&mut self, other: &SIMDIntersectionGeneric<LANES>) {
        let cmp = other.t.simd_lt(self.t);

        self.t = self.t.simd_min(other.t);
        let diff = other.u - self.u;
        self.u += diff * Simd::<f32, LANES>::from_array(cmp.to_array().map(|x| x as i32 as f32));
        let diff = other.v - self.v;
        self.v += diff * Simd::<f32, LANES>::from_array(cmp.to_array().map(|x| x as i32 as f32));
        let diff = other.heat - self.heat;
        self.heat += diff * Simd::<i32, LANES>::from_array(cmp.to_array().map(|x| x as i32));
    }
}