use glam::*;
use std::simd::*;

/*****************************************************************************
*                               PUB STRUCTS
******************************************************************************/

#[derive(Clone, Debug)]
pub struct SIMDRay {
    pub origin_x: f32x4,
    pub origin_y: f32x4,
    pub origin_z: f32x4,
    pub direction_x: f32x4,
    pub direction_y: f32x4,
    pub direction_z: f32x4,
    pub inv_direction_x: f32x4,
    pub inv_direction_y: f32x4,
    pub inv_direction_z: f32x4
}

#[derive(Clone, Debug)]
pub struct SIMDIntersection {
    pub t: f32x4,
    pub u: f32x4,
    pub v: f32x4,
    pub heat: i32x4
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

        SIMDRay {
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

    pub fn apply_transform(&self, inv_transform: &Mat4) -> SIMDRay {
        let mut origins = self.origins();
        let mut directions = self.directions();
        for i in 0..4 {
            origins[i] = (*inv_transform * Vec4::from((origins[i], 1.0))).xyz();
            directions[i] = (*inv_transform * Vec4::from((directions[i], 0.0))).xyz();
        }

        SIMDRay::new(&origins, &directions)
    }

    pub fn origins(&self) -> [Vec3; 4] {
        let mut result = [Vec3::default(); 4];
        result[0].x = self.origin_x.as_array()[0];
        result[1].x = self.origin_x.as_array()[1];
        result[2].x = self.origin_x.as_array()[2];
        result[3].x = self.origin_x.as_array()[3];
        result[0].y = self.origin_y.as_array()[0];
        result[1].y = self.origin_y.as_array()[1];
        result[2].y = self.origin_y.as_array()[2];
        result[3].y = self.origin_y.as_array()[3];
        result[0].z = self.origin_z.as_array()[0];
        result[1].z = self.origin_z.as_array()[1];
        result[2].z = self.origin_z.as_array()[2];
        result[3].z = self.origin_z.as_array()[3];
        result
    }

    pub fn directions(&self) -> [Vec3; 4] {
        let mut result = [Vec3::default(); 4];
        result[0].x = self.direction_x.as_array()[0];
        result[1].x = self.direction_x.as_array()[1];
        result[2].x = self.direction_x.as_array()[2];
        result[3].x = self.direction_x.as_array()[3];
        result[0].y = self.direction_y.as_array()[0];
        result[1].y = self.direction_y.as_array()[1];
        result[2].y = self.direction_y.as_array()[2];
        result[3].y = self.direction_y.as_array()[3];
        result[0].z = self.direction_z.as_array()[0];
        result[1].z = self.direction_z.as_array()[1];
        result[2].z = self.direction_z.as_array()[2];
        result[3].z = self.direction_z.as_array()[3];
        result
    }
}

impl Default for SIMDIntersection {
    fn default() -> Self {
        SIMDIntersection {
            t: f32x4::splat(f32::MAX),
            u: f32x4::splat(0.0),
            v: f32x4::splat(0.0),
            heat: i32x4::splat(0)
        }
    }
}

impl SIMDIntersection {
    pub fn hit(&self, i: usize) -> bool {
        self.t.as_array()[i] != f32::MAX
    }

    pub fn heat(&self, i: usize) -> i32 {
        self.heat.as_array()[i]
    }

    pub fn store_closest(&mut self, other: &SIMDIntersection) {
        let cmp = other.t.simd_lt(self.t);

        self.t = self.t.simd_min(other.t);
        let diff = other.u - self.u;
        self.u += diff * f32x4::from_array(cmp.to_array().map(|x| x as i32 as f32));
        let diff = other.v - self.v;
        self.v += diff * f32x4::from_array(cmp.to_array().map(|x| x as i32 as f32));
        let diff = other.heat - self.heat;
        self.heat += diff * i32x4::from_array(cmp.to_array().map(|x| x as i32));
    }
}