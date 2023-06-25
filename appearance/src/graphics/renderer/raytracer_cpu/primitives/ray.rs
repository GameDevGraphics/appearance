use glam::*;
use super::Frustum;

/*****************************************************************************
*                               PUB STRUCTS
******************************************************************************/

#[derive(Clone, Copy, Debug)]
pub struct Ray {
    pub origin: Vec3,
    pub direction: Vec3,
    pub inv_direction: Vec3
}

#[derive(Clone, Copy, Debug)]
pub struct Intersection {
    pub t: f32,
    pub uv: Vec2,
    pub blas: i32,
    pub instance: i32,
    pub indices: IVec3
}

pub struct RayPacketSize<const SIZE: usize>;
pub trait SupportedRayPacketSize {
    fn stride() -> usize;
}

impl SupportedRayPacketSize for RayPacketSize<4> { fn stride() -> usize { 2 } }
impl SupportedRayPacketSize for RayPacketSize<16> { fn stride() -> usize { 4 } }
impl SupportedRayPacketSize for RayPacketSize<64> { fn stride() -> usize { 8 } }
impl SupportedRayPacketSize for RayPacketSize<256> { fn stride() -> usize { 16 } }
impl SupportedRayPacketSize for RayPacketSize<1024> { fn stride() -> usize { 32 } }

#[derive(Clone, Debug)]
pub struct RayPacket<const SIZE: usize>
where RayPacketSize<SIZE>: SupportedRayPacketSize {
    rays: [Ray; SIZE],
    frustum: Frustum
}

#[derive(Clone, Debug)]
pub struct RayPacketIntersection<const SIZE: usize>
where RayPacketSize<SIZE>: SupportedRayPacketSize {
    intersections: [Intersection; SIZE]
}

/*****************************************************************************
*                               IMPLEMENTATIONS
******************************************************************************/

impl Ray {
    #[inline]
    pub fn new(origin: &Vec3, direction: &Vec3) -> Self {
        let inv_direction = 1.0 / *direction;
        
        Ray {
            origin: *origin,
            direction: *direction,
            inv_direction
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

    #[inline]
    pub fn inv_direction(&self) -> &Vec3 {
        &self.inv_direction
    }
}

impl Default for Ray {
    fn default() -> Self {
        Ray {
            origin: Vec3::ZERO,
            direction: Vec3::ZERO,
            inv_direction: Vec3::ZERO
        }
    }
}

impl Intersection {
    pub fn hit(&self) -> bool {
        self.t != f32::MAX
    }
}

impl Default for Intersection {
    #[inline]
    fn default() -> Self {
        Intersection {
            t: f32::MAX,
            uv: Vec2::ZERO,
            blas: 0,
            instance: 0,
            indices: IVec3::ZERO
        }
    }
}

impl<const SIZE: usize> RayPacket<SIZE>
where RayPacketSize<SIZE>: SupportedRayPacketSize {
    pub fn from_cohorent(rays: [Ray; SIZE]) -> Self {
        let stride = RayPacketSize::<SIZE>::stride();
        let frustum = Frustum::new(
            &rays[stride - 1].direction,
            &rays[0].direction,
            &rays[SIZE - stride].direction,
            &rays[SIZE - 1].direction,
            &rays[0].origin
        );

        RayPacket {
            rays,
            frustum
        }
    }

    pub fn ray(&self, i: usize) -> &Ray {
        &self.rays[i]
    }

    pub fn frustum(&self) -> &Frustum {
        &self.frustum
    }
}

impl<const SIZE: usize> RayPacketIntersection<SIZE>
where RayPacketSize<SIZE>: SupportedRayPacketSize {
    pub fn intersection(&self, i: usize) -> &Intersection {
        &self.intersections[i]
    }

    pub fn intersection_mut(&mut self, i: usize) -> &mut Intersection {
        &mut self.intersections[i]
    }
}

impl<const SIZE: usize> Default for RayPacketIntersection<SIZE>
where RayPacketSize<SIZE>: SupportedRayPacketSize {
   fn default() -> Self {
        RayPacketIntersection {
            intersections: [Intersection::default(); SIZE]
        }
   }
}