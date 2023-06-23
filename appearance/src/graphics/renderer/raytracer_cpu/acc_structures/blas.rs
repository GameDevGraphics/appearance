use glam::*;
use crate::Timer;
use super::{Ray, AABB, Intersection, SIMDRayGeneric, SIMDIntersectionGeneric, StrideableLaneCount, Frustum};
use super::{RayPacket, RayPacketIntersection, SIMDRayPacket, SIMDRayPacketIntersection, RayPacketSize, SupportedRayPacketSize};

use std::simd::*;

/*****************************************************************************
*                               PUB STRUCTS
******************************************************************************/

#[allow(clippy::upper_case_acronyms)]
pub struct BLAS<T: BLASPrimitive> {
    nodes: Vec<Node>,
    node_count: usize,
    primitives: Vec<T>,
    indices: Vec<usize>
}

#[derive(Clone, Copy, Debug)]
pub enum BLASBuildMode {
    FastBuild,
    FastTrace
}

pub trait BLASPrimitive {
    fn centroid(&self) -> Vec3;
    fn expand_aabb(&self, aabb: &mut AABB);
    fn intersect(&self, ray: &Ray, tmin: f32, tmax: f32) -> Intersection;
    fn intersect_simd<const LANES: usize>(&self, ray: &SIMDRayGeneric<LANES>, tmin: f32, tmax: f32)
        -> SIMDIntersectionGeneric<LANES> where LaneCount<LANES>: SupportedLaneCount + StrideableLaneCount;
    fn intersect_frustum(&self, frustum: &Frustum) -> bool;
}

pub struct BLASInstance {
    inv_transform: Mat4,
    bounds: AABB,
    blas_idx: u32
}

/*****************************************************************************
*                               PRIVATE STRUCTS
******************************************************************************/

#[derive(Clone, Copy, Debug)]
struct Node {
    bounds: AABB,
    first_prim: u32,
    prim_count: u32
}

#[derive(Clone, Copy, Debug, Default)]
struct Bin {
    bounds: AABB,
    prim_count: u32
}

/*****************************************************************************
*                               IMPLEMENTATIONS
******************************************************************************/

impl<T: BLASPrimitive> BLAS<T> {
    pub fn new(primitives: Vec<T>) -> Self {
        let mut indices = Vec::with_capacity(primitives.len());
        for i in 0..primitives.len() {
            indices.push(i);
        }

        BLAS {
            nodes: vec![],
            node_count: 0,
            primitives,
            indices
        }
    }

    #[inline]
    pub fn primitives(&mut self) -> &mut Vec<T> {
        &mut self.primitives
    }

    pub fn rebuild(&mut self, build_mode: BLASBuildMode) {
        let mut triangle_centroids = Vec::with_capacity(self.primitives.len());
        for triangle in &self.primitives {
            triangle_centroids.push(triangle.centroid());
        }

        self.nodes = vec![Node::default(); self.primitives.len() * 2 - 1];
        self.node_count = 1;
        let root_node = &mut self.nodes[0];
        root_node.first_prim = 0;
        root_node.prim_count = self.primitives.len() as u32;
        root_node.update_bounds(&self.primitives, &self.indices);

        Self::subdivide(0, &mut self.nodes, &mut self.node_count, &self.primitives, &mut self.indices, &triangle_centroids, build_mode);
    }

    pub fn refit(&mut self) {
        assert_ne!(self.node_count, 0, "Failed to refit BLAS.");

        for i in (0..self.node_count).rev() {
            let first_prim; {
                let node = &mut self.nodes[i];
                if node.is_leaf() {
                    node.update_bounds(&self.primitives, &self.indices);
                    continue;
                }
                first_prim = node.first_prim;
            }

            let mut bounds_refitted; {
                let left_node = &self.nodes[first_prim as usize];
                let right_node = &self.nodes[first_prim as usize + 1];
                bounds_refitted = left_node.bounds;
                bounds_refitted.grow_aabb(&right_node.bounds);
            }

            self.nodes[i].bounds = bounds_refitted;
        }
    }

    fn best_split(
        node: &Node, build_mode: BLASBuildMode,
        primitives: &[T], indices: &[usize], triangle_centroids: &[Vec3]
    ) -> (usize, f32, f32) {
        let mut tight_bounds = AABB::default();
        for i in node.first_prim..(node.first_prim + node.prim_count) {
            tight_bounds.grow_vec3(&triangle_centroids[indices[i as usize]]);
        }
        let extent = tight_bounds.extent();

        let mut best_axis = 0;
        let mut best_split_pos = 0.0;
        let mut best_cost = f32::MAX;

        let bin_count = match build_mode {
            BLASBuildMode::FastBuild => 8,
            BLASBuildMode::FastTrace => 32
        };

        for axis in 0..3 {
            let mut bin = [Bin::default(); 32];
            if extent[axis] == 0.0 {
                continue;
            }
            let inv_scale = bin_count as f32 / extent[axis];

            for i in node.first_prim..(node.first_prim + node.prim_count) {
                let triangle = &primitives[indices[i as usize]];
                let centroid = &triangle_centroids[indices[i as usize]];

                let bin_idx = (bin_count - 1).min(
                    ((centroid[axis] - tight_bounds.min[axis]) * inv_scale) as i32
                ) as usize;
                bin[bin_idx].prim_count += 1;
                triangle.expand_aabb(&mut bin[bin_idx].bounds);
            }

            let mut left_area   = [0.0; 32 - 1];
            let mut right_area  = [0.0; 32 - 1];
            let mut left_count  = [0;   32 - 1];
            let mut right_count = [0;   32 - 1];

            let mut left_aabb = AABB::default();
            let mut right_aabb = AABB::default();
            let mut left_sum = 0;
            let mut right_sum = 0;
            for i in 0..(bin_count as usize - 1) {
                left_sum += bin[i].prim_count;
                left_count[i] = left_sum;
                left_aabb.grow_aabb(&bin[i].bounds);
                left_area[i] = left_aabb.surface_area();

                right_sum += bin[bin_count as usize - 1 - i].prim_count;
                right_count[bin_count as usize - 2 - i] = right_sum;
                right_aabb.grow_aabb(&bin[bin_count as usize - 1 - i].bounds);
                right_area[bin_count as usize - 2 - i] = right_aabb.surface_area();
            }

            let scale = extent[axis] / bin_count as f32;
            for i in 0..(bin_count as usize - 1) {
                let cost = (left_count[i] as f32 * left_area[i]) + (right_count[i] as f32 * right_area[i]);
                if cost < best_cost {
                    best_cost = cost;
                    best_axis = axis;
                    best_split_pos = tight_bounds.min[axis] + scale * (i + 1) as f32;
                }
            }
        }

        (best_axis, best_cost, best_split_pos)
    }

    fn subdivide(
        idx: usize, nodes: &mut Vec<Node>, node_count: &mut usize,
        primitives: &[T], indices: &mut Vec<usize>, triangle_centroids: &[Vec3],
        build_mode: BLASBuildMode
    ) {
        let (mut i, left_count, first_prim, prim_count, lchild); {
            let node = &mut nodes[idx];
            if node.prim_count <= 2 {
                return;
            }

            // Decide split plane
            let (axis, cost, split_pos) = Self::best_split(
                node,
                build_mode,
                primitives,
                indices,
                triangle_centroids
            );
            let parent_cost = node.prim_count as f32 * node.bounds.surface_area();
            if parent_cost < cost {
                return;
            }
            
            // Sort primitives by plane
            i = node.first_prim as i32;
            let mut j = i + node.prim_count as i32 - 1;
            while i <= j {
                if triangle_centroids[indices[i as usize]][axis] < split_pos {
                    i += 1;
                } else {
                    indices.swap(i as usize, j as usize);
                    j -= 1;
                }
            }

            // Create child nodes
            left_count = i as usize - node.first_prim as usize;
            if left_count == 0 || left_count == node.prim_count as usize {
                return;
            }
            first_prim = node.first_prim;
            node.first_prim = *node_count as u32;
            *node_count += 2;
            prim_count = node.prim_count;
            lchild = node.first_prim as usize;

            node.prim_count = 0;
        }

        nodes[lchild].first_prim = first_prim;
        nodes[lchild].prim_count = left_count as u32;
        nodes[lchild + 1].first_prim = i as u32;
        nodes[lchild + 1].prim_count = prim_count - left_count as u32;

        nodes[lchild].update_bounds(primitives, indices);
        nodes[lchild + 1].update_bounds(primitives, indices);
        Self::subdivide(lchild, nodes, node_count, primitives, indices, triangle_centroids, build_mode);
        Self::subdivide(lchild + 1, nodes, node_count, primitives, indices, triangle_centroids, build_mode);
    }

    pub fn intersect(&self, ray: &Ray, tmin: f32, tmax: f32) -> Intersection {
        let mut closest = Intersection::default();
        let mut stack = [None; 64];
        let mut stack_idx = 0;
        let mut node = &self.nodes[0];

        let mut heat = 0;

        loop {
            heat += 1;
            if node.is_leaf() {
                for i in node.first_prim..(node.first_prim + node.prim_count) {
                    let hit = self.primitives[self.indices[i as usize]].intersect(ray, tmin, tmax);
                    if hit.t < closest.t {
                        closest = hit;
                    }
                }

                if stack_idx == 0 {
                    break;
                } else {
                    stack_idx -= 1;
                    node = stack[stack_idx].unwrap();
                }
            } else {
                let lchild_idx = node.first_prim as usize;
                let mut child1 = &self.nodes[lchild_idx];
                let mut child2 = &self.nodes[lchild_idx + 1];

                let mut dist1 = child1.bounds.intersect(ray, tmin, tmax).t;
                let mut dist2 = child2.bounds.intersect(ray, tmin, tmax).t;

                if dist1 > dist2 {
                    std::mem::swap(&mut dist1, &mut dist2);
                    std::mem::swap(&mut child1, &mut child2);
                }

                if dist1 == f32::MAX || dist1 > closest.t {
                    if stack_idx == 0 {
                        break;
                    } else {
                        stack_idx -= 1;
                        node = stack[stack_idx].unwrap();
                    }
                } else {
                    node = child1;
                    if dist2 != f32::MAX && dist2 < closest.t {
                        stack[stack_idx] = Some(child2);
                        stack_idx += 1;
                    }
                }
            }
        }

        closest.heat = heat;

        closest
    }

    fn part_rays<const SIZE: usize>(
        ray_packet: &RayPacket<SIZE>,
        closest_hit: &RayPacketIntersection<SIZE>,
        aabb: &AABB,
        indices: &mut[usize; SIZE],
        last: usize,
        tmin: f32, tmax: f32
    ) -> usize
    where RayPacketSize<SIZE>: SupportedRayPacketSize {
        if aabb.intersect_frustum(ray_packet.frustum()) {
            let mut first = 0;
            for i in 0..last {
                if aabb.intersect(ray_packet.ray(indices[i]), tmin, tmax).t < closest_hit.intersection(indices[i]).t {
                    indices.swap(first, i);
                    first += 1;
                }
            }
            first
        } else {
            0
        }
    }

    pub fn intersect_packet<const SIZE: usize>(&self,
        ray_packet: &RayPacket<SIZE>,
        tmin: f32, tmax: f32
    ) -> RayPacketIntersection<SIZE>
    where RayPacketSize<SIZE>: SupportedRayPacketSize {
        let mut closest = RayPacketIntersection::default();
        let mut stack = [(None, 0); 64];
        let mut stack_idx = 0;
        let mut node = &self.nodes[0];

        let mut indices = [0; SIZE];
        for (i, index) in indices.iter_mut().enumerate() {
            *index = i;
        }
        let mut last = SIZE;

        loop {
            last = Self::part_rays(ray_packet, &closest, &node.bounds, &mut indices, last, tmin, tmax);
            
            if last > 0 {
                if node.is_leaf() {
                    for j in node.first_prim..(node.first_prim + node.prim_count) {
                        if self.primitives[self.indices[j as usize]].intersect_frustum(ray_packet.frustum()) {
                            for i in 0..last {
                                let ray_idx = indices[i];
                                let hit = self.primitives[self.indices[j as usize]].intersect(ray_packet.ray(ray_idx), tmin, tmax);
                                if hit.t < closest.intersection(ray_idx).t {
                                    *closest.intersection_mut(ray_idx) = hit;
                                }
                            }
                        }                    
                    }

                    if stack_idx == 0 {
                        break;
                    } else {
                        stack_idx -= 1;
                        node = stack[stack_idx].0.unwrap();
                        last = stack[stack_idx].1;
                    }
                } else {
                    let lchild_idx = node.first_prim as usize;
                    let mut child1 = &self.nodes[lchild_idx];
                    let mut child2 = &self.nodes[lchild_idx + 1];

                    let origin = *ray_packet.ray(0).origin();
                    let dist1 = child1.bounds.center().distance_squared(origin);
                    let dist2 = child2.bounds.center().distance_squared(origin);
                    if dist1 > dist2 {
                        std::mem::swap(&mut child1, &mut child2);
                    }

                    node = child1;
                    stack[stack_idx].0 = Some(child2);
                    stack[stack_idx].1 = last;
                    stack_idx += 1;
                }
            } else {
                if stack_idx == 0 {
                    break;
                } else {
                    stack_idx -= 1;
                    node = stack[stack_idx].0.unwrap();
                    last = stack[stack_idx].1;
                }
            }
        }

        closest
    }

    fn part_simd_rays<const SIZE: usize, const LANES: usize>(
        ray_packet: &SIMDRayPacket<SIZE, LANES>,
        closest_hit: &SIMDRayPacketIntersection<SIZE, LANES>,
        aabb: &AABB,
        indices: &mut[usize; SIZE],
        last: usize,
        tmin: f32, tmax: f32
    ) -> usize
    where RayPacketSize<SIZE>: SupportedRayPacketSize,
        LaneCount<LANES>: SupportedLaneCount + StrideableLaneCount {
        if aabb.intersect_frustum(ray_packet.frustum()) {
            let mut first = 0;
            for i in 0..last {
                let hit = aabb.intersect_simd(ray_packet.ray(indices[i]), tmin, tmax);
                
                if hit.t.simd_lt(closest_hit.intersection(indices[i]).t).any() {
                    indices.swap(first, i);
                    first += 1;
                }
            }
            first
        } else {
            0
        }
    }

    pub fn intersect_simd_packet<const SIZE: usize, const LANES: usize>(&self,
        ray_packet: &SIMDRayPacket<SIZE, LANES>,
        tmin: f32, tmax: f32,
        mut indices: [usize; SIZE],
        mut last: usize
    ) -> SIMDRayPacketIntersection<SIZE, LANES>
    where RayPacketSize<SIZE>: SupportedRayPacketSize,
        LaneCount<LANES>: SupportedLaneCount + StrideableLaneCount {
        let mut closest = SIMDRayPacketIntersection::default();
        let mut stack = [(None, 0); 64];
        let mut stack_idx = 0;
        let mut node = &self.nodes[0];

        // let mut indices = [0; SIZE];
        // for (i, index) in indices.iter_mut().enumerate() {
        //     *index = i;
        // }
        // let mut last = SIZE;

        loop {
            last = Self::part_simd_rays(ray_packet, &closest, &node.bounds, &mut indices, last, tmin, tmax);
            
            if last > 0 {
                if node.is_leaf() {
                    for j in node.first_prim..(node.first_prim + node.prim_count) {
                        if self.primitives[self.indices[j as usize]].intersect_frustum(ray_packet.frustum()) {
                            for i in 0..last {
                                let ray_idx = indices[i];
                                let hit = self.primitives[self.indices[j as usize]].intersect_simd(ray_packet.ray(ray_idx), tmin, tmax);
                                closest.intersection_mut(ray_idx).store_closest(&hit);
                            }
                        }                    
                    }

                    if stack_idx == 0 {
                        break;
                    } else {
                        stack_idx -= 1;
                        node = stack[stack_idx].0.unwrap();
                        last = stack[stack_idx].1;
                    }
                } else {
                    let lchild_idx = node.first_prim as usize;
                    let mut child1 = &self.nodes[lchild_idx];
                    let mut child2 = &self.nodes[lchild_idx + 1];

                    let origin = ray_packet.ray(0).origins()[0];
                    let dist1 = child1.bounds.center().distance_squared(origin);
                    let dist2 = child2.bounds.center().distance_squared(origin);
                    if dist1 > dist2 {
                        std::mem::swap(&mut child1, &mut child2);
                    }

                    node = child1;
                    stack[stack_idx].0 = Some(child2);
                    stack[stack_idx].1 = last;
                    stack_idx += 1;
                }
            } else {
                if stack_idx == 0 {
                    break;
                } else {
                    stack_idx -= 1;
                    node = stack[stack_idx].0.unwrap();
                    last = stack[stack_idx].1;
                }
            }
        }

        closest
    }

    pub fn intersect_simd<const LANES: usize>(&self,
        ray: &SIMDRayGeneric<LANES>,
        tmin: f32, tmax: f32
    ) -> SIMDIntersectionGeneric<LANES>
    where LaneCount<LANES>: SupportedLaneCount + StrideableLaneCount {
        let mut closest = SIMDIntersectionGeneric::default();
        let mut stack = [None; 64];
        let mut stack_idx = 0;
        let mut node = &self.nodes[0];

        let mut heat = 0;

        loop {
            heat += 1;
            if node.is_leaf() {
                for i in node.first_prim..(node.first_prim + node.prim_count) {
                    let hit = self.primitives[self.indices[i as usize]].intersect_simd(ray, tmin, tmax);
                    closest.store_closest(&hit);
                }

                if stack_idx == 0 {
                    break;
                } else {
                    stack_idx -= 1;
                    node = stack[stack_idx].unwrap();
                }
            } else {
                // Disable all bound checks to speed up this hotspot
                unsafe {
                    let lchild_idx = node.first_prim as usize;
                    let child1 = self.nodes.get_unchecked(lchild_idx);
                    let child2 = self.nodes.get_unchecked(lchild_idx + 1);

                    let mut dist1 = child1.bounds.intersect_simd(ray, tmin, tmax).t.to_array();
                    let mut dist2 = child2.bounds.intersect_simd(ray, tmin, tmax).t.to_array();

                    let mut flip_childs = [false; LANES];
                    for i in 0..LANES {
                        if dist1.get_unchecked(i) > dist2.get_unchecked(i) {
                            std::mem::swap(dist1.get_unchecked_mut(i), dist2.get_unchecked_mut(i));
                            *flip_childs.get_unchecked_mut(i) = true;
                        }
                    }

                    let mut hit1 = [false; LANES];
                    let mut hit2 = [false; LANES];
                    for i in 0..LANES {
                        let closest_t = closest.t.as_array()[i];
                        *hit1.get_unchecked_mut(i) = *dist1.get_unchecked(i) < closest_t;
                        *hit2.get_unchecked_mut(i) = *dist2.get_unchecked(i) < closest_t;
                    }
                    let mut missed_both = true;
                    for i in 0..LANES {
                        if *hit1.get_unchecked(i) || *hit2.get_unchecked(i) {
                            missed_both = false;
                        }
                    }

                    if missed_both {
                        if stack_idx == 0 {
                            break;
                        } else {
                            stack_idx -= 1;
                            node = stack.get_unchecked(stack_idx).unwrap();
                        }
                    } else {
                        // Check if all rays that hit have the same node as closest
                        let mut both_hit_as_closest = true;
                        let mut hit1_indices = [0; LANES];
                        let mut hit1_count = 0;
                        let mut last_hit_flip = -1;
                        for i in 0..LANES {
                            if *hit1.get_unchecked(i) {
                                let flip_childs = *flip_childs.get_unchecked(i) as i32;
                                if last_hit_flip != -1 && last_hit_flip != flip_childs {
                                    both_hit_as_closest = false;
                                }
                                last_hit_flip = flip_childs;

                                *hit1_indices.get_unchecked_mut(hit1_count) = i;
                                hit1_count += 1;
                            }
                        }

                        // If both nodes are seen as the closest node, we add them both, the order is irrelevant
                        if !both_hit_as_closest && hit1_count > 1 {
                            node = if *flip_childs.get_unchecked(*hit1_indices.get_unchecked(0)) { child2 } else { child1 };
                            stack[stack_idx] = Some(if *flip_childs.get_unchecked(*hit1_indices.get_unchecked(0)) { child1 } else { child2 });
                            stack_idx += 1;
                        }
                        // If all rays have the same node as closest, we can be sure to add the first node
                        else if both_hit_as_closest {
                            // We now check if any ray hits the second node
                            let mut any_hit2 = false;
                            let mut any_hit2_idx = 0;
                            for i in 0..hit1_count {
                                if *hit2.get_unchecked(*hit1_indices.get_unchecked(i)) {
                                    any_hit2 = true;
                                    any_hit2_idx = *hit1_indices.get_unchecked(i);
                                }
                            }

                            // Because the first node depends on whether or not there is a second node
                            if any_hit2 {
                                node = if *flip_childs.get_unchecked(any_hit2_idx) { child2 } else { child1 };
                                stack[stack_idx] = Some(if *flip_childs.get_unchecked(any_hit2_idx) { child1 } else { child2 });
                                stack_idx += 1;
                            } else {
                                node = if *flip_childs.get_unchecked(*hit1_indices.get_unchecked(0)) { child2 } else { child1 };
                            }
                        }
                    }
                }
            }
        }

        closest.heat = Simd::<i32, LANES>::splat(heat);

        closest
    }
}

impl BLASInstance {
    pub fn new<T: BLASPrimitive>(
        transform: Mat4, inv_transform: Mat4,
        blas_idx: u32, blases: &[&BLAS<T>]
    ) -> Self {
        let mut bounds = AABB::default();
        for corner in blases[blas_idx as usize].nodes[0].bounds.corners() {
            bounds.grow_vec3(&(transform * Vec4::from((corner, 1.0))).xyz());
        }

        BLASInstance {
            inv_transform,
            bounds,
            blas_idx
        }
    }

    #[inline]
    pub fn bounds(&self) -> &AABB {
        &self.bounds
    }

    pub fn intersect<T: BLASPrimitive>(&self, ray: &Ray, tmin: f32, tmax: f32, blases: &[&BLAS<T>]) -> Intersection {
        let transformed_ray = Ray::new(
            &(self.inv_transform * Vec4::from((*ray.origin(), 1.0))).xyz(),
            &(self.inv_transform * Vec4::from((*ray.direction(), 0.0))).xyz()
        );

        blases[self.blas_idx as usize].intersect(&transformed_ray, tmin, tmax)
    }

    pub fn intersect_frustum(&self, frustum: &Frustum) -> bool {
        self.bounds.intersect_frustum(frustum)
    }

    pub fn intersect_simd<const LANES: usize, T: BLASPrimitive>(&self,
        ray: &SIMDRayGeneric<LANES>,
        tmin: f32, tmax: f32,
        blases: &[&BLAS<T>]
    ) -> SIMDIntersectionGeneric<LANES>
    where LaneCount<LANES>: SupportedLaneCount + StrideableLaneCount {
        let transformed_ray = ray.apply_transform(&self.inv_transform);
        blases[self.blas_idx as usize].intersect_simd(&transformed_ray, tmin, tmax)
    }

    pub fn intersect_simd_packet<const SIZE: usize, const LANES: usize, T: BLASPrimitive>(&self,
        ray_packet: &SIMDRayPacket<SIZE, LANES>,
        tmin: f32, tmax: f32,
        blases: &[&BLAS<T>],
        indices: [usize; SIZE],
        last: usize
    ) -> SIMDRayPacketIntersection<SIZE, LANES>
    where RayPacketSize<SIZE>: SupportedRayPacketSize,
        LaneCount<LANES>: SupportedLaneCount + StrideableLaneCount {
        let transformed_ray_packet = ray_packet.apply_transform(&self.inv_transform);
        blases[self.blas_idx as usize].intersect_simd_packet(&transformed_ray_packet, tmin, tmax, indices, last)
    }
}

impl Default for Node {
    #[inline]
    fn default() -> Self {
        Node {
            bounds: AABB::new(&Vec3::ZERO, &Vec3::ZERO),
            first_prim: 0,
            prim_count: 0
        }
    }
}

impl Node {
    #[inline]
    pub fn is_leaf(&self) -> bool {
        self.prim_count != 0
    }

    pub fn update_bounds<T: BLASPrimitive>(&mut self, primitives: &[T], indices: &[usize]) {
        self.bounds = AABB::default();
        for i in self.first_prim..(self.first_prim + self.prim_count) {
            primitives[indices[i as usize]].expand_aabb(&mut self.bounds);
        }
    }
}