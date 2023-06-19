use glam::*;
use crate::Timer;
use super::{Ray, AABB, Intersection, SIMDRay, SIMDIntersection};

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
    fn intersect_simd(&self, ray: &SIMDRay, tmin: f32, tmax: f32) -> SIMDIntersection;
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

        let timer = Timer::new();
        Self::subdivide(0, &mut self.nodes, &mut self.node_count, &self.primitives, &mut self.indices, &triangle_centroids, build_mode);
        println!("BLAS build in: {:.2}ms", (timer.elapsed() * 1000.0) as f32);
    }

    pub fn refit(&mut self) {
        assert_ne!(self.node_count, 0, "Failed to refit BLAS.");

        let timer = Timer::new();
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
        println!("BLAS refit in: {:.2}ms", (timer.elapsed() * 1000.0) as f32);
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
                    ((centroid[axis] - tight_bounds.min()[axis]) * inv_scale) as i32
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
                    best_split_pos = tight_bounds.min()[axis] + scale * (i + 1) as f32;
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

    pub fn intersect_simd(&self, ray: &SIMDRay, tmin: f32, tmax: f32) -> SIMDIntersection {
        let mut closest = SIMDIntersection::default();
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
                let lchild_idx = node.first_prim as usize;
                let child1 = &self.nodes[lchild_idx];
                let child2 = &self.nodes[lchild_idx + 1];

                let mut dist1 = child1.bounds.intersect_simd(ray, tmin, tmax).t.to_array();
                let mut dist2 = child2.bounds.intersect_simd(ray, tmin, tmax).t.to_array();

                let mut child_indices = [
                    [0, 1],
                    [0, 1],
                    [0, 1],
                    [0, 1]
                ];

                for i in 0..4 {
                    if dist1[i] > dist2[i] {
                        std::mem::swap(&mut dist1[i], &mut dist2[i]);
                        child_indices[i] = [1, 0];
                    }
                }

                let mut missed_both = true;
                for (i, d) in dist1.iter().enumerate() {
                    if *d != f32::MAX && *d < closest.t.as_array()[i] {
                        missed_both = false;
                        break;
                    }
                }

                if missed_both {
                    if stack_idx == 0 {
                        break;
                    } else {
                        stack_idx -= 1;
                        node = stack[stack_idx].unwrap();
                    }
                } else {
                    node = ([child1, child2])[child_indices[0][0]];

                    let mut updated_node = false;
                    for i in 0..4 {
                        if dist2[i] != f32::MAX && dist2[i] < closest.t.as_array()[i] {
                            node = ([child1, child2])[child_indices[i][0]];
                            updated_node = true;

                            stack[stack_idx] = Some(([child1, child2])[child_indices[i][1]]);
                            stack_idx += 1;
                            break;
                        }
                    }

                    if !updated_node {
                        node = ([child1, child2])[child_indices[0][0]];
                    }
                }
            }
        }

        closest.heat = std::simd::i32x4::splat(heat);

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

    pub fn intersect_simd<T: BLASPrimitive>(&self, ray: &SIMDRay, tmin: f32, tmax: f32, blases: &[&BLAS<T>]) -> SIMDIntersection {
        let transformed_ray = ray.apply_transform(&self.inv_transform);
        blases[self.blas_idx as usize].intersect_simd(&transformed_ray, tmin, tmax)
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