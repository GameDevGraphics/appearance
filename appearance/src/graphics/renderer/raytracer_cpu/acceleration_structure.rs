use glam::*;
use crate::Timer;
use super::{Triangle, Ray, AABB, Intersection};

#[derive(Clone, Copy, Debug)]
struct Node {
    bounds: AABB,
    first_prim: u32,
    prim_count: u32
}

impl Default for Node {
    fn default() -> Self {
        Node {
            bounds: AABB::new(&Vec3::ZERO, &Vec3::ZERO),
            first_prim: 0,
            prim_count: 0
        }
    }
}

impl Node {
    pub fn is_leaf(&self) -> bool {
        self.prim_count != 0
    }

    pub fn update_bounds(&mut self, triangles: &[Triangle], indices: &[usize]) {
        let mut min = Vec3::splat(f32::MAX);
        let mut max = Vec3::splat(f32::MIN);
        for i in self.first_prim..(self.first_prim + self.prim_count) {
            let triangle = &triangles[indices[i as usize]];
            min = min.min(triangle.min());
            max = max.max(triangle.max());
        }
        self.bounds = AABB::new(&min, &max);
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct Bin {
    bounds: AABB,
    prim_count: u32
}

#[allow(clippy::upper_case_acronyms)]
pub struct BVH {
    nodes: Vec<Node>,
    node_count: usize,
    triangles: Vec<Triangle>,
    indices: Vec<usize>
}

#[derive(Clone, Copy, Debug)]
pub enum BVHBuildMode {
    FastBuild,
    FastTrace
}

impl BVH {
    pub fn new(triangles: Vec<Triangle>) -> Self {
        let mut indices = Vec::with_capacity(triangles.len());
        for i in 0..triangles.len() {
            indices.push(i);
        }

        BVH {
            nodes: vec![],
            node_count: 0,
            triangles,
            indices
        }
    }

    pub fn triangles(&mut self) -> &mut Vec<Triangle> {
        &mut self.triangles
    }

    pub fn rebuild(&mut self, build_mode: BVHBuildMode) {
        let mut triangle_centroids = Vec::with_capacity(self.triangles.len());
        for triangle in &self.triangles {
            triangle_centroids.push(triangle.centroid());
        }

        self.nodes = vec![Node::default(); self.triangles.len() * 2 - 1];
        self.node_count = 1;
        let root_node = &mut self.nodes[0];
        root_node.first_prim = 0;
        root_node.prim_count = self.triangles.len() as u32;
        root_node.update_bounds(&self.triangles, &self.indices);

        let timer = Timer::new();
        Self::subdivide(0, &mut self.nodes, &mut self.node_count, &self.triangles, &mut self.indices, &triangle_centroids, build_mode);
        println!("BVH build in: {:.2}ms", (timer.elapsed() * 1000.0) as f32);
    }

    pub fn refit(&mut self) {
        assert_ne!(self.node_count, 0, "Failed to refit BVH.");

        let timer = Timer::new();
        for i in (0..self.node_count).rev() {
            let first_prim; {
                let node = &mut self.nodes[i];
                if node.is_leaf() {
                    node.update_bounds(&self.triangles, &self.indices);
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
        println!("BVH refit in: {:.2}ms", (timer.elapsed() * 1000.0) as f32);
    }

    fn best_split(
        node: &Node, build_mode: BVHBuildMode,
        triangles: &[Triangle], indices: &[usize], triangle_centroids: &[Vec3]
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
            BVHBuildMode::FastBuild => 8,
            BVHBuildMode::FastTrace => 32
        };

        for axis in 0..3 {
            let mut bin = [Bin::default(); 32];
            if extent[axis] == 0.0 {
                continue;
            }
            let inv_scale = bin_count as f32 / extent[axis];

            for i in node.first_prim..(node.first_prim + node.prim_count) {
                let triangle = &triangles[indices[i as usize]];
                let centroid = &triangle_centroids[indices[i as usize]];

                let bin_idx = (bin_count - 1).min(
                    ((centroid[axis] - tight_bounds.min()[axis]) * inv_scale) as i32
                ) as usize;
                bin[bin_idx].prim_count += 1;
                bin[bin_idx].bounds.grow_triangle(triangle);
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
        triangles: &[Triangle], indices: &mut Vec<usize>, triangle_centroids: &[Vec3],
        build_mode: BVHBuildMode
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
                triangles,
                indices,
                triangle_centroids
            );

            let parent_cost = node.prim_count as f32 * node.bounds.surface_area();
            if parent_cost < cost {
                return;
            }
            
            // Sort triangles by plane
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

        nodes[lchild].update_bounds(triangles, indices);
        nodes[lchild + 1].update_bounds(triangles, indices);
        Self::subdivide(lchild, nodes, node_count, triangles, indices, triangle_centroids, build_mode);
        Self::subdivide(lchild + 1, nodes, node_count, triangles, indices, triangle_centroids, build_mode);
    }

    pub fn intersect(&mut self, ray: &Ray, tmin: f32, tmax: f32) -> Option<Intersection> {
        let mut closest: Option<Intersection> = None;
        let mut stack: [Option<&Node>; 64] = [None; 64];
        let mut stack_idx = 0;
        let mut node = &self.nodes[0];

        let mut heat = 0;

        loop {
            heat += 1;
            if node.is_leaf() {
                for i in node.first_prim..(node.first_prim + node.prim_count) {
                    if let Some(hit) = self.triangles[self.indices[i as usize]].intersect(ray, false, tmin, tmax) {
                        if let Some(closest_value) = &closest {
                            if hit.t < closest_value.t {
                                closest = Some(hit);
                            }
                        } else {
                            closest = Some(hit);
                        }
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

                let mut dist1 = child1.bounds.intersect(ray, tmin, tmax).unwrap_or_default().t;
                let mut dist2 = child2.bounds.intersect(ray, tmin, tmax).unwrap_or_default().t;

                if dist1 > dist2 {
                    std::mem::swap(&mut dist1, &mut dist2);
                    std::mem::swap(&mut child1, &mut child2);
                }

                if dist1 == f32::MAX {
                    if stack_idx == 0 {
                        break;
                    } else {
                        stack_idx -= 1;
                        node = stack[stack_idx].unwrap();
                    }
                } else {
                    node = child1;
                    if dist2 != f32::MAX {
                        stack[stack_idx] = Some(child2);
                        stack_idx += 1;
                    }
                }
            }
        }

        if let Some(closest) = &mut closest {
            closest.heat = heat;
        }

        closest
    }
}