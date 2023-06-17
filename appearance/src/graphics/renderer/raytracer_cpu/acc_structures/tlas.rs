use glam::*;
use crate::Timer;
use super::{Ray, AABB, Intersection, BLASInstance, BLAS, BLASPrimitive};

/*****************************************************************************
*                               PUB STRUCTS
******************************************************************************/

#[allow(clippy::upper_case_acronyms)]
pub struct TLAS {
    nodes: Vec<Node>,
    node_count: usize,
    blas_instances: Vec<BLASInstance>,
    indices: Vec<usize>
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

impl TLAS {
    pub fn new(blas_instances: Vec<BLASInstance>) -> Self {
        let mut indices = Vec::with_capacity(blas_instances.len());
        for i in 0..blas_instances.len() {
            indices.push(i);
        }

        TLAS {
            nodes: vec![],
            node_count: 0,
            blas_instances,
            indices
        }
    }

    pub fn rebuild(&mut self) {
        let mut blas_centroids = Vec::with_capacity(self.blas_instances.len());
        for blas_instance in &self.blas_instances {
            blas_centroids.push(blas_instance.centroid());
        }

        self.nodes = vec![Node::default(); self.blas_instances.len() * 2 - 1];
        self.node_count = 1;
        let root_node = &mut self.nodes[0];
        root_node.first_prim = 0;
        root_node.prim_count = self.blas_instances.len() as u32;
        root_node.update_bounds(&self.blas_instances, &self.indices);

        let timer = Timer::new();
        Self::subdivide(0, &mut self.nodes, &mut self.node_count, &self.blas_instances, &mut self.indices, &blas_centroids);
        println!("BVH build in: {:.2}ms", (timer.elapsed() * 1000.0) as f32);
    }

    fn best_split(
        node: &Node,
        blas_instances: &[BLASInstance], indices: &[usize], blas_centroids: &[Vec3]
    ) -> (usize, f32, f32) {
        let mut tight_bounds = AABB::default();
        for i in node.first_prim..(node.first_prim + node.prim_count) {
            tight_bounds.grow_vec3(&blas_centroids[indices[i as usize]]);
        }
        let extent = tight_bounds.extent();

        let mut best_axis = 0;
        let mut best_split_pos = 0.0;
        let mut best_cost = f32::MAX;

        let bin_count = 8;

        for axis in 0..3 {
            let mut bin = [Bin::default(); 32];
            if extent[axis] == 0.0 {
                continue;
            }
            let inv_scale = bin_count as f32 / extent[axis];

            for i in node.first_prim..(node.first_prim + node.prim_count) {
                let triangle = &blas_instances[indices[i as usize]];
                let centroid = &blas_centroids[indices[i as usize]];

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
        blas_instances: &[BLASInstance], indices: &mut Vec<usize>, blas_centroids: &[Vec3]
    ) {
        let (mut i, left_count, first_prim, prim_count, lchild); {
            let node = &mut nodes[idx];
            if node.prim_count <= 2 {
                return;
            }

            // Decide split plane
            let (axis, cost, split_pos) = Self::best_split(
                node,
                blas_instances,
                indices,
                blas_centroids
            );
            let parent_cost = node.prim_count as f32 * node.bounds.surface_area();
            if parent_cost < cost {
                return;
            }
            
            // Sort triangles by plane
            i = node.first_prim as i32;
            let mut j = i + node.prim_count as i32 - 1;
            while i <= j {
                if blas_centroids[indices[i as usize]][axis] < split_pos {
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

        nodes[lchild].update_bounds(blas_instances, indices);
        nodes[lchild + 1].update_bounds(blas_instances, indices);
        Self::subdivide(lchild, nodes, node_count, blas_instances, indices, blas_centroids);
        Self::subdivide(lchild + 1, nodes, node_count, blas_instances, indices, blas_centroids);
    }

    pub fn intersect<T: BLASPrimitive>(&self, ray: &Ray, tmin: f32, tmax: f32, blases: &mut [&BLAS<T>]) -> Option<Intersection> {
        let mut closest: Option<Intersection> = None;
        let mut stack: [Option<&Node>; 64] = [None; 64];
        let mut stack_idx = 0;
        let mut node = &self.nodes[0];

        let mut heat = 0;

        loop {
            heat += 1;
            if node.is_leaf() {
                for i in node.first_prim..(node.first_prim + node.prim_count) {
                    if let Some(hit) = self.blas_instances[self.indices[i as usize]].intersect(ray, tmin, tmax, blases) {
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

    pub fn update_bounds(&mut self, blas_instances: &[BLASInstance], indices: &[usize]) {
        self.bounds = AABB::default();
        for i in self.first_prim..(self.first_prim + self.prim_count) {
            blas_instances[indices[i as usize]].expand_aabb(&mut self.bounds);
        }
    }
}