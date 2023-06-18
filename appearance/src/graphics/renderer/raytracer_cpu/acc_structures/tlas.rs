use glam::*;
use super::{Ray, AABB, Intersection, BLASInstance, BLAS, BLASPrimitive};

/*****************************************************************************
*                               PUB STRUCTS
******************************************************************************/

#[allow(clippy::upper_case_acronyms)]
pub struct TLAS {
    nodes: Vec<Node>,
    node_count: usize,
    blas_instances: Vec<BLASInstance>
}

/*****************************************************************************
*                               PRIVATE STRUCTS
******************************************************************************/

#[derive(Clone, Copy, Debug)]
struct Node {
    bounds: AABB,
    left_right: u32,
    blas_idx: u32
}

/*****************************************************************************
*                               IMPLEMENTATIONS
******************************************************************************/

impl TLAS {
    #[inline]
    pub fn new(blas_instances: Vec<BLASInstance>) -> Self {
        TLAS {
            nodes: vec![],
            node_count: 0,
            blas_instances
        }
    }

    pub fn rebuild(&mut self) {
        self.nodes = vec![Node::default(); self.blas_instances.len() * 2];
        self.node_count = 1;

        let mut node_indices = vec![0; self.blas_instances.len()];
        let mut index_count = self.blas_instances.len();

        for (i, blas_instance) in self.blas_instances.iter().enumerate() {
            node_indices[i] = self.node_count;
            self.nodes[self.node_count].bounds = *blas_instance.bounds();
            self.nodes[self.node_count].blas_idx = i as u32;
            self.nodes[self.node_count].left_right = 0;
            self.node_count += 1;
        }

        let mut a = 0;
        let mut b = Self::find_best_match(&self.nodes, &node_indices, index_count, a);
        while index_count > 1 {
            let c = Self::find_best_match(&self.nodes, &node_indices, index_count, b);
            if a == c {
                let node_idx_a = node_indices[a];
                let node_idx_b = node_indices[b];

                let mut bounds = self.nodes[node_idx_a].bounds;
                bounds.grow_aabb(&self.nodes[node_idx_b].bounds);
                let new_node = &mut self.nodes[self.node_count];

                new_node.left_right = node_idx_a as u32 + ((node_idx_b as u32) << 16);
                new_node.bounds = bounds;

                node_indices[a] = self.node_count;
                self.node_count += 1;
                node_indices[b] = node_indices[index_count - 1];
                index_count -= 1;
                b = Self::find_best_match(&self.nodes, &node_indices, index_count, a);
            } else {
                a = b;
                b = c;
            }
        }
        self.nodes[0] = self.nodes[node_indices[a]];
    }

    fn find_best_match(nodes: &[Node], indices: &[usize], n: usize, a: usize) -> usize {
        let mut smallest = f32::MAX;
        let mut best_b = 0;
        for b in 0..n {
            if b != a {
                let mut bounds = nodes[indices[a]].bounds;
                bounds.grow_aabb(&nodes[indices[b]].bounds);

                let surface_area = bounds.surface_area();
                if surface_area < smallest {
                    smallest = surface_area;
                    best_b = b;
                }
            }
        }
        best_b
    }

    pub fn intersect<T: BLASPrimitive>(&self, ray: &Ray, tmin: f32, tmax: f32, blases: &[&BLAS<T>]) -> Option<Intersection> {
        let mut closest: Option<Intersection> = None;
        let mut stack: [Option<&Node>; 64] = [None; 64];
        let mut stack_idx = 0;
        let mut node = &self.nodes[0];

        let mut heat = 0;

        loop {
            heat += 1;
            if node.is_leaf() {
                if let Some(hit) = self.blas_instances[node.blas_idx as usize].intersect(ray, tmin, tmax, blases) {
                    if let Some(closest_value) = &closest {
                        if hit.t < closest_value.t {
                            closest = Some(hit);
                        }
                    } else {
                        closest = Some(hit);
                    }
                }

                if stack_idx == 0 {
                    break;
                } else {
                    stack_idx -= 1;
                    node = stack[stack_idx].unwrap();
                }
            } else {
                let lchild_idx = (node.left_right & 0xffff) as usize;
                let rchild_idx = (node.left_right >> 16) as usize;
                let mut child1 = &self.nodes[lchild_idx];
                let mut child2 = &self.nodes[rchild_idx];

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
            closest.heat += heat;
        }

        closest
    }
}

impl Default for Node {
    #[inline]
    fn default() -> Self {
        Node {
            bounds: AABB::new(&Vec3::ZERO, &Vec3::ZERO),
            left_right: 0,
            blas_idx: 0
        }
    }
}

impl Node {
    #[inline]
    pub fn is_leaf(&self) -> bool {
        self.left_right == 0
    }
}