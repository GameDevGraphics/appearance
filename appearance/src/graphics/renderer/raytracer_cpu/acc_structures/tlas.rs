use glam::*;
use super::{Ray, AABB, Intersection, BLASInstance, BLAS, BLASPrimitive, SIMDRay, SIMDIntersection};

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

    pub fn intersect<T: BLASPrimitive>(&self, ray: &Ray, tmin: f32, tmax: f32, blases: &[&BLAS<T>]) -> Intersection {
        let mut closest = Intersection::default();
        let mut stack = [None; 64];
        let mut stack_idx = 0;
        let mut node = &self.nodes[0];

        let mut heat = 0;

        loop {
            heat += 1;
            if node.is_leaf() {
                let hit = self.blas_instances[node.blas_idx as usize].intersect(ray, tmin, tmax, blases);
                if hit.t < closest.t {
                    closest = hit;
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

        closest.heat += heat;

        closest
    }

    pub fn intersect_simd<T: BLASPrimitive>(&self, ray: &SIMDRay, tmin: f32, tmax: f32, blases: &[&BLAS<T>]) -> SIMDIntersection {
        let mut closest = SIMDIntersection::default();
        let mut stack = [None; 64];
        let mut stack_idx = 0;
        let mut node = &self.nodes[0];

        let mut heat = 0;

        loop {
            heat += 1;
            if node.is_leaf() {
                let hit = self.blas_instances[node.blas_idx as usize].intersect_simd(ray, tmin, tmax, blases);
                closest.store_closest(&hit);

                if stack_idx == 0 {
                    break;
                } else {
                    stack_idx -= 1;
                    node = stack[stack_idx].unwrap();
                }
            } else {
                let lchild_idx = (node.left_right & 0xffff) as usize;
                let rchild_idx = (node.left_right >> 16) as usize;
                let child1 = &self.nodes[lchild_idx];
                let child2 = &self.nodes[rchild_idx];

                let mut dist1 = child1.bounds.intersect_simd(ray, tmin, tmax).t.to_array();
                let mut dist2 = child2.bounds.intersect_simd(ray, tmin, tmax).t.to_array();

                let mut flip_childs = [false; 4];
                for i in 0..4 {
                    if dist1[i] > dist2[i] {
                        std::mem::swap(&mut dist1[i], &mut dist2[i]);
                        flip_childs[i] = true;
                    }
                }

                // This crap can be reduced into 2 simd operations
                let mut hit1 = [false; 4];
                let mut hit2 = [false; 4];
                for i in 0..4 {
                    hit1[i] = dist1[i] < closest.t.as_array()[i];
                    hit2[i] = dist2[i] < closest.t.as_array()[i];
                }
                let mut missed_both = true;
                for i in 0..4 {
                    if hit1[i] || hit2[i] {
                        missed_both = false;
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
                    // Check if all rays that hit have the same node as closest
                    let mut both_hit_as_closest = true;
                    let mut hit1_indices = [0; 4];
                    let mut hit1_count = 0;
                    let mut last_hit_flip = -1;
                    for i in 0..4 {
                        if hit1[i] {
                            if last_hit_flip != -1 && last_hit_flip != flip_childs[i] as i32 {
                                both_hit_as_closest = false;
                            }
                            last_hit_flip = flip_childs[i] as i32;

                            hit1_indices[hit1_count] = i;
                            hit1_count += 1;
                        }
                    }

                    // If both nodes are seen as the closest node, we add them both, the order is irrelevant
                    if !both_hit_as_closest && hit1_count > 1 {
                        node = if flip_childs[hit1_indices[0]] { child2 } else { child1 };
                        stack[stack_idx] = Some(if flip_childs[hit1_indices[0]] { child1 } else { child2 });
                        stack_idx += 1;
                    }
                    // If all rays have the same node as closest, we can be sure to add the first node
                    else if both_hit_as_closest {
                        // We now check if any ray hits the second node
                        let mut any_hit2 = false;
                        let mut any_hit2_idx = 0;
                        for i in 0..hit1_count {
                            if hit2[hit1_indices[i]] {
                                any_hit2 = true;
                                any_hit2_idx = hit1_indices[i];
                            }
                        }

                        // Because the first node depends on whether or not there is a second node
                        if any_hit2 {
                            node = if flip_childs[any_hit2_idx] { child2 } else { child1 };
                            stack[stack_idx] = Some(if flip_childs[any_hit2_idx] { child1 } else { child2 });
                            stack_idx += 1;
                        } else {
                            node = if flip_childs[hit1_indices[0]] { child2 } else { child1 };
                        }
                    }
                }
            }
        }

        closest.heat += std::simd::i32x4::splat(heat);

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