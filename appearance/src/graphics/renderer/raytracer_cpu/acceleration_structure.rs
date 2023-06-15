use glam::*;
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

    pub fn update_bounds(&mut self, triangles: &[Triangle]) {
        let mut min = Vec3::splat(f32::MAX);
        let mut max = Vec3::splat(f32::MIN);
        for i in self.first_prim..(self.first_prim + self.prim_count) {
            let triangle = &triangles[i as usize];
            min = min.min(triangle.min());
            max = max.max(triangle.max());
        }
        self.bounds = AABB::new(&min, &max);
    }
}

#[allow(clippy::upper_case_acronyms)]
pub struct BVH {
    nodes: Vec<Node>,
    triangles: Vec<Triangle>
}

#[derive(Clone, Copy, Debug)]
pub enum BVHBuildMode {
    FastBuild,
    FastTrace
}

impl BVH {
    pub fn new(mut triangles: Vec<Triangle>, build_mode: BVHBuildMode) -> Self {
        let mut triangle_centroids = Vec::with_capacity(triangles.len());
        for triangle in &triangles {
            triangle_centroids.push(triangle.centroid());
        }

        let mut nodes = vec![Node::default(); triangles.len() * 2 - 1];
        let mut node_count = 1;
        let root_node = &mut nodes[0];
        root_node.prim_count = triangles.len() as u32;
        root_node.update_bounds(&triangles);
        Self::subdivide(0, &mut nodes, &mut node_count, &mut triangles, &mut triangle_centroids, build_mode);

        BVH {
            nodes,
            triangles
        }
    }

    fn evaluate_sah(
        node: &Node, axis: usize, split_pos: f32,
        triangles: &[Triangle], triangle_centroids: &[Vec3]
    ) -> f32 {
        let mut left = AABB::default();
        let mut right = AABB::default();
        let mut left_count = 0;
        let mut right_count = 0;
        for i in node.first_prim..(node.first_prim + node.prim_count) {
            let triangle = &triangles[i as usize];
            let centroid = &triangle_centroids[i as usize];

            if centroid[axis] < split_pos {
                left_count += 1;
                left.grow(triangle);
            } else {
                right_count += 1;
                right.grow(triangle);
            }
        }
        (left_count as f32 * left.surface_area()) + (right_count as f32 * right.surface_area())
    }

    fn subdivide(
        idx: usize, nodes: &mut Vec<Node>, node_count: &mut usize,
        triangles: &mut Vec<Triangle>, triangle_centroids: &mut Vec<Vec3>,
        build_mode: BVHBuildMode
    ) {
        let (mut i, left_count, first_prim, prim_count, lchild); {
            let node = &mut nodes[idx];

            // Decide split plane
            let extent = node.bounds.extent();
            let (axis, cost, split_pos) = match build_mode {
                BVHBuildMode::FastBuild => {
                    let mut axis = 0;
                    if extent.y > extent.x {
                        axis = 1;
                    }
                    if extent.z > extent[axis] {
                        axis = 2;
                    }
                    let split_pos = node.bounds.min()[axis] + extent[axis] * 0.5;

                    (axis, 0.0, split_pos)
                },
                BVHBuildMode::FastTrace => {
                    let mut best_axis = 0;
                    let mut best_split_pos = 0.0;
                    let mut best_cost = f32::MAX;
                    for axis in 0..3 {
                        for i in node.first_prim..(node.first_prim + node.prim_count) {
                            let centroid = triangle_centroids[i as usize][axis];
                            let cost = Self::evaluate_sah(
                                node,
                                axis,
                                centroid,
                                triangles,
                                triangle_centroids
                            );

                            if cost < best_cost {
                                best_cost = cost;
                                best_axis = axis;
                                best_split_pos = centroid;
                            }
                        }
                    }
                    (best_axis, best_cost, best_split_pos)
                }
            };

            let parent_cost = node.prim_count as f32 * node.bounds.surface_area();
            if parent_cost < cost {
                return;
            }
            
            // Sort triangles by plane
            i = node.first_prim as i32;
            let mut j = i + node.prim_count as i32 - 1;
            while i <= j {
                if triangle_centroids[i as usize][axis] < split_pos {
                    i += 1;
                } else {
                    triangles.swap(i as usize, j as usize);
                    triangle_centroids.swap(i as usize, j as usize);
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

        nodes[lchild].update_bounds(triangles);
        nodes[lchild + 1].update_bounds(triangles);
        Self::subdivide(lchild, nodes, node_count, triangles, triangle_centroids, build_mode);
        Self::subdivide(lchild + 1, nodes, node_count, triangles, triangle_centroids, build_mode);
    }

    pub fn intersect(&mut self, ray: &Ray, tmin: f32, tmax: f32) -> Option<Intersection> {
        let mut closest: Option<Intersection> = None;
        let mut stack: [Option<&Node>; 64] = [None; 64];
        let mut stack_idx = 0;
        let mut node = &self.nodes[0];

        loop {
            if node.is_leaf() {
                for i in node.first_prim..(node.first_prim + node.prim_count) {
                    if let Some(hit) = self.triangles[i as usize].intersect(ray, false, tmin, tmax) {
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

        closest
    }
}