use glam::*;
use super::{Triangle, Ray, AABB, Intersection};

#[derive(Clone, Debug)]
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

pub struct BVH {
    nodes: Vec<Node>,
    triangles: Vec<Triangle>
}

impl BVH {
    pub fn new(mut triangles: Vec<Triangle>) -> Self {
        let mut triangle_centroids = Vec::with_capacity(triangles.len());
        for triangle in &triangles {
            triangle_centroids.push(triangle.centroid());
        }

        let mut nodes = vec![Node::default(); triangles.len() * 2 - 1];
        let mut node_count = 1;
        let root_node = &mut nodes[0];
        root_node.prim_count = triangles.len() as u32;
        root_node.update_bounds(&triangles);
        Self::subdivide(0, &mut nodes, &mut node_count, &mut triangles, &mut triangle_centroids);

        BVH {
            nodes,
            triangles
        }
    }

    fn subdivide(idx: usize, nodes: &mut Vec<Node>, node_count: &mut usize, triangles: &mut Vec<Triangle>, triangle_centroids: &mut Vec<Vec3>) {
        let (mut i, left_count, first_prim, prim_count, lchild); {
            let node = &mut nodes[idx];

            // Decide split plane
            let extent = node.bounds.extent();
            let mut axis = 0;
            if extent.y > extent.x {
                axis = 1;
            }
            if extent.z > extent[axis] {
                axis = 2;
            }
            let split_pos = node.bounds.min()[axis] + extent[axis] * 0.5;

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
        Self::subdivide(lchild, nodes, node_count, triangles, triangle_centroids);
        Self::subdivide(lchild + 1, nodes, node_count, triangles, triangle_centroids);
    }

    pub fn intersect(&self, ray: &Ray, tmin: f32, tmax: f32) -> Option<Intersection> {
        let mut closest = None;
        self.intersect_recursive(ray, 0, tmin, tmax, &mut closest);
        closest
    }

    fn intersect_recursive(&self, ray: &Ray, node_idx: usize, tmin: f32, tmax: f32, closest: &mut Option<Intersection>) {
        let node = &self.nodes[node_idx];
        if node.bounds.intersect(ray, tmin, tmax).is_none()  {
            return;
        }

        if node.is_leaf() {
            for i in node.first_prim..(node.first_prim + node.prim_count) {
                if let Some(hit) = self.triangles[i as usize].intersect(ray, false, tmin, tmax) {
                    if let Some(closest_value) = closest {
                        if hit.t < closest_value.t {
                            *closest = Some(hit);
                        }
                    } else {
                        *closest = Some(hit);
                    }
                }
            }
        }
        else {
            self.intersect_recursive(ray, node.first_prim as usize, tmin, tmax, closest);
            self.intersect_recursive(ray, node.first_prim as usize + 1, tmin, tmax, closest);
        }
    }
}