use glam::*;

use std::sync::{Arc};

use super::{Triangle, BLAS, BLASBuildMode};

pub struct Mesh {
    mesh: crate::Mesh,
    material: Arc<crate::Material>,
    blas: BLAS<Triangle>
}

impl Mesh {
    pub fn new(mesh: crate::Mesh, material: Arc<crate::Material>) -> Self {
        let mut triangles = Vec::new();
        for i in 0..mesh.indices.len() / 3 {
            triangles.push(Triangle::new(
                &mesh.vertices[mesh.indices[i * 3] as usize].position,
                &mesh.vertices[mesh.indices[i * 3 + 1] as usize].position,
                &mesh.vertices[mesh.indices[i * 3 + 2] as usize].position,
                IVec3::new(mesh.indices[i * 3] as i32, mesh.indices[i * 3 + 1] as i32, mesh.indices[i * 3 + 2] as i32)
            ))
        }

        let mut blas = BLAS::new(triangles.clone());
        blas.rebuild(BLASBuildMode::FastTrace);

        Mesh {
            mesh,
            material,
            blas
        }
    }
    
    pub fn blas(&self) -> &BLAS<Triangle> {
        &self.blas
    }

    pub fn mesh_data(&self) -> &crate::Mesh {
        &self.mesh
    }

    pub fn material(&self) -> &Arc<crate::Material> {
        &self.material
    }
}