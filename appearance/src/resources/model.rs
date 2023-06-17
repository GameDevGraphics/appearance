use glam::*;
use uuid::Uuid;

use std::hash::{Hash, Hasher};
use std::rc::Rc;
use super::Image;

#[derive(Clone, Debug)]
pub struct Material {
    pub name: String,
    pub index: Option<usize>,

    pub base_color_factor: Vec4,
    pub base_color_texture: Option<Rc<Image>>,

    pub normal_scale: f32,
    pub normal_texture: Option<Rc<Image>>,

    pub metallic_factor: f32,
    pub roughness_factor: f32,
    pub metallic_roughness_texture: Option<Rc<Image>>,

    pub occlusion_strength: f32,
    pub occlusion_texture: Option<Rc<Image>>,

    pub emissive_factor: Vec3,
    pub emissive_texture: Option<Rc<Image>>,
}

impl Default for Material {
    fn default() -> Self {
        Material {
            name: "default".to_owned(),
            index: None,
            base_color_factor: Vec4::new(1.0, 1.0, 1.0, 1.0),
            base_color_texture: None,
            normal_scale: 1.0,
            normal_texture: None,
            metallic_factor: 0.0,
            roughness_factor: 1.0,
            metallic_roughness_texture: None,
            occlusion_strength: 1.0,
            occlusion_texture: None,
            emissive_factor: Vec3::ZERO,
            emissive_texture: None,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Vertex {
    pub position: Vec3,
    pub normal: Vec3,
    pub tangent: Vec4,
    pub tex_coord: Vec2,
    pub tex_coord_1: Vec2,
    pub color: Vec4
}

impl Default for Vertex {
    fn default() -> Self {
        Vertex {
            position: Vec3::ZERO,
            normal: Vec3::ZERO,
            tangent: Vec4::ZERO,
            tex_coord: Vec2::ZERO,
            tex_coord_1: Vec2::ZERO,
            color: Vec4::ZERO
        }
    }
}

#[derive(Clone, Debug)]
pub struct Mesh {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,

    pub min: Vec3,
    pub max: Vec3,

    pub material_idx: usize,
    id: Uuid
}

impl Mesh {
    pub fn new(
        vertices: Vec<Vertex>,
        indices: Vec<u32>,
        min: Vec3, max: Vec3,
        material_idx: usize
    ) -> Self {
        Mesh {
            vertices,
            indices,
            min,
            max,
            material_idx,
            id: Uuid::new_v4()
        }
    }

    pub fn get_id(&self) -> Uuid {
        self.id
    }
}

impl PartialEq for Mesh {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
    }
}

impl Hash for Mesh {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.id.hash(state);
    }
}

#[derive(Clone, Debug)]
pub struct ModelNode {
    pub children: Vec<Rc<ModelNode>>,

    pub position: Vec3,
    pub rotation: Quat,
    pub scale: Vec3,

    pub mesh: Option<Mesh>
}

impl Default for ModelNode {
    fn default() -> Self {
        ModelNode {
            children: Vec::new(),
            position: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
            mesh: None
        }
    }
}

#[derive(Clone, Debug)]
pub struct Model {
    pub root_nodes: Vec<Rc<ModelNode>>,
    pub materials: Vec<Rc<Material>>
}