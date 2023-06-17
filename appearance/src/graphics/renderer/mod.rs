use uuid::Uuid;

use std::rc::Rc;
use crate::{Transform};

mod handle_queue;
use handle_queue::HandleQueue;
mod raytracer_cpu;
pub use raytracer_cpu::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct MeshRendererID(u32);
impl From<usize> for MeshRendererID { fn from(value: usize) -> Self { MeshRendererID(value as u32) }}

#[derive(Clone, Copy, Debug)]
pub struct MeshRenderer {
    pub enabled: bool,
    pub transform: Transform,
    id: Uuid
}

impl MeshRenderer {
    pub(crate) fn new(uuid: Uuid) -> Self {
        MeshRenderer {
            enabled: true,
            transform: Transform::new(),
            id: uuid
        }
    }

    pub(crate) fn get_id(&self) -> Uuid {
        self.id
    }
}

pub trait Renderer: private::Renderer {
    fn add_mesh(&mut self, model: &crate::Mesh) -> Rc<MeshRendererID>;
    fn mesh_renderer(&mut self, id: Rc<MeshRendererID>) -> &mut MeshRenderer;
}

pub(crate) mod private {
    use crate::{Camera, Window};

    pub trait Renderer {
        fn resize(&mut self, width: u32, height: u32);
        fn render(&mut self, window: &Window, camera: &mut Camera);
    }
}