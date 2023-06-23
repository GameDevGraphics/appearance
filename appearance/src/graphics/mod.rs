use std::rc::Rc;
use crate::Mesh;

use glam::*;

pub mod window;
pub use window::*;
pub mod camera;
pub use camera::*;
pub mod transform;
pub use transform::*;
pub mod renderer;
pub use renderer::*;

use crate::MainLoop;

pub struct Graphics {
    window: Window,
    renderer: Box<dyn Renderer>,
    camera: Camera
}

impl Graphics {
    pub(crate) fn new(main_loop: &MainLoop, title: &'static str, width: u32, height: u32) -> Self {
        let window = Window::new(main_loop, title, width, height);
        let renderer = Box::new(RaytracerCPU::new(&window));
        let mut camera = Camera::new();
        camera.set_position(&Vec3::new(0.0, 0.0, 15.0));

        Graphics {
            window,
            renderer,
            camera
        }
    }

    pub(crate) fn resize(&mut self, width: u32, height: u32) {
        self.camera.set_aspect_ratio(width as f32 / height as f32);
        self.renderer.resize(width, height);
    }

    pub(crate) fn render(&mut self) {
        self.renderer.render(&self.window, &mut self.camera);
    }

    pub fn window(&mut self) -> &mut Window {
        &mut self.window
    }

    pub fn camera(&mut self) -> &mut Camera {
        &mut self.camera
    }

    pub fn add_mesh(&mut self, mesh: &Mesh, transform: Option<Transform>) -> Rc<MeshRendererID> {
        let id = self.renderer.add_mesh(mesh);
        if let Some(transform) = transform {
            self.renderer.mesh_renderer(id.clone()).transform = transform;
        }
        id
    }

    pub fn mesh_renderer(&mut self, id: Rc<MeshRendererID>) -> &mut MeshRenderer {
        self.renderer.mesh_renderer(id)
    }
}