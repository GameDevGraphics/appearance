use super::*;

pub struct RaytracerCPU {

}

impl Default for RaytracerCPU {
    fn default() -> Self {
        RaytracerCPU::new()
    }
}

impl RaytracerCPU {
    pub fn new() -> Self {
        RaytracerCPU {

        }
    }
}

impl Renderer for RaytracerCPU {
    fn add_model(&mut self, model: Rc<Model>) {

    }
}

impl private::Renderer for RaytracerCPU {
    fn resize(&mut self, width: u32, height: u32) {
        
    }

    fn render(&mut self, camera: &Camera) {

    }
}