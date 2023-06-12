pub mod window;
pub use window::*;
pub mod renderer;
pub use renderer::*;

use crate::MainLoop;

pub struct Graphics {
    window: Window
}

impl Graphics {
    pub(crate) fn new(main_loop: &MainLoop, title: &'static str, width: u32, height: u32) -> Self {
        let window = Window::new(main_loop, title, width, height);

        Graphics {
            window
        }
    }

    pub fn window(&mut self) -> &mut Window {
        &mut self.window
    }
}