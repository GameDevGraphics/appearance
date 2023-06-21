use glutin::window::{Icon, CursorIcon};
use glutin::{ContextWrapper, PossiblyCurrent};

use crate::MainLoop;

pub struct Window {
    context: ContextWrapper<PossiblyCurrent, glutin::window::Window>
}

impl Window {
    pub(crate) fn new(main_loop: &MainLoop, title: &'static str, width: u32, height: u32) -> Self {        
        let window_builder = glutin::window::WindowBuilder::new()
            .with_title(title)
            .with_inner_size(glutin::dpi::PhysicalSize::new(width, height));

        let context = glutin::ContextBuilder::new()
            .with_gl(glutin::GlRequest::Specific(glutin::Api::OpenGl, (3, 0)))
            .build_windowed(window_builder, main_loop.internal_loop())
            .expect("Failed to build context.");

        let context = unsafe {
            context.make_current()
                .expect("Failed to make context current.")
        };

        Window {
            context
        }
    }

    pub(crate) fn internal_context(&self) -> &glutin::ContextWrapper<glutin::PossiblyCurrent, glutin::window::Window> {
        &self.context
    }

    pub(crate) fn internal_window(&self) -> &glutin::window::Window {
        self.context.window()
    }

    /// Get inner with.
    pub fn get_width(&self) -> u32 {
        self.internal_window().inner_size().width
    }

    /// Get inner height.
    pub fn get_height(&self) -> u32 {
        self.internal_window().inner_size().height
    }

    /// Set inner width.
    pub fn set_width(&self, width: u32) {
        self.internal_window().set_inner_size(
            glutin::dpi::LogicalSize::new(
                width,
                self.internal_window().inner_size().height
            )
        );
    }

    /// Set inner height.
    pub fn set_height(&self, height: u32) {
        self.internal_window().set_inner_size(
            glutin::dpi::LogicalSize::new(
                self.internal_window().inner_size().width,
                height
            )
        );
    }

    /// Set the window icon, if `icon == None` the os default window icon will be used.
    pub fn set_icon(&self, icon: Option<Icon>) {
        self.internal_window().set_window_icon(icon);
    }

    /// Set the cursor icon.
    pub fn set_cursor_icon(&self, cursor: CursorIcon) {
        self.internal_window().set_cursor_icon(cursor);
    }

    /// Get if the window is resizable.
    pub fn is_resizable(&self) -> bool {
        self.internal_window().is_resizable()
    }

    /// Set if the window is resizable.
    pub fn set_resizable(&self, resizable: bool) {
        self.internal_window().set_resizable(resizable);
    }
}