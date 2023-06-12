use gl::types::*;
use glam::Vec3;

pub mod buffers;
pub use buffers::*;
pub mod shaders;
pub use shaders::*;
pub mod textures;
pub use textures::*;
pub mod imgui_impl;
pub use imgui_impl::*;

pub fn gl_init(context: &glutin::ContextWrapper<glutin::PossiblyCurrent, glutin::window::Window>) {
    gl::load_with(|ptr| context.get_proc_address(ptr) as *const _);
    gl_check();
}

#[cfg(debug_assertions)]
fn gl_check() {
    unsafe {
        let error = gl::GetError();
        match error {
            gl::NO_ERROR => (),
            gl::INVALID_ENUM => panic!("GL invalid enum."),
            gl::INVALID_VALUE => panic!("GL invalid value."),
            gl::INVALID_OPERATION => panic!("GL invalid operation."),
            gl::OUT_OF_MEMORY => panic!("GL out of memory."),
            gl::STACK_OVERFLOW => panic!("GL stack overflow."),
            gl::STACK_UNDERFLOW => panic!("GL stack underflow"),
            _ => panic!("GL unkown error."),
        }
    }
}

#[cfg(not(debug_assertions))]
fn gl_check() {}

pub fn gl_enable_depth() {
    unsafe {
        gl::Enable(gl::DEPTH_TEST);
        gl_check();
    }
}

pub fn gl_cull(mode: GLenum) {
    unsafe {
        gl::Enable(gl::CULL_FACE);
        gl::CullFace(mode);
        gl_check();
    }
}

pub fn gl_clear_color(color: &Vec3) {
    unsafe {
        gl::ClearColor(color.x, color.y, color.z, 1.0f32);
        gl_check();
    }
}

pub fn gl_clear() {
    unsafe {
        gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);
        gl_check();
    }
}

pub fn gl_viewport(width: u32, height: u32) {
    unsafe {
        gl::Viewport(0, 0, width as i32, height as i32);
        gl_check();
    }
}

pub fn gl_draw_elems(mode: GLenum, count: usize, index_type: GLenum) {
    unsafe {
        gl::DrawElements(mode, count as i32, index_type, std::ptr::null());
        gl_check();
    }
}

pub fn gl_draw_arrays(mode: GLenum, offset: usize, count: usize) {
    unsafe {
        gl::DrawArrays(mode, offset as i32, count as i32);
        gl_check();
    }
}

pub fn gl_finish() {
    unsafe {
        gl::Finish();
        gl_check();
    }
}