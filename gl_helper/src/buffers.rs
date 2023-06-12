use super::*;

use std::ffi::*;

/*****************************************************************************
*                               STRUCTS
******************************************************************************/

pub type GLBuffer = gl::types::GLuint;

#[allow(clippy::upper_case_acronyms)]
pub struct GLVAO {
    buffer: GLBuffer
}

#[allow(clippy::upper_case_acronyms)]
pub struct GLVBO {
    buffer: GLBuffer
}

#[allow(clippy::upper_case_acronyms)]
pub struct GLEBO {
    buffer: GLBuffer
}

#[allow(clippy::upper_case_acronyms)]
pub struct GLFBO {
    buffer: GLBuffer
}

#[allow(clippy::upper_case_acronyms)]
pub struct GLRBO {
    buffer: GLBuffer
}

#[allow(clippy::upper_case_acronyms)]
pub struct GLPBO {
    buffer: GLBuffer,
    size: usize
}

/*****************************************************************************
*                               FUNCS
******************************************************************************/

pub fn gl_vertex_attrib_ptr<T>(index: u32, size: usize, stride: usize, ptr: *const T) {
    unsafe {
        gl::VertexAttribPointer(index, size as i32, gl::FLOAT, gl::FALSE, stride as i32, ptr as *const c_void);
        gl_check();
    }
}

pub fn gl_enable_vertex_attrib_array(index: u32) {
    unsafe {
        gl::EnableVertexAttribArray(index);
        gl_check();
    }
}

pub fn gl_frame_buffer_texture_2d(texture: &GLTexture, attachment: GLenum) {
    unsafe {
        gl::FramebufferTexture2D(gl::FRAMEBUFFER, attachment, gl::TEXTURE_2D, texture.handle(), 0);
        gl_check();
    }
}

pub fn gl_render_buffer_storage(format: GLenum, width: i32, height: i32) {
    unsafe {
        gl::RenderbufferStorage(gl::RENDERBUFFER, format, width, height);
        gl_check();
    }
}

pub fn gl_frame_buffer_render_buffer(rbo: &GLRBO, attachment: GLenum) {
    unsafe {
        gl::FramebufferRenderbuffer(gl::FRAMEBUFFER, attachment, gl::RENDERBUFFER, rbo.handle());
        gl_check();
    }
}

#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub fn gl_draw_buffers(count: usize, buffers: *const GLenum) {
    unsafe {
        gl::DrawBuffers(count as i32, buffers);
        gl_check();
    }
}

/*****************************************************************************
*                               IMPLEMENTATION
******************************************************************************/

pub trait IGLBuffer {
    fn new() -> Self;

    fn bind(&self);
    fn unbind(&self);

    fn set_data<T>(&self, size: usize, data: *mut T);
}

impl IGLBuffer for GLVAO {
    fn new() -> Self {
        GLVAO {
            buffer: gl_gen_vert_array()
        }
    }

    fn bind(&self) {
        unsafe {
            gl::BindVertexArray(self.buffer);
            gl_check();
        }
    }

    fn unbind(&self) {
        unsafe {
            gl::BindVertexArray(0);
            gl_check();
        }
    }

    fn set_data<T>(&self, _: usize, _: *mut T) {
        panic!("Failed to set data on VAO.")
    }
}

impl Drop for GLVAO {
    fn drop(&mut self) {
        gl_del_vert_array(self.buffer);
    }
}



impl IGLBuffer for GLVBO {
    fn new() -> Self {
        GLVBO {
            buffer: gl_gen_buffer()
        }
    }

    fn bind(&self) {
        unsafe {
            gl::BindBuffer(gl::ARRAY_BUFFER, self.buffer);
            gl_check();
        }
    }

    fn unbind(&self) {
        unsafe {
            gl::BindBuffer(gl::ARRAY_BUFFER, 0);
            gl_check();
        }
    }

    fn set_data<T>(&self, size: usize, data: *mut T) {
        unsafe {
            gl::BufferData(gl::ARRAY_BUFFER, size as isize, data as *mut c_void, gl::STATIC_DRAW);
            gl_check();
        }
    }
}

impl Drop for GLVBO {
    fn drop(&mut self) {
        gl_del_buffer(self.buffer);
    }
}

impl IGLBuffer for GLEBO {
    fn new() -> Self {
        GLEBO {
            buffer: gl_gen_buffer()
        }
    }

    fn bind(&self) {
        unsafe {
            gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, self.buffer);
            gl_check();
        }
    }

    fn unbind(&self) {
        unsafe {
            gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, 0);
            gl_check();
        }
    }

    fn set_data<T>(&self, size: usize, data: *mut T) {
        unsafe {
            gl::BufferData(gl::ELEMENT_ARRAY_BUFFER, size as isize, data as *mut c_void, gl::STATIC_DRAW);
            gl_check();
        }
    }
}

impl Drop for GLEBO {
    fn drop(&mut self) {
        gl_del_buffer(self.buffer);
    }
}

impl IGLBuffer for GLFBO {
    fn new() -> Self {
        GLFBO {
            buffer: gl_gen_frame_buffer()
        }
    }

    fn bind(&self) {
        unsafe {
            gl::BindFramebuffer(gl::FRAMEBUFFER, self.buffer);
            gl_check();
        }
    }

    fn unbind(&self) {
        unsafe {
            gl::BindFramebuffer(gl::FRAMEBUFFER, 0);
            gl_check();
        }
    }

    fn set_data<T>(&self, _: usize, _: *mut T) {
        panic!("Failed to set data on FBO. (Should never be called)");
    }
}

impl GLFBO {
    pub fn check_status(&self) {
        unsafe {
            assert_eq!(gl::CheckFramebufferStatus(gl::FRAMEBUFFER), gl::FRAMEBUFFER_COMPLETE, "Failed to bind frame buffer.");
        }
    }
}

impl Drop for GLFBO {
    fn drop(&mut self) {
        gl_del_frame_buffer(self.buffer);
    }
}

impl IGLBuffer for GLRBO {
    fn new() -> Self {
        GLRBO {
            buffer: gl_gen_render_buffer()
        }
    }

    fn bind(&self) {
        unsafe {
            gl::BindRenderbuffer(gl::RENDERBUFFER, self.buffer);
            gl_check();
        }
    }

    fn unbind(&self) {
        unsafe {
            gl::BindRenderbuffer(gl::RENDERBUFFER, 0);
            gl_check();
        }
    }

    fn set_data<T>(&self, _: usize, _: *mut T) {
        panic!("Failed to set data on RBO. (Should never be called)");
    }
}

impl GLRBO {
    pub fn handle(&self) -> GLBuffer {
        self.buffer
    }
}

impl Drop for GLRBO {
    fn drop(&mut self) {
        gl_del_render_buffer(self.buffer);
    }
}

impl IGLBuffer for GLPBO {
    fn new() -> Self {
        GLPBO {
            buffer: gl_gen_buffer(),
            size: 0
        }
    }

    fn bind(&self) {
        unsafe {
            gl::BindBuffer(gl::PIXEL_UNPACK_BUFFER, self.buffer);
            gl_check();
        }
    }

    fn unbind(&self) {
        unsafe {
            gl::BindBuffer(gl::PIXEL_UNPACK_BUFFER, 0);
            gl_check();
        }
    }

    fn set_data<T>(&self, _: usize, _: *mut T) {
        panic!("Failed to set data on RBO. (Should never be called)");
    }
}

impl GLPBO {
    pub fn handle(&self) -> GLBuffer {
        self.buffer
    }

    pub fn allocate(&mut self, size: usize) {
        self.size = size;

        unsafe {
            gl::BufferData(
                gl::PIXEL_UNPACK_BUFFER,
                size as isize,
                std::ptr::null(),
                gl::STREAM_DRAW
            );
            gl_check();
        }
    }

    pub fn map<T>(&self) -> *mut T {
        unsafe {
            let data_ptr = gl::MapBufferRange(
                gl::PIXEL_UNPACK_BUFFER,
                0,
                self.size as isize,
                gl::MAP_WRITE_BIT
            ) as *mut T;
            gl_check();
            data_ptr
        }
    }

    pub fn unmap(&self) {
        unsafe {
            gl::UnmapBuffer(gl::PIXEL_UNPACK_BUFFER);
            gl_check();
        }
    }
}

impl Drop for GLPBO {
    fn drop(&mut self) {
        gl_del_buffer(self.buffer);
    }
}

/*****************************************************************************
*                               HELPERS
******************************************************************************/

fn gl_gen_buffer() -> GLBuffer {
    let mut buffer: u32 = 0;
    unsafe {
        gl::GenBuffers(1, &mut buffer);
        gl_check();
    }
    buffer
}

fn gl_gen_vert_array() -> GLBuffer {
    let mut buffer: u32 = 0;
    unsafe {
        gl::GenVertexArrays(1, &mut buffer);
        gl_check();
    }
    buffer
}

fn gl_gen_frame_buffer() -> GLBuffer {
    let mut buffer: u32 = 0;
    unsafe {
        gl::GenFramebuffers(1, &mut buffer);
        gl_check();
    }
    buffer
}

fn gl_gen_render_buffer() -> GLBuffer {
    let mut buffer: u32 = 0;
    unsafe {
        gl::GenRenderbuffers(1, &mut buffer);
        gl_check();
    }
    buffer
}

fn gl_del_buffer(buffer: GLBuffer) {
    unsafe {
        gl::DeleteBuffers(1, &buffer);
        gl_check();
    }
}

fn gl_del_vert_array(buffer: GLBuffer) {
    unsafe {
        gl::DeleteVertexArrays(1, &buffer);
        gl_check();
    }
}

fn gl_del_frame_buffer(buffer: GLBuffer) {
    unsafe {
        gl::DeleteFramebuffers(1, &buffer);
        gl_check();
    }
}

fn gl_del_render_buffer(buffer: GLBuffer) {
    unsafe {
        gl::DeleteRenderbuffers(1, &buffer);
        gl_check();
    }
}