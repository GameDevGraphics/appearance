use super::*;

use std::ffi::*;

/*****************************************************************************
*                               STRUCTS
******************************************************************************/

pub type GLTextureBuffer = gl::types::GLuint;
type GLTextureType = gl::types::GLenum;

pub struct GLTexture {
    buffer: GLTextureBuffer,
    target: GLTextureType
}

pub type GLPackAlignment = gl::types::GLuint;

/*****************************************************************************
*                               FUNCS
******************************************************************************/

pub fn gl_tex_parami(target: GLTextureType, name: GLenum, param: u32) {
    unsafe {
        gl::TexParameteri(target, name, param as i32);
        gl_check();
    }
}

pub fn gl_gen_mips(target: GLTextureType) {
    unsafe {
        gl::GenerateMipmap(target);
        gl_check();
    }
}

pub fn gl_tex_image_2d<T>(internal_format: u32, width: i32, height: i32, format: u32, ty: u32, data: *const T) {
    unsafe {
        gl::TexImage2D(
            gl::TEXTURE_2D,
            0,
            internal_format as i32,
            width,
            height,
            0,
            format,
            ty,
            data as *const c_void
        );
        gl_check();
    }
}

pub fn gl_tex_sub_image_2d<T>(width: i32, height: i32, format: u32, ty: u32, data: *const T) {
    unsafe {
        gl::TexSubImage2D(
            gl::TEXTURE_2D,
            0,
            0,
            0,
            width,
            height,
            format,
            ty,
            data as *const c_void
        );
        gl_check();
    }
}

pub fn gl_active_texture(slot: u32) {
    unsafe {
        gl::ActiveTexture(gl::TEXTURE0 + slot);
        gl_check();
    }
}

pub fn gl_pixel_store_i(alignment: GLPackAlignment, count: i32) {
    unsafe {
        gl::PixelStorei(alignment, count);
        gl_check();
    }
}

/*****************************************************************************
*                               IMPLEMENTATION
******************************************************************************/

impl GLTexture {
    pub fn new(target: GLTextureType) -> Self {
        GLTexture {
            buffer: gl_gen_texture(),
            target
        }
    }

    pub fn bind(&self) {
        unsafe {
            gl::BindTexture(self.target, self.buffer);
            gl_check();
        }
    }

    pub fn unbind(&self) {
        unsafe {
            gl::BindTexture(self.target, 0);
            gl_check();
        }
    }

    pub fn handle(&self) -> GLTextureBuffer {
        self.buffer
    }

    pub fn target(&self) -> GLTextureType {
        self.target
    }
}

impl Drop for GLTexture {
    fn drop(&mut self) {
        gl_del_texture(self.buffer);
    }
}

/*****************************************************************************
*                               HELPERS
******************************************************************************/

fn gl_gen_texture() -> GLTextureBuffer {
    unsafe {
        let mut buffer: GLTextureBuffer = 0;
        gl::GenTextures(1, &mut buffer as *mut GLTextureBuffer);
        gl_check();
        buffer
    }
}

fn gl_del_texture(texture: GLTextureBuffer) {
    unsafe {
        gl::DeleteTextures(1, &texture as *const u32);
        gl_check();
    }
}