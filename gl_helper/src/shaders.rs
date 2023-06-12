use super::*;
use glam::*;

use std::{mem, collections::HashMap};
use std::ffi::*;

/*****************************************************************************
*                               STRUCTS
******************************************************************************/

type GLShaderBuffer = gl::types::GLuint;
type GLShaderProgramBuffer = gl::types::GLuint;

pub struct GLShader {
    buffer: GLShaderBuffer
}

pub enum GLShaderType {
    Vertex,
    Fragment
}

pub struct GLShaderProgram {
    buffer: GLShaderProgramBuffer,
    uniform_locations: HashMap<String, i32>,
    uniforms: Option<Vec<GLUniform>>
}

#[derive(Clone)]
pub struct GLUniform {
    pub name: String,
    pub category: u32,
    pub size: i32
}

/*****************************************************************************
*                               IMPLEMENTATION
******************************************************************************/

impl GLShader {
    pub fn new(shader_type: GLShaderType, source: &str) -> Self {
        let buffer: GLShaderBuffer = match shader_type {
            GLShaderType::Vertex => gl_create_vert_shader(),
            GLShaderType::Fragment => gl_create_frag_shader()
        };

        gl_shader_source(buffer, source);
        gl_compile_shader(buffer);

        GLShader {
            buffer
        }
    }

    pub fn attach(&self, shader_program: &GLShaderProgram) {
        gl_attach_shader(self.buffer, shader_program.buffer());
    }
}

impl Drop for GLShader {
    fn drop(&mut self) {
        gl_del_shader(self.buffer);
    }
}

impl GLShaderProgram {
    pub fn new(vertex_shader: &GLShader, fragment_shader: &GLShader) -> GLShaderProgram {
        let program = GLShaderProgram {
            buffer: gl_create_program(),
            uniform_locations: HashMap::new(),
            uniforms: None
        };

        vertex_shader.attach(&program);
        fragment_shader.attach(&program);

        gl_link_program(program.buffer);

        program
    }

    pub fn buffer(&self) -> GLShaderProgramBuffer {
        self.buffer
    }

    pub fn bind(&self) {
        gl_use_program(self.buffer);
    }

    pub fn unbind(&self) {
        gl_use_program(0);
    }

    pub fn uniforms(&mut self) -> Vec<GLUniform> {
        match &self.uniforms {
            Some(uniforms) => uniforms.clone(),
            None => {
                let mut count: i32 = 0;
                unsafe {
                    gl::GetProgramiv(self.buffer, gl::ACTIVE_ATTRIBUTES, &mut count as *mut i32);
                    gl_check();
                }

                let mut uniforms: Vec<GLUniform> = Vec::new();

                for i in 0..count {
                    let mut name_data: [u8; 1024] = [0; 1024];
                    let mut name_length: i32 = 0;

                    let mut uniform_size: i32 = 0;
                    let mut uniform_type: u32 = 0;

                    unsafe {
                        gl::GetActiveUniform(self.buffer, i as u32, 1024, (&mut name_length) as *mut i32, (&mut uniform_size) as *mut i32, (&mut uniform_type) as *mut u32, name_data.as_mut_ptr() as *mut c_char);
                        gl_check();
                    }

                    let mut name_str: String = String::new();
                    for i in 0..name_length {
                        name_str.push(name_data[i as usize] as char);
                    }

                    uniforms.push(GLUniform {
                        name: name_str,
                        category: uniform_type,
                        size: uniform_size
                    })
                }

                self.uniforms = Some(uniforms);
                self.uniforms.clone().unwrap()
            }
        }
    }

    pub fn set_int(&mut self, name: &String, value: i32) {
        unsafe {
            gl::Uniform1i(self.uniform_location(name), value);
            gl_check();
        }
    }

    pub fn set_bool(&mut self, name: &String, value: bool) {
        unsafe {
            gl::Uniform1i(self.uniform_location(name), value as i32);
            gl_check();
        }
    }

    pub fn set_float(&mut self, name: &String, value: f32) {
        unsafe {
            gl::Uniform1f(self.uniform_location(name), value);
            gl_check();
        }
    }

    pub fn set_float3(&mut self, name: &String, value: &Vec3) {
        unsafe {
            gl::Uniform3f(self.uniform_location(name), value.x, value.y, value.z);
            gl_check();
        }
    }

    pub fn set_float4(&mut self, name: &String, value: &Vec4) {
        unsafe {
            gl::Uniform4f(self.uniform_location(name), value.x, value.y, value.z, value.w);
            gl_check();
        }
    }

    pub fn set_mat4(&mut self, name: &String, value: &Mat4) {
        unsafe {
            gl::UniformMatrix4fv(self.uniform_location(name), 1, gl::FALSE, value as *const Mat4 as *const f32);
            gl_check();
        }
    }

    pub fn set_sampler_slot(&mut self, name: &String, value: i32) {
        self.set_int(name, value);
    }

    fn uniform_location(&mut self, name: &String) -> i32 {
        match self.uniform_locations.get(name) {
            Some(location) => *location,
            None => {
                unsafe {
                    let mut cname = name.clone();
                    cname.push('\0');
                    
                    let location: i32 = gl::GetUniformLocation(self.buffer, cname.as_ptr() as *const i8);
                    gl_check();
                    if location < 0 {
                        eprintln!("Failed to get uniform location. (Name: '{}')", name);
                    }

                    self.uniform_locations.insert(name.clone(), location);
                    location
                }
            }
        }
    }
}

/*****************************************************************************
*                               HELPERS
******************************************************************************/

fn gl_create_vert_shader() -> GLShaderBuffer {
    unsafe {
        let shader: GLShaderBuffer = gl::CreateShader(gl::VERTEX_SHADER);
        gl_check();
        shader
    }
}

fn gl_create_frag_shader() -> GLShaderBuffer {
    unsafe {
        let shader: GLShaderBuffer = gl::CreateShader(gl::FRAGMENT_SHADER);
        gl_check();
        shader
    }
}

fn gl_shader_source(shader: GLShaderBuffer, source: &str) {
    let mut safe_source = source.to_owned();
    safe_source.push('\0');

    unsafe {
        let source_ptr_ptr = &(safe_source.as_ptr()) as *const *const u8;
        gl::ShaderSource(shader, 1, source_ptr_ptr as *const *const gl::types::GLchar, std::ptr::null());
        gl_check();
    }
}

fn gl_compile_shader(shader: GLShaderBuffer) {
    unsafe {
        gl::CompileShader(shader);
        gl_check();

        if cfg!(debug_assertions) {
            let mut buffer_data: [u8; 1024*32] = [0; 1024*32];
            let mut info_size: usize = 0;

            gl::GetShaderInfoLog(shader, (mem::size_of::<char>() * buffer_data.len()) as i32, (&mut info_size) as *mut usize as *mut i32, buffer_data.as_mut_ptr() as *mut c_char);
            gl_check();

            let mut buffer_str: String = String::new();
            for buffer_data in buffer_data {
                buffer_str.push(buffer_data as char);
            }

            if info_size > 0 && buffer_str.contains("error") {
               panic!("Failed to compile shader. \nOpenGL Error:\n{}\n", buffer_str);
            }
        }
    }
}

fn gl_attach_shader(shader: GLShaderBuffer, shader_program: GLShaderProgramBuffer) {
    unsafe {
        gl::AttachShader(shader_program, shader);
        gl_check();
    }
}

fn gl_del_shader(shader: GLShaderBuffer) {
    unsafe {
        gl::DeleteShader(shader);
        gl_check();
    }
}

fn gl_create_program() -> GLShaderProgramBuffer {
    unsafe {
        let program = gl::CreateProgram();
        gl_check();
        program
    }
}

fn gl_link_program(shader_program: GLShaderProgramBuffer) {
    unsafe {
        gl::LinkProgram(shader_program);
        gl_check();

        if cfg!(debug_assertions) {
            let mut buffer_data: [u8; 1024] = [0; 1024];
            let mut info_size: usize = 0;

            gl::GetProgramInfoLog(shader_program, (mem::size_of::<char>() * buffer_data.len()) as i32, (&mut info_size) as *mut usize as *mut i32, buffer_data.as_mut_ptr() as *mut c_char);
            gl_check();

            let mut buffer_str: String = String::new();
            for buffer_data in buffer_data {
                buffer_str.push(buffer_data as char);
            }

            if info_size > 0 && buffer_str.contains("error") {
               panic!("Failed to link program. \nOpenGL Error:\n{}\n", buffer_str);
            }
        }
    }
}

fn gl_use_program(shader_program: GLShaderProgramBuffer) {
    unsafe {
        gl::UseProgram(shader_program);
        gl_check();
    }
}