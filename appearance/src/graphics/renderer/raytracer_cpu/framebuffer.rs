use std::ffi::c_void;
use gl_helper::*;
use glam::*;
use crate::Window;

pub(crate) static DISPLAY_SHADER_SRC_VERT: &str = "
#version 430 core
out vec2 texcoords;

void main() {
    vec2 vertices[3]=vec2[3](vec2(-1,-1), vec2(3,-1), vec2(-1, 3));
    gl_Position = vec4(vertices[gl_VertexID],0,1);
    texcoords = 0.5 * gl_Position.xy + vec2(0.5);
}
";

pub(crate) static DISPLAY_SHADER_SRC_FRAG: &str = "
#version 430 core
uniform sampler2D tex;
in vec2 texcoords;
out vec4 FragColor;

void main() {
    FragColor.rgb = pow(texture(tex, texcoords).rgb, vec3(2.2));
}
";

struct FramebufferTexture {
    texture: GLTexture,
    pbo: GLPBO,
    width: u32,
    height: u32,
    pixels: Vec<u32>,
}

impl FramebufferTexture {
    fn new(width: u32, height: u32) -> Self {
        let (texture, mut pbo, pixels);
        gl_pixel_store_i(gl::UNPACK_ALIGNMENT, 1); {
            let size = (width * height * std::mem::size_of::<u32>() as u32) as usize;
            texture = GLTexture::new(gl::TEXTURE_2D);
            pbo = GLPBO::new();
            pixels = vec![0; (width * height) as usize];

            texture.bind(); {
                gl_tex_parami(gl::TEXTURE_2D, gl::TEXTURE_WRAP_S, gl::CLAMP_TO_EDGE);
                gl_tex_parami(gl::TEXTURE_2D, gl::TEXTURE_WRAP_T, gl::CLAMP_TO_EDGE);
                gl_tex_parami(gl::TEXTURE_2D, gl::TEXTURE_MIN_FILTER, gl::NEAREST);
                gl_tex_parami(gl::TEXTURE_2D, gl::TEXTURE_MAG_FILTER, gl::NEAREST);
    
                gl_tex_image_2d(
                    gl::RGBA,
                    width as i32,
                    height as i32,
                    gl::RGBA,
                    gl::UNSIGNED_BYTE,
                    std::ptr::null::<c_void>()
                );
    
                pbo.bind();
                pbo.allocate(size);
                pbo.unbind();
            } texture.unbind();
        } gl_pixel_store_i(gl::UNPACK_ALIGNMENT, 4);

        FramebufferTexture {
            texture,
            pbo,
            width,
            height,
            pixels
        }
    }

    #[inline]
    fn set_pixel(&mut self, x: u32, y: u32, value: u32) {
        if x < self.width && y < self.height {
            self.pixels[(x + y * self.width) as usize] = value;
        }
    }

    fn bind(&self, slot: u32) {
        gl_active_texture(slot);
        self.texture.bind();
    }

    fn async_write(&self) {
        self.pbo.bind(); {
            let pixels = self.pbo.map() as *mut u32;
            unsafe {
                pixels.copy_from_nonoverlapping(self.pixels.as_ptr(), self.pixels.len());
            }
            self.pbo.unmap();
        } self.pbo.unbind();
    }
    
    fn flush_write(&self) {
        self.pbo.bind();
        gl_pixel_store_i(gl::UNPACK_ALIGNMENT, 1); {
            self.texture.bind();
            gl_tex_sub_image_2d(self.width as i32, self.height as i32, gl::RGBA, gl::UNSIGNED_BYTE, std::ptr::null::<c_void>());
            self.texture.unbind();
        } gl_pixel_store_i(gl::UNPACK_ALIGNMENT, 4);
        self.pbo.unbind();
    }
}

pub(crate) struct Framebuffer {
    framebuffer_textures: Vec<FramebufferTexture>,
    texture_idx: usize,
    prev_texture_idx: usize,

    display_program: GLShaderProgram,
    display_vao: GLVAO
}

impl Framebuffer {
    pub fn new(width: u32, height: u32) -> Self {
        gl_viewport(width, height);

        let framebuffer_textures = vec![
            FramebufferTexture::new(width, height),
            FramebufferTexture::new(width, height),
            FramebufferTexture::new(width, height)
        ];
        let texture_idx = 0;
        let prev_texture_idx = framebuffer_textures.len() - 1;

        let display_program = GLShaderProgram::new(
            &GLShader::new(GLShaderType::Vertex, DISPLAY_SHADER_SRC_VERT),
            &GLShader::new(GLShaderType::Fragment, DISPLAY_SHADER_SRC_FRAG)
        );
        let display_vao = GLVAO::new();

        Framebuffer {
            framebuffer_textures,
            texture_idx,
            prev_texture_idx,
            display_program,
            display_vao
        }
    }

    #[inline]
    pub fn width(&self) -> u32 {
        self.framebuffer_textures[0].width
    }

    #[inline]
    pub fn height(&self) -> u32 {
        self.framebuffer_textures[0].height
    }

    #[inline]
    pub fn set_pixel(&mut self, x: u32, y: u32, value: &Vec3) {
        let (r, g, b) = ((value.x * 255.99) as u32, (value.y * 255.99) as u32, (value.z * 255.99) as u32);
        let packed_value = (b << 16) | (g << 8) | r;

        self.framebuffer_textures[self.texture_idx].set_pixel(x, y, packed_value);
    }

    pub fn display(&mut self, window: &Window) {
        gl_clear_color(&Vec3::new(0.0, 0.0, 0.0));
        gl_clear();

        let current_texture = &self.framebuffer_textures[self.texture_idx];
        current_texture.async_write();

        self.display_program.bind(); {
            let prev_texture = &self.framebuffer_textures[self.prev_texture_idx];
            prev_texture.flush_write();
            prev_texture.bind(0);
            self.display_program.set_sampler_slot(&"tex".to_owned(), 0);

            self.display_vao.bind();
            gl_draw_arrays(gl::TRIANGLES, 0, 3);
        } self.display_program.unbind();

        window.internal_context().swap_buffers()
            .expect("Failed to swap buffers.");

        self.prev_texture_idx = self.texture_idx;
        self.texture_idx = (self.texture_idx + 1) % self.framebuffer_textures.len();
    }
}

impl Drop for Framebuffer {
    fn drop(&mut self) {
        for texture in &self.framebuffer_textures {
            texture.flush_write();
        }

        gl_finish();
    }
}