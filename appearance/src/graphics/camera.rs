use glam::*;
use crate::{RIGHT, UP, FORWARD};

pub struct Camera {
    position: Vec3,
    rotation: Quat,

    fov: f32,
    aspect_ratio: f32,
    near: f32,
    far: f32,

    view_matrix: Mat4,
    proj_matrix: Mat4,
    view_inv_matrix: Mat4,
    proj_inv_matrix: Mat4,
    dirty_view: bool,
    dirty_proj: bool
}

impl Default for Camera {
    fn default() -> Self {
        Camera::new()
    }
}

impl Camera {
    pub fn new() -> Camera {
        Camera {
            position: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            fov: 60.0,
            aspect_ratio: 1.0,
            near: 0.1,
            far: 300.0,
            view_matrix: Mat4::IDENTITY,
            proj_matrix: Mat4::IDENTITY,
            view_inv_matrix: Mat4::IDENTITY,
            proj_inv_matrix: Mat4::IDENTITY,
            dirty_view: true,
            dirty_proj: true
        }
    }

    pub fn right(&mut self) -> Vec3 {
        (*self.view_inv_matrix() * Vec4::from((RIGHT, 0.0))).xyz()
    }

    pub fn up(&mut self) -> Vec3 {
        (*self.view_inv_matrix() * Vec4::from((UP, 0.0))).xyz()
    }

    pub fn forward(&mut self) -> Vec3 {
        (*self.view_inv_matrix() * Vec4::from((FORWARD, 0.0))).xyz()
    }

    pub fn get_position(&self) -> &Vec3 {
        &self.position
    }

    pub fn get_rotation(&self) -> &Quat {
        &self.rotation
    }

    pub fn set_position(&mut self, position: &Vec3) {
        self.position = *position;
        self.dirty_view = true;
    }

    pub fn set_rotation(&mut self, rotation: &Quat) {
        self.rotation = *rotation;
        self.dirty_view = true;
    }

    pub fn get_fov(&self) -> f32 {
        self.fov
    }

    pub fn get_aspect_ratio(&self) -> f32 {
        self.aspect_ratio
    }

    pub fn get_near(&self) -> f32 {
        self.near
    }

    pub fn get_far(&self) -> f32 {
        self.far
    }

    pub fn set_fov(&mut self, fov: f32) {
        self.fov = fov;
        self.dirty_proj = true;
    }

    pub fn set_aspect_ratio(&mut self, aspect_ratio: f32) {
        self.aspect_ratio = aspect_ratio;
        self.dirty_proj = true;
    }

    pub fn set_near(&mut self, near: f32) {
        self.near = near;
        self.dirty_proj = true;
    }

    pub fn set_far(&mut self, far: f32) {
        self.far = far;
        self.dirty_proj = true;
    }

    fn recalculate_view(&mut self) {
        if self.dirty_view {
            self.dirty_view = false;
            self.view_matrix = Mat4::from_quat(self.rotation) * Mat4::from_translation(self.position);
            self.view_inv_matrix = self.view_matrix.inverse();
        }
    }

    fn recalculate_proj(&mut self) {
        if self.dirty_proj {
            self.dirty_proj = false;
            self.proj_matrix = Mat4::perspective_rh(
                self.fov.to_radians(),
                self.aspect_ratio,
                self.near,
                self.far
            );
            self.proj_inv_matrix = self.proj_matrix.inverse();
        }
    }

    pub fn view_matrix(&mut self) -> &Mat4 {
        self.recalculate_view();
        &self.view_matrix
    }

    pub fn view_inv_matrix(&mut self) -> &Mat4 {
        self.recalculate_view();
        &self.view_inv_matrix
    }

    pub fn proj_matrix(&mut self) -> &Mat4 {
        self.recalculate_proj();
        &self.proj_matrix
    }

    pub fn proj_inv_matrix(&mut self) -> &Mat4 {
        self.recalculate_proj();
        &self.proj_inv_matrix
    }
}