use glam::*;

#[derive(Clone, Copy, Debug)]
pub struct Transform {
    position: Vec3,
    rotation: Quat,
    scale: Vec3,

    dirty: bool,
    model_matrix: Mat4,
    inv_trans_model_matrix: Mat4
}

impl Default for Transform {
    fn default() -> Self {
        Transform::new()
    }
}

impl Transform {
    pub fn new() -> Self {
        Transform {
            position: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
            dirty: true,
            model_matrix: Mat4::IDENTITY,
            inv_trans_model_matrix: Mat4::IDENTITY
        }
    }

    pub fn get_position(&self) -> &Vec3 {
        &self.position
    }

    pub fn get_rotation(&self) -> &Quat {
        &self.rotation
    }

    pub fn get_scale(&self) -> &Vec3 {
        &self.scale
    }

    pub fn set_position(&mut self, position: &Vec3) {
        self.position = *position;
        self.dirty = true;
    }

    pub fn set_rotation(&mut self, rotation: &Quat) {
        self.rotation = *rotation;
        self.dirty = true;
    }

    pub fn set_scale(&mut self, scale: &Vec3) {
        self.scale = *scale;
        self.dirty = true;
    }

    fn recalculate_matrices(&mut self) {
        if self.dirty {
            self.dirty = false;
            self.model_matrix = Mat4::from_translation(self.position) * Mat4::from_quat(self.rotation) * Mat4::from_scale(self.scale);
            self.inv_trans_model_matrix = self.model_matrix.inverse().transpose();
        }
    }

    pub fn get_model_matrix(&mut self) -> &Mat4 {
        self.recalculate_matrices();
        &self.model_matrix
    }

    pub fn get_inv_trans_model_matrix(&mut self) -> &Mat4 {
        self.recalculate_matrices();
        &self.inv_trans_model_matrix
    }
}