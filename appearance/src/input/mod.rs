pub use glutin::event::{MouseButton, VirtualKeyCode};

use crate::Window;
use glam::*;

const MAX_KEYS: usize = 512;
const MAX_BUTTONS: usize = 32;

/// The cursor mode.
/// - `FREE` the cursor is not restrained in any way.
/// - `LOCKED` the cursor is contained within the window and hidden.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum CursorMode {
    FREE,
    LOCKED
}

/// Input manager
pub struct Input {
    keys: [bool; MAX_KEYS],
    keys_prev: [bool; MAX_KEYS],
    buttons: [bool; MAX_BUTTONS],
    buttons_prev: [bool; MAX_BUTTONS],
    mouse_pos: IVec2,
    mouse_delta: Vec2,
    cursor_mode: CursorMode
}

impl Input {
    pub(crate) fn new() -> Self {
        Input {
            keys: [false; MAX_KEYS],
            keys_prev: [false; MAX_KEYS],
            buttons: [false; MAX_BUTTONS],
            buttons_prev: [false; MAX_BUTTONS],
            mouse_pos: IVec2::ZERO,
            mouse_delta: Vec2::ZERO,
            cursor_mode: CursorMode::FREE
        }
    }

    pub(crate) fn update(&mut self) {
        self.keys_prev = self.keys;
        self.buttons_prev = self.buttons;
        self.mouse_delta = Vec2::ZERO;
    }

    /// Check if key is pressed.
    pub fn key(&self, key_code: VirtualKeyCode) -> bool {
        self.keys[key_code as usize]
    }

    /// Check if key is pressed AND was not pressed previous frame.
    pub fn key_down(&self, key_code: VirtualKeyCode) -> bool {
        self.keys[key_code as usize] && !self.keys_prev[key_code as usize]
    }

    /// Check if mouse button is pressed.
    pub fn mouse_button(&self, button: MouseButton) -> bool {
        self.buttons[Self::mb_to_idx(button)]
    }

    /// Check if mouse button is pressed AND was not pressed previous frame.
    pub fn mouse_button_down(&self, button: MouseButton) -> bool {
        self.buttons[Self::mb_to_idx(button)] && !self.buttons_prev[Self::mb_to_idx(button)]
    }

    /// Get current mouse position in window space.
    pub fn mouse_pos(&self) -> IVec2 {
        self.mouse_pos
    }

    /// Get mouse velocity.
    pub fn mouse_delta(&self) -> Vec2 {
        self.mouse_delta
    }

    /// Get current cursor mode.
    pub fn get_cursor_mode(&self) -> CursorMode {
        self.cursor_mode
    }

    /// Set current cursor mode.
    pub fn set_cursor_mode(&mut self, mode: CursorMode, window: &Window) {
        let winit_window = window.internal_window();

        match mode {
            CursorMode::FREE => {
                winit_window.set_cursor_grab(glutin::window::CursorGrabMode::None)
                    .expect("Failed to free cursor.");
                winit_window.set_cursor_visible(true);
            },
            CursorMode::LOCKED => {
                let _ = winit_window.set_cursor_grab(glutin::window::CursorGrabMode::Confined)
                    .and_then(|_| {
                        winit_window.set_cursor_grab(glutin::window::CursorGrabMode::Locked)
                    });
                    winit_window.set_cursor_visible(false);
            }
        }

        self.cursor_mode = mode;
    }

    /// Toggle current cursor mode. `CursorMode::FREE` becomes `CursorMode::LOCKED` and and vice versa.
    pub fn toggle_cursor_mode(&mut self, window: &Window) {
        if self.cursor_mode == CursorMode::FREE {
            self.set_cursor_mode(CursorMode::LOCKED, window);
        } else {
            self.set_cursor_mode(CursorMode::FREE, window);
        }
    }

    pub(crate) fn set_key(&mut self, key_code: VirtualKeyCode, value: bool) {
        self.keys[key_code as usize] = value;
    }

    pub(crate) fn set_mouse_button(&mut self, button: MouseButton, value: bool) {
        self.buttons[Self::mb_to_idx(button)] = value;
    }

    pub(crate) fn set_mouse_pos(&mut self, mouse_pos: &IVec2) {
        self.mouse_pos = *mouse_pos;
    }

    pub(crate) fn set_mouse_delta(&mut self, mouse_delta: &Vec2) {
        self.mouse_delta = *mouse_delta;
    }

    fn mb_to_idx(button: MouseButton) -> usize {
        match button {
            MouseButton::Right => 0,
            MouseButton::Middle => 1,
            MouseButton::Left => 2,
            MouseButton::Other(i) => 3 + i as usize
        }
    }
}