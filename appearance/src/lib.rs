#![feature(portable_simd)]
#![allow(clippy::collapsible_else_if)]

use glam::Vec3;
pub static RIGHT: Vec3 = Vec3::new(1.0, 0.0, 0.0);
pub static UP: Vec3 = Vec3::new(0.0, 1.0, 0.0);
pub static FORWARD: Vec3 = Vec3::new(0.0, 0.0, -1.0);

mod main_loop;
pub use main_loop::*;

mod resources;
pub use resources::*;
mod input;
pub use input::*;
mod graphics;
pub use graphics::*;


mod timer;
pub use timer::Timer;