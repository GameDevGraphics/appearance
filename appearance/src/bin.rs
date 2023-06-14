use std::rc::Rc;
use appearance::*;
use glam::*;

fn main() {
    let main_loop = MainLoop::new();
    main_loop.run(init, update);
}

struct GameState {
    helmet_model: Rc<Model>,
    timer: Timer
}

fn init(resources: &mut Resources, graphics: &mut Graphics) -> GameState {
    let helmet_model = resources.get_model("assets/models/AnimatedCube/glTF/AnimatedCube.gltf");
    graphics.add_model(helmet_model.clone());

    GameState {
        helmet_model,
        timer: Timer::new()
    }
}

fn update(app: &mut AppState<GameState>) {
    let dt = app.user_state.timer.elapsed() as f32;
    println!("FPS: {}", 1.0 / dt);
    app.user_state.timer.reset();

    let camera = app.graphics.camera();
    let mut dir = Vec3::ZERO;
    if app.input.key(VirtualKeyCode::W) {
        dir += Vec3::new(0.0, 0.0, 1.0);
    }
    if app.input.key(VirtualKeyCode::S) {
        dir += Vec3::new(0.0, 0.0, -1.0);
    }
    if app.input.key(VirtualKeyCode::A) {
        dir += Vec3::new(-1.0, 0.0, 0.0);
    }
    if app.input.key(VirtualKeyCode::D) {
        dir += Vec3::new(1.0, 0.0, 0.0);
    }
    if app.input.key(VirtualKeyCode::Q) {
        dir += Vec3::new(0.0, -1.0, 0.0);
    }
    if app.input.key(VirtualKeyCode::E) {
        dir += Vec3::new(0.0, 1.0, 0.0);
    }
    let pos = *camera.get_position() + dir * dt * 1.0;
    camera.set_position(&pos);
}