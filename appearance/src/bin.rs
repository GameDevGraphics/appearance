use std::rc::Rc;

use appearance::*;
use glam::*;

fn main() {
    let main_loop = MainLoop::new();
    main_loop.run(init, update);
}

struct GameState {
    helmet_ids: Vec<Rc<MeshRendererID>>,
    timer: Timer,
    delta_timer: Timer,

    camera_yaw: f32,
    camera_pitch: f32
}

fn init(resources: &mut Resources, graphics: &mut Graphics) -> GameState {
    let helmet_model = resources.get_model("assets/models/DamagedHelmet/glTF/DamagedHelmet.gltf");

    let mut helmet_ids = Vec::new();
    for x in 0..4 {
        for y in 0..4 {
            for z in 0..4 {
                let mut transform = Transform::new();
                transform.set_position(&Vec3::new(x as f32 * 2.0, y as f32 * 2.0, z as f32 * 2.0));

                let helmet_id = graphics.add_mesh(
                    helmet_model.root_nodes[0].mesh.as_ref().unwrap(),
                    Some(transform)
                );
                helmet_ids.push(helmet_id);
            }
        }
    }

    GameState {
        helmet_ids,
        timer: Timer::new(),
        delta_timer: Timer::new(),
        camera_yaw: 0.0,
        camera_pitch: 0.0
    }
}

fn update(app: &mut AppState<GameState>) {
    // let window = app.graphics.window();
    // app.input.set_cursor_mode(CursorMode::LOCKED, window);

    // FPS Counter
    let dt = app.user_state.delta_timer.elapsed() as f32;
    let time = app.user_state.timer.elapsed() as f32;
    println!("FPS: {}", 1.0 / dt);
    app.user_state.delta_timer.reset();

    // Camera controller
    let camera = app.graphics.camera();
    let mut dir = Vec3::ZERO;
    if app.input.key(VirtualKeyCode::W) {
        dir += camera.forward();
    }
    if app.input.key(VirtualKeyCode::S) {
        dir += -camera.forward();
    }
    if app.input.key(VirtualKeyCode::A) {
        dir += -camera.right();
    }
    if app.input.key(VirtualKeyCode::D) {
        dir += camera.right();
    }
    if app.input.key(VirtualKeyCode::Q) {
        dir += -camera.up();
    }
    if app.input.key(VirtualKeyCode::E) {
        dir += camera.up();
    }
    let pos = *camera.get_position() + dir * dt * 1.0;
    camera.set_position(&pos);

    // let look_offset = app.input.mouse_delta() * 0.7;
    // app.user_state.camera_pitch += look_offset.y;
    // app.user_state.camera_yaw += look_offset.x;
    // app.user_state.camera_pitch = (app.user_state.camera_pitch).clamp(-90.0, 90.0);
    // camera.set_rotation(&Quat::from_euler(
    //     EulerRot::XYZ,
    //     (app.user_state.camera_pitch).to_radians(),
    //     (app.user_state.camera_yaw).to_radians(),
    //     0.0)
    // );

    // Remove first helmet
    if app.input.key_down(VirtualKeyCode::Space) {
        app.user_state.helmet_ids.remove(0);
    }

    // Spin all helmets
    for helmet_id in &app.user_state.helmet_ids {
        let helmet_renderer = app.graphics.mesh_renderer(helmet_id.clone());
        helmet_renderer.transform.set_rotation(
            &Quat::from_axis_angle(
                Vec3::new(0.0, 1.0, 0.0),
                (time * 15.0).to_radians()
            )
        );
    }
}