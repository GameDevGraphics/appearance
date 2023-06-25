use std::{rc::Rc, collections::VecDeque, sync::Arc};

use appearance::*;
use glam::*;

fn main() {
    let main_loop = MainLoop::new();
    main_loop.run(init, update);
}

struct GameState {
    _pica_ids: Vec<Rc<MeshRendererID>>,
    timer: Timer,
    delta_timer: Timer,

    camera_yaw: f32,
    camera_pitch: f32,
    last_dts: VecDeque<f32>
}

fn load_recursive(graphics: &mut Graphics, ids: &mut Vec<Rc<MeshRendererID>>, materials: &Vec<Arc<Material>>, current_node: Arc<ModelNode>, mut transform_matrix: Mat4) {
    transform_matrix *= *Transform::from_position_rotation_scale(
        current_node.position,
        current_node.rotation,
        current_node.scale
    ).get_model_matrix();

    if let Some(mesh) = &current_node.mesh {
        ids.push(graphics.add_mesh(
            mesh.clone(),
            materials[mesh.material_idx].clone(),
            Some(Transform::from_matrix(&transform_matrix))
        ));
    }

    for child in &current_node.children {
        load_recursive(graphics, ids, materials, child.clone(), transform_matrix);
    }
}

fn init(resources: &mut Resources, graphics: &mut Graphics) -> GameState {
    let pica_model = resources.get_model("assets/models/pica_pica/scene.gltf");

    let mut root_transform = Transform::new();
    root_transform.set_scale(&Vec3::splat(35.0));

    let mut pica_ids = Vec::new();
    load_recursive(graphics, &mut pica_ids, &pica_model.materials, pica_model.root_nodes[0].clone(), *root_transform.get_model_matrix());

    GameState {
        _pica_ids: pica_ids,
        timer: Timer::new(),
        delta_timer: Timer::new(),
        camera_yaw: 0.0,
        camera_pitch: 0.0,
        last_dts: VecDeque::new()
    }
}

fn update(app: &mut AppState<GameState>) {
    let window = app.graphics.window();
    app.input.set_cursor_mode(CursorMode::LOCKED, window);

    // FPS Counter
    let dt = app.user_state.delta_timer.elapsed() as f32;
    app.user_state.delta_timer.reset();
    let time = app.user_state.timer.elapsed() as f32;

    // Last 100 frames
    let last_dts = &mut app.user_state.last_dts;
    if last_dts.len() >= 100 { last_dts.pop_front(); }
    last_dts.push_back(dt);
    let avg = last_dts.iter().sum::<f32>() / last_dts.len() as f32;
    println!("ms: {:.2}", avg * 1000.0);

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

    let look_offset = app.input.mouse_delta() * 0.7;
    app.user_state.camera_pitch += look_offset.y;
    app.user_state.camera_yaw += look_offset.x;
    app.user_state.camera_pitch = (app.user_state.camera_pitch).clamp(-90.0, 90.0);
    camera.set_rotation(&Quat::from_euler(
        EulerRot::XYZ,
        (app.user_state.camera_pitch).to_radians(),
        (app.user_state.camera_yaw).to_radians(),
        0.0)
    );
}