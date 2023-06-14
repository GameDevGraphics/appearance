use std::rc::Rc;
use appearance::*;

fn main() {
    let main_loop = MainLoop::new();
    main_loop.run(init, update);
}

struct GameState {
    helmet_model: Rc<Model>
}

fn init(resources: &mut Resources, graphics: &mut Graphics) -> GameState {
    let helmet_model = resources.get_model("assets/models/DamagedHelmet/glTF/DamagedHelmet.gltf");
    graphics.add_model(helmet_model.clone());

    GameState {
        helmet_model
    }
}

fn update(app: &mut AppState<GameState>) {
    
}