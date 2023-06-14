use glutin::event::{Event, VirtualKeyCode, ElementState, KeyboardInput, WindowEvent, DeviceEvent};
use glutin::event_loop::{ControlFlow, EventLoop};
use glam::*;

use crate::{Resources, Input, Graphics};

pub struct MainLoop {
    event_loop: EventLoop<()>
}

pub struct AppState<U: 'static> {
    pub resources: Resources,
    pub input: Input,
    pub graphics: Graphics,
    pub user_state: U
}

impl MainLoop {
    pub fn new() -> Self {
        let event_loop = EventLoop::new();

        MainLoop {
            event_loop
        }
    }

    pub(crate) fn internal_loop(&self) -> &EventLoop<()> {
        &self.event_loop
    }

    pub fn run<F0, F1, U>(self,
        init: F0,
        update: F1
    ) where
        F0: Fn(&mut Resources, &mut Graphics) -> U + 'static,
        F1: Fn(&mut AppState<U>) + 'static,
        U: 'static
    {
        let mut resources = Resources::new();
        let input = Input::new();
        let mut graphics = Graphics::new(&self, "Appearance", 512, 512);
        let user_state = init(&mut resources, &mut graphics);

        let mut app_state = AppState {
            resources,
            input,
            graphics,
            user_state
        };

        self.event_loop.run(move |event, _, control_flow| {
            match event {
                | Event::WindowEvent { event, .. } => {
                    match event {
                        | WindowEvent::CloseRequested => {
                            *control_flow = ControlFlow::Exit
                        },
                        | WindowEvent::Resized(size) => {
                            app_state.graphics.resize(size.width, size.height);
                        },
                        | WindowEvent::KeyboardInput { input, .. } => {
                            match input {
                                | KeyboardInput { virtual_keycode, state, .. } => {
                                    match (virtual_keycode, state) {
                                        | (Some(VirtualKeyCode::Escape), ElementState::Pressed) => {
                                            *control_flow = ControlFlow::Exit
                                        },
                                        | (Some(virtual_keycode), state) => {
                                            app_state.input.set_key(virtual_keycode, state == ElementState::Pressed);
                                        },
                                        | _ => {}
                                    }
                                },
                            }
                        },
                        | WindowEvent::MouseInput { state, button, .. } => {
                            app_state.input.set_mouse_button(button, state == ElementState::Pressed);
                        },
                        | WindowEvent::CursorMoved { position, .. } => {
                            app_state.input.set_mouse_pos(&IVec2::new(position.x as i32, position.y as i32));
                        }
                        | _ => {},
                    }
                },
                | Event::MainEventsCleared => {
                    app_state.graphics.window().internal_window().request_redraw();
                },
                | Event::RedrawRequested(_window_id) => {
                    update(&mut app_state);

                    app_state.resources.update();
                    app_state.input.update();
                    app_state.graphics.render();
                },
                | Event::LoopDestroyed => {
                    
                },
                | Event::DeviceEvent { event, ..} => {
                    match event {
                        | DeviceEvent::MouseMotion { delta } => {
                            app_state.input.set_mouse_delta(&Vec2::new(delta.0 as f32, delta.1 as f32));
                        },
                        | _ => {}
                    }
                },
                _ => (),
            }
        })
    }
}

impl Default for MainLoop {
    fn default() -> Self {
        MainLoop::new()
    }
}