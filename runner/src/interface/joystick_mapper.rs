
use pasts::Loop;
use std::{sync::mpsc, task::Poll::{self, Pending, Ready}};
use stick::{Controller, Event, Listener};

type Exit = usize;

#[derive(Debug, Clone, Copy)]
/// Represents the mapped coordinates of the controller inputs.
pub struct Mappedcoordinates {
    pub pitch: u16,
    pub roll: u16,
    pub yaw: u16,
    pub lift: u16,
    pub abort: bool,
}
/// Represents the state of the event loop. It contains a listener for controller events, a vector of controllers, 
/// the current rumble settings, the current mapped controller inputs, and a sender channel for sending the mapped inputs to another thread.
pub struct State {
    listener: Listener,
    controllers: Vec<Controller>,
    rumble: (f32, f32),
    pub mapped: Mappedcoordinates,
    sender: mpsc::Sender<Mappedcoordinates>,
}
/// This code defines two methods for the State struct. connect is called when a new controller is connected, and event is 
/// called when a controller event occurs. Both methods return a Poll enum that indicates whether the event loop should continue (Pending) 
/// or exit (Ready with an exit code).
impl State {
    pub fn connect(&mut self, controller: Controller) -> Poll<Exit> {
        // println!(
        //     "\rConnected p{}, id: {:016X}, name: {}",
        //     self.controllers.len() + 1,
        //     controller.id(),
        //     controller.name(),
        // );
        self.controllers.push(controller);
        self.mapped.abort = false;
        Pending
    }

    pub fn event(&mut self, id: usize, event: Event) -> Poll<Exit> {

        let player = id + 1;

        match event {
            Event::Disconnect => {
                if self.controllers.len() > 0 {
                    // println!("\rJjoystick unplugged");
                    self.controllers.swap_remove(id);
                    self.mapped.abort = true;
                    self.sender.send(self.mapped).unwrap();
                }
            }
            Event::MenuR(true) => return Ready(player),
            Event::ActionA(pressed) => {
                self.controllers[id].rumble(f32::from(u8::from(pressed)));
            }
            Event::ActionB(pressed) => {
                self.controllers[id].rumble(0.5 * f32::from(u8::from(pressed)));
            }
            Event::BumperL(pressed) => {
                self.rumble.0 = f32::from(u8::from(pressed));
                self.controllers[id].rumble(self.rumble);
            }
            Event::BumperR(pressed) => {
                self.rumble.1 = f32::from(u8::from(pressed));
                self.controllers[id].rumble(self.rumble);
            }

            Event::JoyX(x) => {
                self.mapped.roll = ((x + 1.0) / 2.0 * (u16::MAX as f64)) as u16;
                self.sender.send(self.mapped).unwrap();
            }

            Event::JoyY(y) => {
                self.mapped.pitch = ((y + 1.0) / 2.0 * (u16::MAX as f64)) as u16;
                self.sender.send(self.mapped).unwrap();
            }

            // Yaw on real joystick
            // 0 - 16000
            // middle: 8000 - 8600
            Event::CamZ(z) => {
                self.mapped.yaw = ((z + 1.0) / 2.0 * (u16::MAX as f64)) as u16;
                self.sender.send(self.mapped).unwrap();
            }

            // Yaw on PS4 controller
            Event::CamX(x) => {

                self.mapped.yaw = ((x + 1.0) / 2.0 * (16000 as f64)) as u16;
                self.sender.send(self.mapped).unwrap();
            }

            Event::JoyZ(z) => {
                self.mapped.lift = ((z + 1.0) / 2.0 * (u16::MAX as f64)) as u16;
                // self.mapped.lift = u16::MAX - (z * (u16::MAX as f64)) as u16;
                self.sender.send(self.mapped).unwrap();
            }

            Event::Throttle(z) => {
                self.mapped.lift = u16::MAX - (z * (u16::MAX as f64)) as u16;
                self.sender.send(self.mapped).unwrap();
            }

            Event::Trigger(t) => {
                self.mapped.abort = t;
                self.sender.send(self.mapped).unwrap();
            }

            _ => {}
        }
        Pending
    }
}

/// Defines the main function of the module. It creates a new State object, sets up a loop to handle controller events, and sends mapped controller inputs 
/// over the sender channel.
pub async fn event_loop(sender: mpsc::Sender<Mappedcoordinates>) {
    let mut state = State {
        listener: Listener::default(),
        controllers: Vec::new(),
        rumble: (0.0, 0.0),
        mapped: Mappedcoordinates { pitch: 0, roll: 0, yaw: 0, lift: 0, abort: false },
        sender: sender,
    };

    let player_id = Loop::new(&mut state)
        .when(|s| &mut s.listener, State::connect)
        .poll(|s| &mut s.controllers, State::event)
        .await;

    println!("p{} ended the session", player_id);
}

#[cfg(test)]
mod tests {
use super::*;

    #[test]
    fn main() {
        let (tx, rx) = mpsc::channel();
        
        std::thread::spawn(|| {
            pasts::block_on(event_loop(tx));
        });

        while let Ok(packet) = rx.recv(){
            println!("{:?}", packet);
        }
    }    
}


