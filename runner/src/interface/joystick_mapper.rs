
//! This is the example from the lib.rs documentation.

use pasts::Loop;
use std::{sync::mpsc, task::Poll::{self, Pending, Ready}};
use stick::{Controller, Event, Listener};

type Exit = usize;

#[derive(Debug, Clone, Copy)]
pub struct Mappedcoordinates {
    pub pitch: u16,
    pub roll: u16,
    pub yawn: u16,
    pub lift: u16,
    pub panic: bool,
}

pub struct State {
    listener: Listener,
    controllers: Vec<Controller>,
    rumble: (f32, f32),
    pub mapped: Mappedcoordinates,
    sender: mpsc::Sender<Mappedcoordinates>,
}

impl State {
    fn connect(&mut self, controller: Controller) -> Poll<Exit> {
        println!(
            "Connected p{}, id: {:016X}, name: {}",
            self.controllers.len() + 1,
            controller.id(),
            controller.name(),
        );
        self.controllers.push(controller);
        Pending
    }

    fn event(&mut self, id: usize, event: Event) -> Poll<Exit> {

        let player = id + 1;

        // println!("p{}: {}", player, event);
        match event {
            Event::Disconnect => {
                self.controllers.swap_remove(id);
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

            Event::CamZ(z) => {
                self.mapped.yawn = ((z + 1.0) / 2.0 * (u16::MAX as f64)) as u16;
                self.sender.send(self.mapped).unwrap();
            }

            Event::JoyZ(z) => {
                self.mapped.lift = ((z + 1.0) / 2.0 * (u16::MAX as f64)) as u16;
                self.sender.send(self.mapped).unwrap();
            }

            Event::Trigger(t) => {
                self.mapped.panic = t;
                self.sender.send(self.mapped).unwrap();
            }

            _ => {}
        }
        Pending
    }
}

pub async fn event_loop(sender: std::sync::mpsc::Sender<Mappedcoordinates>) {
    let mut state = State {
        listener: Listener::default(),
        controllers: Vec::new(),
        rumble: (0.0, 0.0),
        mapped: Mappedcoordinates { pitch: 0, roll: 0, yawn: 0, lift: 0, panic: false },
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


