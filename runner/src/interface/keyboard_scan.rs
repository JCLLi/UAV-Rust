#![allow(unused)]
use std::io::{stdin, stdout, Write};
use std::path::PathBuf;
use std::thread::sleep;
use std::time::Duration;
use termion::event::Key;
use termion::input::TermRead;
use termion::raw::IntoRawMode;
use serial2::SerialPort;
use device_query::{DeviceQuery, DeviceState, Keycode};
use tudelft_serial_upload::{upload_file_or_stop, PortSelector};

///Keyboard scan,a slice of u8 will be returned which stands for a serialized command
pub fn keyboard_scan() -> &'static [u8]{
    //New keyboard scanner
    let device_state = DeviceState::new();
    let mut keys: Vec<Keycode> = Vec::new();

    //loop_count and sleep() are both used for avoid key jitter
    let mut loop_count = 0;
    loop {
        loop_count += 1;
        sleep(Duration::from_micros(1700));
        keys = device_state.get_keys();
        if loop_count == 60{
            loop_count = 0;

            if !keys.is_empty(){
                let key = keys[0];
                keys.clear();

                //TODO key mapping to serialized protocol message
                match key {
                    Keycode::Escape | Keycode::Space => return b"go to safe mode, through panic mode",
                    Keycode::Key0 => return b"mode0",
                    Keycode::Key1 => return b"mode 1",
                    Keycode::A => return b"lift up",
                    Keycode::Z => return b"lift down",
                    Keycode::Left => return b"roll up",
                    Keycode::Right => return b"roll down",
                    Keycode::Up => return b"Pitch down",
                    Keycode::Down => return b"Pitch up",
                    Keycode::Q => return b"yaw down",
                    Keycode::W => return b"yaw up",
                    Keycode::U => return b"yaw control P up",
                    Keycode::J => return b"yaw control P down",
                    Keycode::I => return b"roll/pitch control P1 up/down",
                    Keycode::K => return b"roll/pitch control P1 up/down",
                    Keycode::O => return b"roll/pitch control P2 up/down",
                    Keycode::L => return b"roll/pitch control P2 up/down",
                    _ => (),
                }
            }
       }
    }
}