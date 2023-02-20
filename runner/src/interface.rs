use std::io::{stdin, stdout, Write};
use termion::event::Key;
use termion::input::TermRead;
use termion::raw::IntoRawMode;

/// Starts interface to read joystick/keyboard inputs and maps it to a command
pub fn start_interface() {
    let stdin = stdin();
    //setting up stdout and going into raw mode. 
    // This makes it possible to read one byte without enter
    let mut stdout = stdout().into_raw_mode().unwrap();

    stdout.flush().unwrap();

    //detecting keydown events
    for c in stdin.keys() {
 
        // clear terminal
        write!(
            stdout,
            "{}{}",
            termion::cursor::Goto(1, 1),
            termion::clear::All
        )
        .unwrap();

        // keyboard mapper
        match c.unwrap() {
            //Key::Char(x) => println!("{} pressed", x),
            Key::Esc => println!("go to safe mode, through panic mode"),
            Key::Char(' ') => println!("go to safe mode, through panic mode"),
            Key::Char('0') => println!("mode 0"),
            Key::Char('1') => println!("mode 1"),
            Key::Char('a') => println!("lift up"),
            Key::Char('z') => println!("lift down"),
            Key::Char('q') => println!("yaw down"),
            Key::Char('w') => println!("yaw up"),
            Key::Char('u') => println!("yaw control P up"),
            Key::Char('j') => println!("yaw control P down"),
            Key::Char('i') => println!("roll/pitch control P1 up/down"),
            Key::Char('k') => println!("roll/pitch control P1 up/down"),
            Key::Char('o') => println!("roll/pitch control P2 up/down"),
            Key::Char('l') => println!("roll/pitch control P2 up/down"),
            
            Key::Char(x) => println!("{} pressed", x),
            
            Key::Ctrl('q') => break,
            _ => (),
        }

        stdout.flush().unwrap();
    }
}
