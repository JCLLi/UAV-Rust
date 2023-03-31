use protocol::{self, Packet, Message, WorkingModes};
use serde::{Deserialize, Serialize};
use serde_json::Value;

use std::error::Error;
use std::fs;
use std::fs::File;
use std::io::Write;

use std::path::Path;
use std::time::SystemTime;

#[derive(Debug, Serialize, Deserialize)]
pub struct DatabaseManager {
    motor1: u16,
    motor2: u16,
    motor3: u16,
    motor4:u16,
    rtc: u64,
    yaw: f32,
    pitch: f32,
    roll: f32,
    x: i16, 
    y: i16,
    z: i16, 
    bat: u16, 
    bar: u32,
    workingmode: WorkingModes,
}

impl DatabaseManager {
    ///Create the Json for it to be stored in the directory
    pub fn create_json(packet: &Packet)-> () {

        // Create folder to store jsons, if it does not exist yet
        let database_exists = Path::new("database").is_dir();
        
        if !database_exists {
            fs::create_dir("database").unwrap();
        }

        let test = packet;
        let msg = test.message;
        
        match msg {
            Message::Datalogging(datalog) => {
                let json = serde_json::to_string(&datalog).unwrap();
                let now = SystemTime::now().duration_since(SystemTime::UNIX_EPOCH).unwrap().as_nanos();

                let file_name = now.to_string();
                let mut file = File::create(format!("database/{}.json", file_name)).unwrap();
                file.write_all(json.as_bytes()).unwrap();
            }
            _ => ()
        }
        
    }
    
    fn read_json_files() -> Result<Vec<Value>, Box<dyn Error>>{
        let mut json_files: Vec<Value> = Vec::new();
        for entry in fs::read_dir("database/")? {
            let entry = entry?;
            let file_path = entry.path();
            if file_path.is_file() && file_path.extension().unwrap_or_default() == "json" {
                let json_string = fs::read_to_string(file_path)?;
                let json_value: Value = serde_json::from_str(&json_string)?;
                json_files.push(json_value);
            }
        }
        Ok(json_files)
    }
}

#[cfg(test)]
mod tests {
    use std::{time::{Duration}, thread::sleep};

    use protocol::Datalog;

    use super::*;

    #[test]
    fn test_store_json() {

        for i in 0..10 {
            let datalog = Datalog {motor1: i, motor2: 0, motor3: 0, motor4: 0, rtc: 9, yaw: 0.0, pitch: 0.0, roll: 0.0, x: 0, y: 0, z: 0, bat: 0, bar: 0, workingmode: WorkingModes::ManualMode, arguments: [0, 0, 0, 0], control_loop_time: 0, yaw_f: 0.0, pitch_f: 0.0, roll_f: 0.0, yaw_r: 0.0, pitch_r: 0.0, roll_r: 0.0  };
            let message = Message::Datalogging(datalog);
            let packet = Packet::new(message);
            
            DatabaseManager::create_json(&packet);

            sleep(Duration::from_millis(500));
        }
    }

    #[test]
    fn test_read_json() {
        let result = DatabaseManager::read_json_files().unwrap();
        println!("{:?}", result);
    }
}