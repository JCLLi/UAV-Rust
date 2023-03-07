use protocol::{self, Packet, Message, PacketError, PacketManager};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use serde_json::json;
use std::error::Error;
use std::fs;
use std::fs::File;
use std::io::Write;
use std::io::BufReader;

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
    workingmode: u8,
}

impl DatabaseManager {
    ///Create the Json for it to be stored in the directory
    /// -> Result<(), DatabaseError>{
    pub fn create_json(packet: &Packet)-> () {
        let test = packet;
        let msg = test.message;
        
        match msg {
            Message::Datalogging(motor1,motor2 ,motor3 ,motor4 ,delay ,ypr_yaw ,ypr_pitch ,ypr_roll ,acc_x ,acc_y ,acc_z ,bat ,bar ) => {
                print!("test");
                let drone_stats = DatabaseManager {
                    motor1: motor1,
                    motor2: motor2,
                    motor3: motor3,
                    motor4: motor4,
                    rtc: delay,
                    yaw: ypr_yaw,
                    pitch: ypr_pitch,
                    roll: ypr_roll,
                    x: acc_x,
                    y: acc_y,
                    z: acc_z,
                    bat: bat,
                    bar: bar,
                    workingmode: 0
                };
                let json = serde_json::to_string(&drone_stats).unwrap();
                let file_name = drone_stats.rtc.to_string();
                let mut file = File::create(format!("database/{}.json", file_name)).unwrap();
                file.write_all(json.as_bytes()).unwrap();
            }
            _ => ()
        }
        
    }
    
    pub fn read_json()->Vec<DatabaseManager>{ //think about what you are going to return
            
        let file = File::open("database/data.json").expect("Failed to open file");
        let reader = BufReader::new(file);

        let json_log: Vec<DatabaseManager> = serde_json::from_reader(reader).expect("Failed to parse JSON");
        return json_log;  
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
    use super::*;

    #[test]
    fn test_store_json() {

        for i in 0..10 {
            let message = Message::Datalogging(0, 0, 0, 0, i, 0.0, 0.0, 0.0 ,0, 0, 0, 0, 0);
            let packet = Packet::new(message);

            DatabaseManager::create_json(&packet);
        }
    }

    #[test]
    fn test_read_json() {
        let result = DatabaseManager::read_json_files().unwrap();
        println!("{:?}", result);
    }
}