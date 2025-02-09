use std::sync::{Arc, Mutex};
use std::collections::HashMap;
use wg_internal::controller::DroneCommand;
use crossbeam::channel::Sender;

#[derive(Debug)]
pub enum NodeType{
    Drone,
    Client,
    Server,
}

// send drone commands
pub fn send_drone_command(senders: &Arc<Mutex<Option<HashMap<u8, Sender<DroneCommand>>>>>, id:u8, command: Box<DroneCommand>)->Result<(), String>{
    if let Some(ref s) = *senders.lock().unwrap() {
        // println!("Senders map {:?}", s);
        if let Some(sender) = s.get(&(id as u8)) {
            let res = sender.send(*command);
            match res {
                Ok(_) => {
                    // println!("DroneCommand sent to drone {}", id);
                    return Ok(());
                }
                Err(e) => {
                    let error = format!("Error sending DroneCommand to drone {}: {:?}", id, e);
                    return Err(error);
                }
            }
        } else {
            let error = format!("No sender for drone {}", id);
            return Err(error);
        }
    } else {
        let error = format!("Empty senders map");
        return Err(error);
    }
}

pub fn get_node_type(id: i32, id_to_type: &Arc<Mutex<HashMap<i32, (NodeType, i32)>>>) -> (i32, i32) {
    let id_to_type_ = id_to_type.lock().unwrap();
    match id_to_type_.get(&id){
        Some((NodeType::Drone, index)) => return (0, *index),
        Some((NodeType::Client, index)) => return (1, *index),
        Some((NodeType::Server, index)) => return (2, *index),
        None => return (-1, -1),
    }
}