slint::include_modules!();
use crossbeam::channel::Sender;
use network_initializer::{errors::ConfigError, NetworkInitializer};
use slint::{Model, ModelRc, VecModel, Weak};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::thread;
use wg_internal::controller::DroneCommand;
use wg_internal::packet::{NackType, Packet, PacketType};

use logger::{LogLevel, Logger};

#[derive(Debug)]
pub enum NodeType {
    Drone,
    Client,
    Server,
}

// send DroneCommand to drone
pub fn send_drone_command(
    senders: &Arc<Mutex<Option<HashMap<u8, Sender<DroneCommand>>>>>,
    id: u8,
    command: Box<DroneCommand>,
    logger: &Arc<Mutex<Logger>>,
) {
    if let Some(ref s) = *senders.lock().unwrap() {
        if let Some(sender) = s.get(&(id as u8)) {
            let res = sender.send(*command.clone());
            match res {
                Ok(_) => {
                    logger
                        .lock()
                        .unwrap()
                        .log_debug(&format!("{:?} succesfully sent to Node{}", *command, id));
                }
                Err(e) => {
                    logger.lock().unwrap().log_debug(&format!(
                        "Error sending DroneCommand to drone {}: {:?}",
                        id, e
                    ));
                }
            }
        } else {
            logger
                .lock()
                .unwrap()
                .log_debug(&format!("No sender for drone {}", id));
        }
    } else {
        logger.lock().unwrap().log_debug("No senders available");
    }
}

// given the id of a node, return the type of the node 0 if drone, 1 if client, 2 if server
pub fn get_node_type(
    id: i32,
    id_to_type: &Arc<Mutex<HashMap<i32, (NodeType, i32)>>>,
) -> (i32, i32) {
    let id_to_type_ = id_to_type.lock().unwrap();
    match id_to_type_.get(&id) {
        Some((NodeType::Drone, index)) => return (0, *index),
        Some((NodeType::Client, index)) => return (1, *index),
        Some((NodeType::Server, index)) => return (2, *index),
        None => return (-1, -1),
    }
}

// to initate the logger with the given log level
pub fn initiate_logger(level: LogLevel) -> Arc<Mutex<Logger>> {
    let logger: Arc<Mutex<Logger>> = Arc::new(Mutex::new(Logger::new(
        0,
        true,
        "SimulationController".to_string(),
    )));
    (*logger).lock().unwrap().add_displayable_flag(level);
    return logger;
}

pub fn run_simulation_thread(
    logger_: Arc<Mutex<Logger>>,
    network_initializer_run_simulation: Arc<Mutex<Result<NetworkInitializer, ConfigError>>>,
) -> std::thread::JoinHandle<()> {
    thread::spawn(move || {
        logger_.lock().unwrap().log_debug("Simulation started");
        if let Ok(ref mut c) = *network_initializer_run_simulation.lock().unwrap() {
            match c.run_simulation(None, None) {
                Ok(_) => {
                    logger_
                        .lock()
                        .unwrap()
                        .log_debug("Simulation ended correctly");
                }
                Err(e) => {
                    let error = format!("Simulation ended with error {}", e);
                    logger_.lock().unwrap().log_error(&error);
                }
            }
        }
    })
}
