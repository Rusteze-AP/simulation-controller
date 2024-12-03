slint::include_modules!();
use rfd::FileDialog;
use slint::{Model, Window};
use std::{cell::{RefCell, RefMut}, ffi::OsString, rc::Rc, time::Duration};
use network_initializer::{errors::ConfigError, NetworkInitializer};
use crossbeam::channel::{unbounded, Receiver, Sender, Select};
use wg_internal::packet::Packet;
use wg_internal::controller::{DroneCommand, NodeEvent};
use std::thread;
use std::sync::{Arc, Mutex};
use std::collections::HashMap;
use wg_internal::network::NodeId;
use logger::Logger;
// use slint::Model;

fn check_edges(edges : &Vec<Edge>, id1: i32, id2: i32) -> bool {
    for edge in edges {
        if (edge.id1 == id1 && edge.id2 == id2) || (edge.id1 == id2 && edge.id2 == id1) {
            return true;
        }
    }
    return false;
}

fn run_config_simulation(config: Arc<Mutex<Result<NetworkInitializer, ConfigError>>>) {
    thread::spawn(move || {
        if let Ok(ref mut c) = *config.lock().unwrap(){
            println!("{:?}",c);
            c.run_simulation();
            println!("Simulation done");
        }
    });
}

fn main() -> Result<(), slint::PlatformError> {
    let logger = Logger::new(true,"SimulationController".to_string());


    let mut main_window = MainWindow::new()?;
    let weak = main_window.as_weak();
    let weak1 = main_window.as_weak();

    let mut general_receiver: Arc<Mutex<Option<Receiver<NodeEvent>>>> = Arc::new(Mutex::new(None));
    let mut general_receiver1 = general_receiver.clone();
    let mut general_receiver2 = general_receiver.clone();

    let mut senders: Arc<Mutex<Option<HashMap<NodeId,Sender<DroneCommand>>>>> = Arc::new(Mutex::new(None));
    let mut senders1 = senders.clone();

    let mut network_initializer:Arc<Mutex<Result<NetworkInitializer, ConfigError>>> = Arc::new(Mutex::new(NetworkInitializer::new(None)));
    let mut network_initializer1:Arc<Mutex<Result<NetworkInitializer, ConfigError>>> = network_initializer.clone();
    let mut network_initializer2:Arc<Mutex<Result<NetworkInitializer, ConfigError>>> = network_initializer.clone();


    // thread for checking if there is a configuration and run it                
    thread::spawn(move || {
        loop {
            thread::sleep(Duration::from_millis(5000));

            println!("[SIM-CONTROLLER] Selecting config");
            // let mut n1 = network_initializer2.clone();
            if let Ok(ref mut c) = *network_initializer2.lock().unwrap(){
                c.run_simulation();
            }
        }
    });


    // thread for message receiving from drones
    thread::spawn(move || {
        loop {
            thread::sleep(Duration::from_millis(5000));
            if let Some(ref mut rec_from_drones) = *general_receiver2.lock().unwrap() {
                let mut select = Select::new();
                let oper1 = select.recv(&rec_from_drones); // Blocks until a message is available
                let message = select.select();
                match message.recv(&rec_from_drones) {
                    Ok(NodeEvent::PacketDropped(packet)) => {
                        println!("[SIMULATION CONTROLLER] PacketDropped {:?}", packet);
                    },
                    Ok(NodeEvent::PacketSent(packet)) => {
                        println!("[SIMULATION CONTROLLER] PacketSent {:?}", packet);
                    },
                    Err(e) => {
                        println!("Error receiving message: {:?}", e);
                    },
                    _ => {
                        println!("Unknown message");
                    }
                }
            }else{
                println!("No receiver");
            }
        }
    });

    main_window.on_select_file(move || {
        logger.log_info("file selection");
        let file = FileDialog::new().pick_file();
        if let Some(mut path)= file {
            let path_string = <OsString as Clone>::clone(&path.as_mut_os_string()).into_string();
            if let Ok(path_string) = path_string {
                println!("Selected file: {}", path_string);

                *network_initializer1.lock().unwrap() = NetworkInitializer::new(Some(path_string.as_str()));
                let mut config = network_initializer1.lock().unwrap();
                

                if let Ok(ref mut c) = *config {
                    *general_receiver1.lock().unwrap() = Some(c.get_controller_recv());
                    println!("Rec directly from config {:?}", c.get_controller_recv());
                    *senders.lock().unwrap() = Some(c.get_controller_senders());
                    println!("Senders directly from config {:?}", c.get_controller_senders());
                    println!("Sender in loading {:?}", senders.lock().unwrap());

                    let from_network_initializer = c.get_nodes();
                
                    if let Some(window) = weak.upgrade() {
                        let mut edges : Vec<Edge> = vec![];


                        let mut clients : Vec<Drone> = vec![];
                        for drone in from_network_initializer.1 {
                            let mut adjent = vec![];
                            for adj in &drone.connected_drone_ids {
                                adjent.push(*adj as i32);
                                if !check_edges(&edges, drone.id as i32, *adj as i32) {
                                    edges.push(Edge{id1:drone.id as i32, id2:*adj as i32});
                                }
                            }
                            clients.push(Drone{adjent:slint::ModelRc::new(slint::VecModel::from(adjent)), crashed: false, id:drone.id as i32, pdr:0.0});
                        }
                        window.set_clients(slint::ModelRc::new(slint::VecModel::from(clients)));


                        let mut drones : Vec<Drone> = vec![];
                        for drone in from_network_initializer.0 {
                            let mut adjent = vec![];
                            for adj in &drone.connected_drone_ids {
                                adjent.push(*adj as i32);
                                if !check_edges(&edges, drone.id as i32, *adj as i32) {
                                    edges.push(Edge{id1:drone.id as i32, id2:*adj as i32});
                                }
                            }
                            drones.push(Drone{adjent:slint::ModelRc::new(slint::VecModel::from(adjent)), crashed: false, id:drone.id as i32, pdr:drone.pdr});
                        }
                        window.set_drones(slint::ModelRc::new(slint::VecModel::from(drones)));

                        let mut servers : Vec<Drone> = vec![];
                        for drone in from_network_initializer.2 {
                            let mut adjent = vec![];
                            for adj in &drone.connected_drone_ids {
                                adjent.push(*adj as i32);
                                if !check_edges(&edges, drone.id as i32, *adj as i32) {
                                    edges.push(Edge{id1:drone.id as i32, id2:*adj as i32});
                                }
                            }
                            servers.push(Drone{adjent:slint::ModelRc::new(slint::VecModel::from(adjent)), crashed: false, id:drone.id as i32, pdr:0.0});
                        }
                        window.set_servers(slint::ModelRc::new(slint::VecModel::from(servers)));

                        window.set_edges(slint::ModelRc::new(slint::VecModel::from(edges)));
                    }
                }

                
                
                
        
            }else{
                println!("Error converting path to string");
            }
        }else{
            println!("No file selected\nTry again");
        }
    });



    main_window.on_remove_edges(move || {
        
        if let Some(window) = weak1.upgrade() {

            let id = window.get_id_selected_drone();
            if let Some(ref mut s) = *senders1.lock().unwrap(){
                println!("Senders map {:?}",s);
                if let Some(sender) = s.get(&(id as u8)){
                    let res = sender.send(DroneCommand::Crash);
                    match res {
                        Ok(_) => {
                            println!("Crash command sent to drone {}", id);
                        },
                        Err(e) => {
                            println!("Error sending crash command to drone {}: {:?}", id, e);
                        }
                    }
                }else {
                    println!("No sender for drone {}", id);
                }
            }else{
                println!("No senders map loaded");
            }





            println!("[SIMULATION CONTROLLER] REMOVE EDGES");
            let mut edges = window.get_edges();
            let mut edges_new : Vec<Edge> = vec![];
            for edge in edges.iter(){
                if edge.id1 == window.get_id_selected_drone() || edge.id2 ==  window.get_id_selected_drone() {
                    println!("removed {} {}", edge.id1, edge.id2);
                }else{
                    edges_new.push(Edge{id1:edge.id1, id2:edge.id2});
                }
            }
            window.set_edges(slint::ModelRc::new(slint::VecModel::from(edges_new)));
        }
    });



    main_window.run();

    // println!("Set config to :{:?} ", general_receiver);
    Ok(())
}
