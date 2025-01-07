slint::include_modules!();
use crossbeam::channel::{ Receiver, Sender};
use network_initializer::{errors::ConfigError, NetworkInitializer, parsed_nodes::{ParsedDrone, ParsedClient, ParsedServer}};
use slint::{Model, ModelRc, VecModel};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::thread;
use wg_internal::controller::{DroneCommand, DroneEvent};
use wg_internal::network::NodeId;
use wg_internal::packet::Packet;
use network_initializer::types::channel::Channel;


fn check_edges(edges: &Vec<Edge>, id1: i32, id2: i32) -> bool {
    for edge in edges {
        if (edge.id1 == id1 && edge.id2 == id2) || (edge.id1 == id2 && edge.id2 == id1) {
            return true;
        }
    }
    return false;
}

fn populate_drones(parsed_drones: &Vec<ParsedDrone>, edges: &mut Vec<Edge>) -> Vec<Drone> {
    let mut drones: Vec<Drone> = vec![];
    for drone in parsed_drones {
        let mut adjent = vec![];
        for adj in &drone.connected_drone_ids {
            adjent.push(*adj as i32);
            if !check_edges(&edges, drone.id as i32, *adj as i32) {
                edges.push(Edge {
                    id1: drone.id as i32,
                    id2: *adj as i32,
                });
            }
        }

        let mut not_adj = vec![];
        for d in parsed_drones {
            if (!adjent.contains(&(d.id as i32))) && d.id != drone.id {
                not_adj.push(d.id as i32);
            }
        }
        drones.push(Drone {
            adjent: slint::ModelRc::new(slint::VecModel::from(adjent)),
            not_adjacent: slint::ModelRc::new(slint::VecModel::from(not_adj)),
            id: drone.id as i32,
            pdr: drone.pdr,
        });
    }
    return drones;
}

fn populate_clients(parsed_client: &Vec<ParsedClient>, edges: &mut Vec<Edge>, parsed_drones: &Vec<ParsedDrone>)-> Vec<ClientServer>{
    let mut clients: Vec<ClientServer> = vec![];
    for client in parsed_client{
        let mut adjent = vec![];
        for adj in &client.connected_drone_ids {
            adjent.push(*adj as i32);
            if !check_edges(&edges, client.id as i32, *adj as i32) {
                edges.push(Edge {
                    id1: client.id as i32,
                    id2: *adj as i32,
                });
            }
        }

        let mut not_adj = vec![];
        for d in parsed_drones {
            if !adjent.contains(&(d.id as i32)) && d.id != client.id {
                not_adj.push(d.id as i32);
            }
        }

        clients.push(ClientServer {
            drones_adjacent: slint::ModelRc::new(slint::VecModel::from(adjent)),
            drones_not_adjacent: slint::ModelRc::new(slint::VecModel::from(not_adj)),
            id: client.id as i32,
        });
    }
    return clients;
}


fn populate_servers(parsed_server: &Vec<ParsedServer>, edges: &mut Vec<Edge>, parsed_drones: &Vec<ParsedDrone>)-> Vec<ClientServer>{
    let mut servers: Vec<ClientServer> = vec![];
    for server in parsed_server{
        let mut adjent = vec![];
        for adj in &server.connected_drone_ids {
            adjent.push(*adj as i32);
            if !check_edges(&edges, server.id as i32, *adj as i32) {
                edges.push(Edge {
                    id1: server.id as i32,
                    id2: *adj as i32,
                });
            }
        }

        let mut not_adj = vec![];
        for d in parsed_drones {
            if !adjent.contains(&(d.id as i32)) && d.id != server.id {
                not_adj.push(d.id as i32);
            }
        }

        // println!("server_adj: {:?}", adjent);
        // println!("server_adj: {:?}", not_adj);


        servers.push(ClientServer {
            drones_adjacent: slint::ModelRc::new(slint::VecModel::from(adjent)),
            drones_not_adjacent: slint::ModelRc::new(slint::VecModel::from(not_adj)),
            id: server.id as i32,
        });
    }
    return servers;
}


fn main() -> Result<(), slint::PlatformError> {
    // let logger = Logger::new(0, true, "SimulationController".to_string());
    let main_window = MainWindow::new()?;

    //initial configuration -> default
    let network_initializer: Arc<Mutex<Result<NetworkInitializer, ConfigError>>> = Arc::new(Mutex::new(NetworkInitializer::new(Some("test1.toml"))));
    let mut sc_receiver: Arc<Mutex<Option<Receiver<DroneEvent>>>> = Arc::new(Mutex::new(None));
    let mut sc_senders: Arc<Mutex<Option<HashMap<NodeId, Sender<DroneCommand>>>>> = Arc::new(Mutex::new(None));
    let mut channels: Arc<Mutex<Option<HashMap<NodeId, Channel<Packet>>>>> = Arc::new(Mutex::new(None));


    if let Ok(ref mut c)= *network_initializer.lock().unwrap() {
        sc_receiver = Arc::new(Mutex::new(Some((*c).get_controller_recv())));
        sc_senders = Arc::new(Mutex::new(Some((*c).get_controller_senders())));
        channels = Arc::new(Mutex::new(Some((*c).get_channels())));

        let nodes = c.get_nodes();

        let mut edges: Vec<Edge> = vec![];
        let clients = populate_clients(&nodes.1, &mut edges, &nodes.0);
        let drones = populate_drones(&nodes.0, &mut edges);
        let servers = populate_servers(&nodes.2, &mut edges, &nodes.0);

        println!("setting instances");
        let weak = main_window.as_weak();
        if let Some(window) = weak.upgrade() {
            window.set_edges(slint::ModelRc::new(slint::VecModel::from(edges)));
            window.set_clients(slint::ModelRc::new(slint::VecModel::from(clients)));
            window.set_drones(slint::ModelRc::new(slint::VecModel::from(drones)));
            window.set_servers(slint::ModelRc::new(slint::VecModel::from(servers)));
        }
    }

    // thread for running the simulation
    let network_initializer_run_simulation = network_initializer.clone();
    thread::spawn(move || {
        println!("Starting simulation");
        if let Ok(ref mut c) = *network_initializer_run_simulation.lock().unwrap() {
            match c.run_simulation(){
                Ok(_) => {
                    println!("[SIMULATION CONTROLLER] Simulation started");
                }
                Err(e) => {
                    println!("[SIMULATION CONTROLLER] Error starting simulation: {:?}", e);
                }
            }
        }
    });

    // thread for receiving DroneEvent
    let weak = main_window.as_weak();
    thread::spawn(move ||{
        loop{
            if let Some(sc_rec) = sc_receiver.lock().unwrap().as_ref(){
                    match sc_rec.recv(){
                        // PacketDropped
                        Ok(DroneEvent::PacketDropped(packet)) => {
                            println!("[SIMULATION CONTROLLER] PacketDropped {:?}", packet);
                            match weak.upgrade_in_event_loop(move |window| 
                                {let messages : ModelRc<Message> = window.get_messages();
                                if let Some(vec_model) = messages.as_any().downcast_ref::<VecModel<Message>>() {
                                    vec_model.push(Message{id1: packet.routing_header.hops[packet.routing_header.hop_index-1] as i32 , id2: packet.routing_header.hops[packet.routing_header.hop_index] as i32, msg_type:1});
                                }else{
                                    window.set_messages(slint::ModelRc::new(slint::VecModel::from(vec![
                                        Message { id1: packet.routing_header.hops[packet.routing_header.hop_index-1] as i32 , id2: packet.routing_header.hops[packet.routing_header.hop_index] as i32, msg_type:1},
                                    ])));
                                }
                                }
                            ){
                                Ok(_) => {
                                    println!("[SIMULATION CONTROLLER] Message sent to window");
                                },
                                Err(e) => {
                                    println!("[SIMULATION CONTROLLER] Error sending message to window: {:?}", e);
                                }
                            }
                        }
                        // PacketSent
                        Ok(DroneEvent::PacketSent(packet)) => {
                            println!("[SIMULATION CONTROLLER] PacketSent {:?}", packet);
                            match weak.upgrade_in_event_loop(move |window| 
                                {let messages : ModelRc<Message> = window.get_messages();
                                    if packet.routing_header.hops.len() > 1{
                                        if let Some(vec_model) = messages.as_any().downcast_ref::<VecModel<Message>>() {
                                            vec_model.push(Message{id1: packet.routing_header.hops[packet.routing_header.hop_index] as i32 , id2: packet.routing_header.hops[packet.routing_header.hop_index+1] as i32, msg_type:0});
                                        }else{
                                            window.set_messages(slint::ModelRc::new(slint::VecModel::from(vec![
                                                Message { id1: packet.routing_header.hops[packet.routing_header.hop_index] as i32 , id2: packet.routing_header.hops[packet.routing_header.hop_index+1] as i32, msg_type:0},
                                            ])));
                                        }
                                    }
                                }
                            ){
                                Ok(_) => {
                                    println!("[SIMULATION CONTROLLER] Message sent to window");
                                },
                                Err(e) => {
                                    println!("[SIMULATION CONTROLLER] Error sending message to window: {:?}", e);
                                }
                            }
                        }
                        // ControllerShortcut
                        Ok(DroneEvent::ControllerShortcut(packet)) => {
                            println!("[SIMULATION CONTROLLER] ControllerShortcut {:?}", packet);
                            match weak.upgrade_in_event_loop(move |window| 
                                {let messages : ModelRc<Message> = window.get_messages();
                                if let Some(vec_model) = messages.as_any().downcast_ref::<VecModel<Message>>() {
                                    vec_model.push(Message{id1: packet.routing_header.hops[packet.routing_header.hop_index-1] as i32 , id2: packet.routing_header.hops[packet.routing_header.hop_index] as i32, msg_type:2});
                                }else{
                                    window.set_messages(slint::ModelRc::new(slint::VecModel::from(vec![
                                        Message { id1: packet.routing_header.hops[packet.routing_header.hop_index-1] as i32 , id2: packet.routing_header.hops[packet.routing_header.hop_index] as i32, msg_type:2},
                                    ])));
                                }
                                }
                            ){
                                Ok(_) => {
                                    println!("[SIMULATION CONTROLLER] Message sent to window");
                                },
                                Err(e) => {
                                    println!("[SIMULATION CONTROLLER] Error sending message to window: {:?}", e);
                                }
                            }
                        }
                        Err(e) => {
                            println!("[SIMULATION CONTROLLER] Error receiving message: {:?}", e);
                        }
                    }
                }
        }
    });

    main_window.on_select_file(move || {
        // logger.log_info("file selection");
        // let file = FileDialog::new().pick_file();
        // if let Some(mut path) = file {
        //     let path_string = <OsString as Clone>::clone(&path.as_mut_os_string()).into_string();
        //     if let Ok(path_string) = path_string {
        //         println!("Selected file: {}", path_string);

        //         // before initilizing check kill all the nodes sending a crash command
        //         if let Some(window) = weak.upgrade() {
        //             let drones = window.get_drones();
        //             let clients = window.get_clients();
        //             let servers = window.get_servers();

        //             for drone in drones.iter() {
        //                 if let Some(ref mut s) = *senders2.lock().unwrap() {
        //                     if let Some(sender) = s.get(&(drone.id as u8)) {
        //                         if drone.crashed {
        //                             continue;
        //                         }
        //                         let res = sender.send(DroneCommand::Crash);
        //                         match res {
        //                             Ok(_) => {
        //                                 println!("Crash command sent to drone {}", drone.id);
        //                             }
        //                             Err(e) => {
        //                                 println!(
        //                                     "Error sending crash command to drone {}: {:?}",
        //                                     drone.id, e
        //                                 );
        //                             }
        //                         }
        //                     } else {
        //                         println!("No sender for drone {}", drone.id);
        //                     }
        //                 } else {
        //                     println!("No senders map loaded");
        //                 }
        //             }

        //             for client in clients.iter() {
        //                 if let Some(ref mut s) = *senders2.lock().unwrap() {
        //                     if let Some(sender) = s.get(&(client.id as u8)) {
        //                         let res = sender.send(DroneCommand::Crash);
        //                         match res {
        //                             Ok(_) => {
        //                                 println!("Crash command sent to drone {}", client.id);
        //                             }
        //                             Err(e) => {
        //                                 println!(
        //                                     "Error sending crash command to drone {}: {:?}",
        //                                     client.id, e
        //                                 );
        //                             }
        //                         }
        //                     } else {
        //                         println!("No sender for drone {}", client.id);
        //                     }
        //                 } else {
        //                     println!("No senders map loaded");
        //                 }
        //             }

        //             for server in servers.iter() {
        //                 if let Some(ref mut s) = *senders2.lock().unwrap() {
        //                     if let Some(sender) = s.get(&(server.id as u8)) {
        //                         let res = sender.send(DroneCommand::Crash);
        //                         match res {
        //                             Ok(_) => {
        //                                 println!("Crash command sent to drone {}", server.id);
        //                             }
        //                             Err(e) => {
        //                                 println!(
        //                                     "Error sending crash command to drone {}: {:?}",
        //                                     server.id, e
        //                                 );
        //                             }
        //                         }
        //                     } else {
        //                         println!("No sender for drone {}", server.id);
        //                     }
        //                 } else {
        //                     println!("No senders map loaded");
        //                 }
        //             }

        //             window.set_drones(slint::ModelRc::new(slint::VecModel::from(vec![])));
        //             window.set_clients(slint::ModelRc::new(slint::VecModel::from(vec![])));
        //             window.set_servers(slint::ModelRc::new(slint::VecModel::from(vec![])));
        //             window.set_edges(slint::ModelRc::new(slint::VecModel::from(vec![])));

        //             // reset configuration and decomment line
        //             let lock_result = network_initializer1.lock();
        //             if let Ok(mut guard) = lock_result {
        //                 if let Ok(network) = &mut *guard {
        //                     // network.stop_simulation();
        //                     println!("[SIMULATION-CONTROLLER]Error: Failed to acquire lock");
        //                 } else {
        //                     println!("[SIMULATION-CONTROLLER]Error: Failed to acquire lock");
        //                 }
        //             }

        //             // enforce new configuration
        //             *network_initializer1.lock().unwrap() =
        //                 NetworkInitializer::new(Some(path_string.as_str()));
        //             let mut config = network_initializer1.lock().unwrap();

        //             if let Ok(ref mut c) = *config {
        //                 *general_receiver1.lock().unwrap() = Some(c.get_controller_recv());
        //                 println!("Rec directly from config {:?}", c.get_controller_recv());
        //                 *senders.lock().unwrap() = Some(c.get_controller_senders());
        //                 println!(
        //                     "Senders directly from config {:?}",
        //                     c.get_controller_senders()
        //                 );
        //                 println!("Sender in loading {:?}", senders.lock().unwrap());

        //                 *channels.lock().unwrap() = Some(c.get_channels());

        //                 let from_network_initializer = c.get_nodes();

        //                 let mut edges: Vec<Edge> = vec![];

        //                 let mut clients: Vec<Drone> = vec![];
        //                 for drone in from_network_initializer.1 {
        //                     let mut adjent = vec![];
        //                     for adj in &drone.connected_drone_ids {
        //                         adjent.push(*adj as i32);
        //                         if !check_edges(&edges, drone.id as i32, *adj as i32) {
        //                             edges.push(Edge {
        //                                 id1: drone.id as i32,
        //                                 id2: *adj as i32,
        //                             });
        //                         }
        //                     }

        //                     let mut not_adj = vec![];
        //                     for d in from_network_initializer.0 {
        //                         if !adjent.contains(&(d.id as i32)) && d.id != drone.id {
        //                             not_adj.push(d.id as i32);
        //                         }
        //                     }

        //                     clients.push(Drone {
        //                         adjent: slint::ModelRc::new(slint::VecModel::from(adjent)),
        //                         not_adjacent: slint::ModelRc::new(slint::VecModel::from(not_adj)),
        //                         crashed: false,
        //                         id: drone.id as i32,
        //                         pdr: 0.0,
        //                     });
        //                 }

        //                 let mut drones: Vec<Drone> = vec![];
        //                 for drone in from_network_initializer.0 {
        //                     let mut adjent = vec![];
        //                     for adj in &drone.connected_drone_ids {
        //                         adjent.push(*adj as i32);
        //                         if !check_edges(&edges, drone.id as i32, *adj as i32) {
        //                             edges.push(Edge {
        //                                 id1: drone.id as i32,
        //                                 id2: *adj as i32,
        //                             });
        //                         }
        //                     }

        //                     let mut not_adj = vec![];
        //                     for d in from_network_initializer.0 {
        //                         if !adjent.contains(&(d.id as i32)) && d.id != drone.id {
        //                             not_adj.push(d.id as i32);
        //                         }
        //                     }
        //                     // println!("HERE ADJ: {:?}", adjent);
        //                     // println!("HERE NOT_ADJ: {:?}", not_adj);
        //                     drones.push(Drone {
        //                         adjent: slint::ModelRc::new(slint::VecModel::from(adjent)),
        //                         not_adjacent: slint::ModelRc::new(slint::VecModel::from(not_adj)),
        //                         crashed: false,
        //                         id: drone.id as i32,
        //                         pdr: drone.pdr,
        //                     });
        //                 }

        //                 let mut servers: Vec<Drone> = vec![];
        //                 for drone in from_network_initializer.2 {
        //                     let mut adjent = vec![];
        //                     for adj in &drone.connected_drone_ids {
        //                         adjent.push(*adj as i32);
        //                         if !check_edges(&edges, drone.id as i32, *adj as i32) {
        //                             edges.push(Edge {
        //                                 id1: drone.id as i32,
        //                                 id2: *adj as i32,
        //                             });
        //                         }
        //                     }

        //                     let mut not_adj = vec![];
        //                     for d in from_network_initializer.0 {
        //                         if !adjent.contains(&(d.id as i32)) && d.id != drone.id {
        //                             not_adj.push(d.id as i32);
        //                         }
        //                     }

        //                     // TODO : add non adj
        //                     servers.push(Drone {
        //                         adjent: slint::ModelRc::new(slint::VecModel::from(adjent)),
        //                         not_adjacent: slint::ModelRc::new(slint::VecModel::from(not_adj)),
        //                         crashed: false,
        //                         id: drone.id as i32,
        //                         pdr: 0.0,
        //                     });
        //                 }

        //                 window.set_edges(slint::ModelRc::new(slint::VecModel::from(edges)));

        //                 window.set_clients(slint::ModelRc::new(slint::VecModel::from(clients)));
        //                 window.set_drones(slint::ModelRc::new(slint::VecModel::from(drones)));
        //                 window.set_servers(slint::ModelRc::new(slint::VecModel::from(servers)));
        //                 window.set_messages(slint::ModelRc::new(slint::VecModel::from(vec![
        //                     Message { id1: 0, id2: 2 , msg_type: 0},
        //                     Message { id1: 20, id2: 1 , msg_type: 1},
        //                 ])));
                        
        //             }
        //         }
        //     } else {
        //         println!("Error converting path to string");
        //     }
        // } else {
        //     println!("No file selected\nTry again");
        // }
    });

    let weak = main_window.as_weak();
    let senders = sc_senders.clone();
    let channels_ = channels.clone();
    main_window.on_crash(move || {
        if let Some(window) = weak.upgrade() {
            let id = window.get_id_selected_drone();

            // SEND COMMAND TO DRONE
            if let Some(ref mut s) = *senders.lock().unwrap() {
                println!("Senders map {:?}", s);
                if let Some(sender) = s.get(&(id as u8)) {
                    let res = sender.send(DroneCommand::Crash);
                    match res {
                        Ok(_) => {
                            println!("Crash command sent to drone {}", id);
                        }
                        Err(e) => {
                            println!("Error sending crash command to drone {}: {:?}", id, e);
                        }
                    }
                } else {
                    println!("No sender for drone {}", id);
                }
            } else {
                println!("No senders map loaded");
            }

            // REMOVE ALL EDGES communicating with it
            let edges = window.get_edges();
            let mut i: usize= 0;
            let mut to_remove: Vec<usize> = vec![];
            for edge in edges.iter() {
                if edge.id1 == id || edge.id2 == id{
                    to_remove.push(i);
                }
                i = i+1;
            }

            for index in to_remove.iter().rev(){
                if let Some(vec_model) = edges.as_any().downcast_ref::<VecModel<Edge>>() {
                    vec_model.remove(*index);
                }else{
                    println!("problems in downcasting edges");
                }
            }

            // REMOVE DRONE from DRONES
            let drones = window.get_drones();
            let mut i = 0;
            for drone in drones.iter() {
                if drone.id == id {
                    if let Some(vec_model) = drones.as_any().downcast_ref::<VecModel<Drone>>() {
                        vec_model.remove(i);
                    }else{
                        println!("problems in downcasting drones");
                    }
                    break;
                }
                i = i+1;
            }

            // REMOVE ALL ADJACENT of others
            for drone in drones.iter() {
                let mut i = 0;
                for adj in drone.adjent.iter() {
                    if adj == id {
                        if let Some(vec_model) = drone.adjent.as_any().downcast_ref::<VecModel<i32>>() {
                            vec_model.remove(i);
                        }else{
                            println!("problems in downcasting adjacent");
                        }
                        break;
                    }
                    i = i+1;
                }
                i = 0;
                for not_adj in drone.not_adjacent.iter() {
                    if not_adj == id {
                        if let Some(vec_model) = drone.not_adjacent.as_any().downcast_ref::<VecModel<i32>>() {
                            vec_model.remove(i);
                        }else{
                            println!("problems in downcasting adjacent");
                        }
                        break;
                    }
                    i = i+1;
                }
            }

            // DROP IT'S CHANNELS from channels and senders
            if let Some(ref mut channel) = *channels_.lock().unwrap() {
                channel.remove(&(id as u8));
            } else {
                println!("No channels map loaded");
            }
            if let Some(ref mut s) = *senders.lock().unwrap() {
                s.remove(&(id as u8));
            } else {
                println!("No senders map loaded");
            }
            

        }
    });

    // Note: the channels are not removed becuase the connection cannot be deleted if it is the last of the drone
    let weak = main_window.as_weak();
    let senders = sc_senders.clone();
    main_window.on_remove_edge(move || {
        println!("[SIMULATION CONTROLLER] REMOVE EDGE");
        if let Some(window) = weak.upgrade() {
            let id_1 = window.get_sender_id();
            let id_2 = window.get_receiver_id();


            // COMMUNICATE REMOTION TO DRONES to id_1
            if let Some(ref mut s) = *senders.lock().unwrap() {
                println!("Senders map {:?}", s);
                if let Some(sender) = s.get(&(id_1 as u8)) {
                    let res = sender.send(DroneCommand::RemoveSender(id_2 as u8));
                    match res {
                        Ok(_) => {
                            println!("RemoveSender command sent to drone {} to remove {}", id_1, id_2);
                        }
                        Err(e) => {
                            println!("Error sending RemoveSender command to drone {}: {:?}", id_1, e);
                        }
                    }
                } else {
                    println!("No sender for drone {}", id_1);
                }
            } else {
                println!("No senders map loaded");
            }

            // COMMUNICATE REMOTION TO DRONES to id_2
            if let Some(ref mut s) = *senders.lock().unwrap() {
                println!("Senders map {:?}", s);
                if let Some(sender) = s.get(&(id_2 as u8)) {
                    let res = sender.send(DroneCommand::RemoveSender(id_1 as u8));
                    match res {
                        Ok(_) => {
                            println!("RemoveSender command sent to drone {} to remove {}", id_2, id_1);
                        }
                        Err(e) => {
                            println!("Error sending RemoveSender command to drone {}: {:?}", id_2, e);
                        }
                    }

                } else {
                    println!("No sender for drone {}", id_2);
                }
            } else {
                println!("No senders map loaded");
            }

            // REMOVE EDGE
            let edges = window.get_edges();
            let mut i = 0;
            for edge in edges.iter() {
                if (edge.id1 == id_1 && edge.id2 == id_2) || (edge.id1 == id_2 && edge.id2 == id_1){
                   if let Some(vec_model) = edges.as_any().downcast_ref::<VecModel<Edge>>() {
                        vec_model.remove(i);
                    }else{
                        println!("problems in downcasting edges");
                    }
                    break;
                }
                i = i+1;
            }


            // REMOVE ADJACENT
            let drones = window.get_drones();
            for drone in drones.iter() {
                if drone.id == id_1 {
                    let mut i = 0;
                    for adj in drone.adjent.iter() {
                        if adj == id_2 {
                            if let Some(vec_model) = drone.adjent.as_any().downcast_ref::<VecModel<i32>>() {
                                vec_model.remove(i);
                            }else{
                                println!("problems in downcasting adjacent");
                            }
                            if let Some(vec_model) = drone.not_adjacent.as_any().downcast_ref::<VecModel<i32>>() {
                                vec_model.push(id_2);
                            }else{
                                println!("problems in downcasting not_adjacent");
                            }
                            break;

                        }
                        i = i+1;
                    }
                } else if drone.id == id_2 {
                    let mut i = 0;
                    for adj in drone.adjent.iter() {
                        if adj == id_1 {
                            if let Some(vec_model) = drone.adjent.as_any().downcast_ref::<VecModel<i32>>() {
                                vec_model.remove(i);
                            }else{
                                println!("problems in downcasting adjacent");
                            }
                            if let Some(vec_model) = drone.not_adjacent.as_any().downcast_ref::<VecModel<i32>>() {
                                vec_model.push(id_1);
                            }else{
                                println!("problems in downcasting not_adjacent");
                            }
                            break;

                        }
                        i = i+1;
                    }
                }
            }
        }
    });

    // Note: the channels are not removed becuase the connection cannot be deleted if it is the last of the drone
    let weak = main_window.as_weak();
    let senders = sc_senders.clone();
    main_window.on_remove_edge_client_server(move || {
        println!("[SIMULATION CONTROLLER] REMOVE EDGE CLIENT SERVER");
        if let Some(window) = weak.upgrade() {
            let id_1 = window.get_sender_id();
            let id_2 = window.get_receiver_id();


            // COMMUNICATE REMOTION TO DRONES to id_1
            if let Some(ref mut s) = *senders.lock().unwrap() {
                println!("Senders map {:?}", s);
                if let Some(sender) = s.get(&(id_1 as u8)) {
                    let res = sender.send(DroneCommand::RemoveSender(id_2 as u8));
                    match res {
                        Ok(_) => {
                            println!("RemoveSender command sent to drone {} to remove {}", id_1, id_2);
                        }
                        Err(e) => {
                            println!("Error sending RemoveSender command to drone {}: {:?}", id_1, e);
                        }
                    }
                } else {
                    println!("No sender for drone {}", id_1);
                }
            } else {
                println!("No senders map loaded");
            }

            // COMMUNICATE REMOTION TO DRONES to id_2
            if let Some(ref mut s) = *senders.lock().unwrap() {
                println!("Senders map {:?}", s);
                if let Some(sender) = s.get(&(id_2 as u8)) {
                    let res = sender.send(DroneCommand::RemoveSender(id_1 as u8));
                    match res {
                        Ok(_) => {
                            println!("RemoveSender command sent to drone {} to remove {}", id_2, id_1);
                        }
                        Err(e) => {
                            println!("Error sending RemoveSender command to drone {}: {:?}", id_2, e);
                        }
                    }

                } else {
                    println!("No sender for drone {}", id_2);
                }
            } else {
                println!("No senders map loaded");
            }

            // REMOVE EDGE
            let edges = window.get_edges();
            let mut i = 0;
            for edge in edges.iter() {
                if (edge.id1 == id_1 && edge.id2 == id_2) || (edge.id1 == id_2 && edge.id2 == id_1){
                   if let Some(vec_model) = edges.as_any().downcast_ref::<VecModel<Edge>>() {
                        vec_model.remove(i);
                    }else{
                        println!("problems in downcasting edges");
                    }
                    break;
                }
                i = i+1;
            }


            // REMOVE ADJACENT FROM CLIENT/SERVER
            let tmp1 : i32; // lower -> drone
            let tmp2 : i32; // higher -> client or server 
            if id_1<id_2{
                tmp1 = id_1;
                tmp2 = id_2;
            }else{
                tmp1 = id_2;
                tmp2 = id_1;
            }
            if tmp2<30{ //->cliet
                let clients = window.get_clients();
                for c in clients.iter(){
                    if c.id == tmp2{
                        if let Some(not_adj) = c.drones_not_adjacent.as_any().downcast_ref::<VecModel<i32>>() {
                            not_adj.push(tmp1);
                        }
                        if let Some(adj) = c.drones_adjacent.as_any().downcast_ref::<VecModel<i32>>() {
                            let mut i = 0;
                            for e in adj.iter(){
                                if e==tmp1{
                                    adj.remove(i);
                                    break;
                                }
                                i = i+1;
                            }
                        }
                    }
                }
            }else{ //->server
                let servers = window.get_servers();
                for s in servers.iter(){
                    if s.id == tmp2{
                        if let Some(n_adj) = s.drones_not_adjacent.as_any().downcast_ref::<VecModel<i32>>() {
                            n_adj.push(tmp1);
                        }
                        if let Some(adj) = s.drones_adjacent.as_any().downcast_ref::<VecModel<i32>>() {
                            let mut i = 0;
                            for e in adj.iter(){
                                if e==tmp1{
                                    adj.remove(i);
                                    break;
                                }
                                i = i+1;
                            }
                        }
                    }
                }
            }
        

            // TODO : - sistemare rimozione e aggiunta clients e server con le altre strutture -> capire se fare altra callback separata per evitare scazzi
            //        - sisitemare struttura dati drone e aggiungere bottoni per connessioni client server ( o forsee no?)
        }
    });

    // Note: the channels are not created becuase the connection the drones already exists, they are only cloned
    let weak = main_window.as_weak();
    let senders = sc_senders.clone();
    let channels_ = channels.clone();
    main_window.on_add_edge(move || {
        println!("[SIMULATION CONTROLLER ] ADD EDGE");
        if let Some(window) = weak.upgrade() {
            let id_1 = window.get_sender_id();
            let id_2 = window.get_receiver_id();

            let mut sender_id_1: Option<Sender<Packet>> = None;
            let mut sender_id_2: Option<Sender<Packet>> = None;

            if let Some(ref mut channel) = *channels_.lock().unwrap() {

                if let Some(ch_1) = (*channel).get(&(id_1 as u8)) {
                    sender_id_1 = Some(ch_1.sender.clone());
                } else {
                    println!("No sender for id1");
                }

                if let Some(ch_2) = channel.get(&(id_2 as u8)) {
                    sender_id_2 = Some(ch_2.sender.clone());
                } else {
                    println!("No sender for id2");
                }
            } else {
                println!("No senders map loaded");
            }


            // COMMUNICATE ADDITION TO DRONES to id_1
            if let Some(ref mut s) = *senders.lock().unwrap() {
                println!("Senders map {:?}", s);
                if let Some(sender) = s.get(&(id_1 as u8)) {
                    if let Some(s_id_2)= sender_id_2{
                    let res = sender.send(DroneCommand::AddSender(id_2 as u8, s_id_2)); 
                    match res {
                        Ok(_) => {
                            println!("AddSender command sent to drone {} to add {}", id_1, id_2);
                        }
                        Err(e) => {
                            println!("Error sending AddSender command to drone {}: {:?}", id_1, e);
                        }
                    }
                } else {
                    println!("No sender for drone {}", id_1);
                }
            }
            } else {
                println!("No senders map loaded");
            }

            // COMMUNICATE ADDITION TO DRONES to id_2
            if let Some(ref mut s) = *senders.lock().unwrap() {
                println!("Senders map {:?}", s);
                if let Some(sender) = s.get(&(id_2 as u8)) {
                    if let Some(s_id_1)= sender_id_1{
                    let res = sender.send(DroneCommand::AddSender(id_1 as u8, s_id_1)); 
                    match res {
                        Ok(_) => {
                            println!("AddSender command sent to drone {} to add {}", id_2, id_1);
                        }
                        Err(e) => {
                            println!("Error sending AddSender command to drone {}: {:?}", id_2, e);
                        }
                    }
                } else {
                    println!("No sender for drone {}", id_2);
                }
            }
            } else {
                println!("No senders map loaded");
            }

            // ADD EDGE
            let edges = window.get_edges();
            if let Some(edge) = edges.as_any().downcast_ref::<VecModel<Edge>>() {
                if id_1<id_2{
                    edge.push(Edge{id1: id_1, id2: id_2});
                }else{
                    edge.push(Edge{id1: id_2, id2: id_1});
                }
            }

            // ADD ADJCENT TO NODE (and remove from not_adjacent)
            let drones = window.get_drones();
            if let Some(drone) = drones.as_any().downcast_ref::<VecModel<Drone>>() {
                for d in drone.iter(){
                    if d.id==id_1{
                        if let Some(adj) = d.adjent.as_any().downcast_ref::<VecModel<i32>>() {
                            adj.push(id_2);
                        }
                        if let Some(n_adj) = d.not_adjacent.as_any().downcast_ref::<VecModel<i32>>() {
                            let mut i = 0;
                            for e in n_adj.iter(){
                                if e==id_2{
                                    n_adj.remove(i);
                                    break;
                                }
                                i = i+1;
                            }
                        }
                    }
                    if d.id==id_2{
                        if let Some(adj) = d.adjent.as_any().downcast_ref::<VecModel<i32>>() {
                            adj.push(id_1);
                        }
                        if let Some(n_adj) = d.not_adjacent.as_any().downcast_ref::<VecModel<i32>>() {
                            let mut i = 0;
                            for e in n_adj.iter(){
                                if e==id_1{
                                    n_adj.remove(i);
                                    break;
                                }
                                i = i+1;
                            }
                        }
                    }
                }
            }

        }
    });


    // Note: the channels are not created becuase the connection the drones already exists, they are only cloned
    let weak = main_window.as_weak();
    let senders = sc_senders.clone();
    let channels_ = channels.clone();
    main_window.on_add_edge_client_server(move || {
        println!("[SIMULATION CONTROLLER ] ADD EDGE CLIENT-SERVER");
        if let Some(window) = weak.upgrade() {
            let id_1 = window.get_sender_id();
            let id_2 = window.get_receiver_id();

            let mut sender_id_1: Option<Sender<Packet>> = None;
            let mut sender_id_2: Option<Sender<Packet>> = None;

            if let Some(ref mut channel) = *channels_.lock().unwrap() {

                if let Some(ch_1) = (*channel).get(&(id_1 as u8)) {
                    sender_id_1 = Some(ch_1.sender.clone());
                } else {
                    println!("No sender for id1");
                }

                if let Some(ch_2) = channel.get(&(id_2 as u8)) {
                    sender_id_2 = Some(ch_2.sender.clone());
                } else {
                    println!("No sender for id2");
                }
            } else {
                println!("No senders map loaded");
            }


            // COMMUNICATE ADDITION TO DRONES to id_1
            if let Some(ref mut s) = *senders.lock().unwrap() {
                println!("Senders map {:?}", s);
                if let Some(sender) = s.get(&(id_1 as u8)) {
                    if let Some(s_id_2)= sender_id_2{
                    let res = sender.send(DroneCommand::AddSender(id_2 as u8, s_id_2)); 
                    match res {
                        Ok(_) => {
                            println!("AddSender command sent to drone {} to add {}", id_1, id_2);
                        }
                        Err(e) => {
                            println!("Error sending AddSender command to drone {}: {:?}", id_1, e);
                        }
                    }
                } else {
                    println!("No sender for drone {}", id_1);
                }
            }
            } else {
                println!("No senders map loaded");
            }

            // COMMUNICATE ADDITION TO DRONES to id_2
            if let Some(ref mut s) = *senders.lock().unwrap() {
                println!("Senders map {:?}", s);
                if let Some(sender) = s.get(&(id_2 as u8)) {
                    if let Some(s_id_1)= sender_id_1{
                    let res = sender.send(DroneCommand::AddSender(id_1 as u8, s_id_1)); 
                    match res {
                        Ok(_) => {
                            println!("AddSender command sent to drone {} to add {}", id_2, id_1);
                        }
                        Err(e) => {
                            println!("Error sending AddSender command to drone {}: {:?}", id_2, e);
                        }
                    }
                } else {
                    println!("No sender for drone {}", id_2);
                }
            }
            } else {
                println!("No senders map loaded");
            }

            // ADD EDGE
            let edges = window.get_edges();
            if let Some(edge) = edges.as_any().downcast_ref::<VecModel<Edge>>() {
                if id_1<id_2{
                    edge.push(Edge{id1: id_1, id2: id_2});
                }else{
                    edge.push(Edge{id1: id_2, id2: id_1});
                }
            }else{
                println!("problems in downcasting edges in add_edges");
            }

            // ADD ADJCENT TO CLIENT/SERVER (and remove from not_adjacent)
            let tmp1 : i32; // lower -> drone
            let tmp2 : i32; // higher -> client or server 
            if id_1<id_2{
                tmp1 = id_1;
                tmp2 = id_2;
            }else{
                tmp1 = id_2;
                tmp2 = id_1;
            }
            if tmp2<30{ //->clinet
                let clients = window.get_clients();
                for c in clients.iter(){
                    if c.id == tmp2{
                        if let Some(adj) = c.drones_adjacent.as_any().downcast_ref::<VecModel<i32>>() {
                            adj.push(tmp1);
                        }
                        if let Some(n_adj) = c.drones_not_adjacent.as_any().downcast_ref::<VecModel<i32>>() {
                            let mut i = 0;
                            for e in n_adj.iter(){
                                if e==tmp1{
                                    n_adj.remove(i);
                                    break;
                                }
                                i = i+1;
                            }
                        }
                    }
                }
            }else{ //->server
                let servers = window.get_servers();
                for s in servers.iter(){
                    if s.id == tmp2{
                        if let Some(adj) = s.drones_adjacent.as_any().downcast_ref::<VecModel<i32>>() {
                            adj.push(tmp1);
                        }
                        if let Some(n_adj) = s.drones_not_adjacent.as_any().downcast_ref::<VecModel<i32>>() {
                            let mut i = 0;
                            for e in n_adj.iter(){
                                if e==tmp1{
                                    n_adj.remove(i);
                                    break;
                                }
                                i = i+1;
                            }
                        }
                    }
                }
            }
        
        }
    });



    let weak = main_window.as_weak();
    let senders = sc_senders.clone();
    main_window.on_change_pdr(move || {
        println!("[SIMULATION CONTROLLER ] CHANGE PDR");
        if let Some(window) = weak.upgrade() {
            let id = window.get_id_selected_drone();
            let new_pdr = window.get_new_pdr();

            if let Some(ref mut s) = *senders.lock().unwrap() {
                println!("Senders map {:?}", s);
                if let Some(sender) = s.get(&(id as u8)) {
                    let res = sender.send(DroneCommand::SetPacketDropRate(new_pdr)); 
                    match res {
                        Ok(_) => {
                            println!("SentPacketDropRate command sent to drone {}", id);
                        }
                        Err(e) => {
                            println!("Error sending SentPacketDropRate command to drone {}: {:?}", id, e);
                        }
                    }
                } else {
                    println!("No sender for drone {}", id);
                }
            } else {
                println!("No senders map loaded");
            }
        }  
    });

    let _res = main_window.run();

    Ok(())
}



// TODO: 
// - chiedere domani -> devo controllare per network partitions?
// - testing with different comfig files
// - controllare tutte cose segnate sul protocollo
// - vogliamo aggiungere nodi nuovi? come?
// - vogliamo cambiare la configurazione? come?