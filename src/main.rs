slint::include_modules!();
use crossbeam::channel::{unbounded, Receiver, Select, Sender};
use logger::Logger;
use network_initializer::{errors::ConfigError, NetworkInitializer};
use rfd::FileDialog;
use slint::{Model, Window};
use slint::{ModelRc, VecModel};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::thread;
use std::{
    cell::{RefCell, RefMut},
    ffi::OsString,
    rc::Rc,
    time::Duration,
};
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

// fn run_config_simulation(config: Arc<Mutex<Result<NetworkInitializer, ConfigError>>>) {
//     thread::spawn(move || {
//         if let Ok(ref mut c) = *config.lock().unwrap(){
//             println!("{:?}",c);
//             c.run_simulation();
//             println!("Simulation done");
//         }
//     });
// }

fn main() -> Result<(), slint::PlatformError> {
    let logger = Logger::new(0, true, "SimulationController".to_string());

    let mut main_window = MainWindow::new()?;
    let weak = main_window.as_weak();
    let weak1 = main_window.as_weak();
    let weak2 = main_window.as_weak();
    let weak3 = main_window.as_weak();
    let weak4 = main_window.as_weak();

    let mut general_receiver: Arc<Mutex<Option<Receiver<DroneEvent>>>> = Arc::new(Mutex::new(None));
    let mut general_receiver1 = general_receiver.clone();
    let mut general_receiver2 = general_receiver.clone();

    let mut senders: Arc<Mutex<Option<HashMap<NodeId, Sender<DroneCommand>>>>> =
        Arc::new(Mutex::new(None));
    let mut senders1 = senders.clone();
    let mut senders2 = senders.clone();
    let mut senders3 = senders.clone();
    let mut senders4 = senders.clone();
    let mut senders5 = senders.clone();

    let mut network_initializer: Arc<Mutex<Result<NetworkInitializer, ConfigError>>> =
        Arc::new(Mutex::new(NetworkInitializer::new(None)));
    let mut network_initializer1: Arc<Mutex<Result<NetworkInitializer, ConfigError>>> =
        network_initializer.clone();
    let mut network_initializer2: Arc<Mutex<Result<NetworkInitializer, ConfigError>>> =
        network_initializer.clone();
    let mut network_initializer3: Arc<Mutex<Result<NetworkInitializer, ConfigError>>> = network_initializer.clone();
    let mut network_initializer4: Arc<Mutex<Result<NetworkInitializer, ConfigError>>> = network_initializer.clone();

    let mut channels: Arc<Mutex<Option<HashMap<NodeId, Channel<Packet>>>>> = Arc::new(Mutex::new(None));
    let mut channels1 = channels.clone();
    let mut channels2 = channels.clone();
    let mut channels3 = channels.clone();

    // thread for checking if there is a configuration and run it
    thread::spawn(move || {
        loop {
            thread::sleep(Duration::from_millis(5000));

            println!("[SIM-CONTROLLER] Selecting config");
            // let mut n1 = network_initializer2.clone();
            if let Ok(ref mut c) = *network_initializer2.lock().unwrap() {
                c.run_simulation();
            }
        }
    });

    // thread for message receiving from drones
    thread::spawn(move || {
        // let mut n1 = network_initializer2.clone();
        if let Some(window) = weak2.upgrade() {
            loop {
                thread::sleep(Duration::from_millis(5000));
                if let Some(ref mut rec_from_drones) = *general_receiver2.lock().unwrap() {
                    let mut select = Select::new();
                    let oper1 = select.recv(&rec_from_drones); // Blocks until a message is available
                    let message = select.select();
                    match message.recv(&rec_from_drones) {
                        Ok(DroneEvent::PacketDropped(packet)) => {
                            println!("[SIMULATION CONTROLLER] PacketDropped {:?}", packet);
                            // let mut messages : ModelRc<Message> = window.get_messages();
                            // if let Some(vec_model) = messages.as_any().downcast_ref::<VecModel<Message>>() {
                            //     vec_model.push(Message{id1: packet.routing_header.hops[packet.routing_header.hop_index-1] as i32 , id2: packet.routing_header.hops[packet.routing_header.hop_index] as i32});
                            // }
                            // window.set_messages(messages);
                        }
                        Ok(DroneEvent::PacketSent(packet)) => {
                            println!("[SIMULATION CONTROLLER] PacketSent {:?}", packet);
                            // let mut messages : ModelRc<Message> = window.get_messages();
                            // if let Some(vec_model) = messages.as_any().downcast_ref::<VecModel<Message>>() {
                            //     vec_model.push(Message{id1: packet.routing_header.hops[packet.routing_header.hop_index-1] as i32 , id2: packet.routing_header.hops[packet.routing_header.hop_index] as i32});
                            // }
                            // window.set_messages(messages);
                        }
                        Ok(DroneEvent::ControllerShortcut(packet)) => {
                            println!("[SIMULATION CONTROLLER] ControllerShortcut {:?}", packet);
                            // add to message queue
                            // let mut messages : ModelRc<Message> = window.get_messages();
                            // if let Some(vec_model) = messages.as_any().downcast_ref::<VecModel<Message>>() {
                            //     vec_model.push(Message{id1: packet.routing_header.hops[packet.routing_header.hop_index-1] as i32 , id2: packet.routing_header.hops[packet.routing_header.hop_index] as i32});
                            // }
                            // window.set_messages(messages);

                            // send directly to destination -> TO TEST
                            let destination = packet.routing_header.hops[packet.routing_header.hops.len() - 1];
                            if let Some(ref mut s) = *channels1.lock().unwrap() {
                                if let Some(channel) = (*s).get(&destination) {
                                    channel.sender.send(packet);
                                } else {
                                    println!("No sender for drone {}", destination);
                                }

                            } else {
                                println!("No senders map loaded");
                            }

                        }
                        Err(e) => {
                            println!("Error receiving message: {:?}", e);
                        }
                        _ => {
                            println!("Unknown message");
                        }
                    }
                } else {
                    println!("No receiver");
                }
            }
        }
    });

    main_window.on_select_file(move || {
        logger.log_info("file selection");
        let file = FileDialog::new().pick_file();
        if let Some(mut path) = file {
            let path_string = <OsString as Clone>::clone(&path.as_mut_os_string()).into_string();
            if let Ok(path_string) = path_string {
                println!("Selected file: {}", path_string);

                // before initilizing check kill all the nodes sending a crash command
                if let Some(window) = weak.upgrade() {
                    let drones = window.get_drones();
                    let clients = window.get_clients();
                    let servers = window.get_servers();

                    for drone in drones.iter() {
                        if let Some(ref mut s) = *senders2.lock().unwrap() {
                            if let Some(sender) = s.get(&(drone.id as u8)) {
                                if drone.crashed {
                                    continue;
                                }
                                let res = sender.send(DroneCommand::Crash);
                                match res {
                                    Ok(_) => {
                                        println!("Crash command sent to drone {}", drone.id);
                                    }
                                    Err(e) => {
                                        println!(
                                            "Error sending crash command to drone {}: {:?}",
                                            drone.id, e
                                        );
                                    }
                                }
                            } else {
                                println!("No sender for drone {}", drone.id);
                            }
                        } else {
                            println!("No senders map loaded");
                        }
                    }

                    for client in clients.iter() {
                        if let Some(ref mut s) = *senders2.lock().unwrap() {
                            if let Some(sender) = s.get(&(client.id as u8)) {
                                let res = sender.send(DroneCommand::Crash);
                                match res {
                                    Ok(_) => {
                                        println!("Crash command sent to drone {}", client.id);
                                    }
                                    Err(e) => {
                                        println!(
                                            "Error sending crash command to drone {}: {:?}",
                                            client.id, e
                                        );
                                    }
                                }
                            } else {
                                println!("No sender for drone {}", client.id);
                            }
                        } else {
                            println!("No senders map loaded");
                        }
                    }

                    for server in servers.iter() {
                        if let Some(ref mut s) = *senders2.lock().unwrap() {
                            if let Some(sender) = s.get(&(server.id as u8)) {
                                let res = sender.send(DroneCommand::Crash);
                                match res {
                                    Ok(_) => {
                                        println!("Crash command sent to drone {}", server.id);
                                    }
                                    Err(e) => {
                                        println!(
                                            "Error sending crash command to drone {}: {:?}",
                                            server.id, e
                                        );
                                    }
                                }
                            } else {
                                println!("No sender for drone {}", server.id);
                            }
                        } else {
                            println!("No senders map loaded");
                        }
                    }

                    window.set_drones(slint::ModelRc::new(slint::VecModel::from(vec![])));
                    window.set_clients(slint::ModelRc::new(slint::VecModel::from(vec![])));
                    window.set_servers(slint::ModelRc::new(slint::VecModel::from(vec![])));
                    window.set_edges(slint::ModelRc::new(slint::VecModel::from(vec![])));

                    // reset configuration and decomment line
                    let lock_result = network_initializer1.lock();
                    if let Ok(mut guard) = lock_result {
                        if let Ok(network) = &mut *guard {
                            // network.stop_simulation();
                            println!("[SIMULATION-CONTROLLER]Error: Failed to acquire lock");
                        } else {
                            println!("[SIMULATION-CONTROLLER]Error: Failed to acquire lock");
                        }
                    }

                    // enforce new configuration
                    *network_initializer1.lock().unwrap() =
                        NetworkInitializer::new(Some(path_string.as_str()));
                    let mut config = network_initializer1.lock().unwrap();

                    if let Ok(ref mut c) = *config {
                        *general_receiver1.lock().unwrap() = Some(c.get_controller_recv());
                        println!("Rec directly from config {:?}", c.get_controller_recv());
                        *senders.lock().unwrap() = Some(c.get_controller_senders());
                        println!(
                            "Senders directly from config {:?}",
                            c.get_controller_senders()
                        );
                        println!("Sender in loading {:?}", senders.lock().unwrap());

                        *channels.lock().unwrap() = Some(c.get_channels());

                        let from_network_initializer = c.get_nodes();

                        let mut edges: Vec<Edge> = vec![];

                        let mut clients: Vec<Drone> = vec![];
                        for drone in from_network_initializer.1 {
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
                            for d in from_network_initializer.0 {
                                if !adjent.contains(&(d.id as i32)) && d.id != drone.id {
                                    not_adj.push(d.id as i32);
                                }
                            }

                            clients.push(Drone {
                                adjent: slint::ModelRc::new(slint::VecModel::from(adjent)),
                                not_adjacent: slint::ModelRc::new(slint::VecModel::from(not_adj)),
                                crashed: false,
                                id: drone.id as i32,
                                pdr: 0.0,
                            });
                        }

                        let mut drones: Vec<Drone> = vec![];
                        for drone in from_network_initializer.0 {
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
                            for d in from_network_initializer.0 {
                                if !adjent.contains(&(d.id as i32)) && d.id != drone.id {
                                    not_adj.push(d.id as i32);
                                }
                            }
                            drones.push(Drone {
                                adjent: slint::ModelRc::new(slint::VecModel::from(adjent)),
                                not_adjacent: slint::ModelRc::new(slint::VecModel::from(not_adj)),
                                crashed: false,
                                id: drone.id as i32,
                                pdr: drone.pdr,
                            });
                        }

                        let mut servers: Vec<Drone> = vec![];
                        for drone in from_network_initializer.2 {
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
                            for d in from_network_initializer.0 {
                                if !adjent.contains(&(d.id as i32)) && d.id != drone.id {
                                    not_adj.push(d.id as i32);
                                }
                            }

                            // TODO : add non adj
                            servers.push(Drone {
                                adjent: slint::ModelRc::new(slint::VecModel::from(adjent)),
                                not_adjacent: slint::ModelRc::new(slint::VecModel::from(not_adj)),
                                crashed: false,
                                id: drone.id as i32,
                                pdr: 0.0,
                            });
                        }

                        window.set_edges(slint::ModelRc::new(slint::VecModel::from(edges)));

                        window.set_clients(slint::ModelRc::new(slint::VecModel::from(clients)));
                        window.set_drones(slint::ModelRc::new(slint::VecModel::from(drones)));
                        window.set_servers(slint::ModelRc::new(slint::VecModel::from(servers)));
                        window.set_messages(slint::ModelRc::new(slint::VecModel::from(vec![
                            Message { id1: 0, id2: 2 },
                            Message { id1: 20, id2: 1 },
                        ])));
                    }
                }
            } else {
                println!("Error converting path to string");
            }
        } else {
            println!("No file selected\nTry again");
        }
    });

    main_window.on_remove_edges(move || {
        if let Some(window) = weak1.upgrade() {
            let id = window.get_id_selected_drone();
            if let Some(ref mut s) = *senders1.lock().unwrap() {
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

            // remove its edges
            println!("[SIMULATION CONTROLLER] REMOVE EDGES");
            let mut edges = window.get_edges();
            let mut edges_new: Vec<Edge> = vec![];
            for edge in edges.iter() {
                if edge.id1 == window.get_id_selected_drone()
                    || edge.id2 == window.get_id_selected_drone()
                {
                    println!("removed {} {}", edge.id1, edge.id2);
                } else {
                    edges_new.push(Edge {
                        id1: edge.id1,
                        id2: edge.id2,
                    });
                }
            }
            window.set_edges(slint::ModelRc::new(slint::VecModel::from(edges_new)));
        }
    });

    // TODO: remove channel
    main_window.on_remove_edge(move || {
        println!("[SIMULATION CONTROLLER ]");
        if let Some(window) = weak3.upgrade() {
            let mut id_1 = window.get_id_selected_drone();
            let mut id_2 = window.get_edge_selected();

            // REMOVE EDGE
            let mut edges = window.get_edges();
            let mut edges_new: Vec<Edge> = vec![];
            for edge in edges.iter() {
                if (edge.id1 == id_1 && edge.id2 == id_2) || (edge.id1 == id_2 && edge.id2 == id_1)
                {
                    println!("removed {} {}", edge.id1, edge.id2);
                } else {
                    edges_new.push(Edge {
                        id1: edge.id1,
                        id2: edge.id2,
                    });
                }
            }
            window.set_edges(slint::ModelRc::new(slint::VecModel::from(edges_new)));

            // COMMUNICATE REMOTION TO DRONES to id_1
            if let Some(ref mut s) = *senders3.lock().unwrap() {
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
            if let Some(ref mut s) = *senders3.lock().unwrap() {
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

            // remove channel -> TO TEST (dovrei controllare se ha delle connessioni e in caso eliminarle??????)
            // if let Some(ref mut channel) = *channels2.lock().unwrap() {

            //     if let Some(channel_) = s.get(&(id_1 as u8)) {
            //         let res = sender.send(DroneCommand::RemoveSender(id_2 as u8));
            //         match res {
            //             Ok(_) => {
            //                 println!("RemoveSender command sent to drone {} to remove {}", id_1, id_2);
            //             }
            //             Err(e) => {
            //                 println!("Error sending RemoveSender command to drone {}: {:?}", id_1, e);
            //             }
            //         }
            //     } else {
            //         println!("No sender for drone {}", id_1);
            //     }
            // } else {
            //     println!("No senders map loaded");
            // }


            // REMOVE ADJACENT
            let mut drones = window.get_drones();
            let mut drones_new: Vec<Drone> = vec![];
            for drone in drones.iter() {
                if drone.id == id_1 {
                    let mut adjent = vec![];
                    for adj in drone.adjent.iter() {
                        if adj != id_2 {
                            adjent.push(adj);
                        }
                    }
                    let mut not_a = vec![];
                    for not_adj in drone.not_adjacent.iter() {
                        not_a.push(not_adj);
                    }
                    not_a.push(id_2);

                    drones_new.push(Drone {
                        adjent: slint::ModelRc::new(slint::VecModel::from(adjent)),
                        not_adjacent: slint::ModelRc::new(slint::VecModel::from(not_a)),
                        crashed: drone.crashed,
                        id: drone.id,
                        pdr: drone.pdr,
                    });
                } else if drone.id == id_2 {
                    let mut adjent = vec![];
                    for adj in drone.adjent.iter() {
                        if adj != id_1 {
                            adjent.push(adj);
                        }
                    }

                    let mut not_a = vec![];
                    for not_adj in drone.not_adjacent.iter() {
                        not_a.push(not_adj);
                    }
                    not_a.push(id_1);
                    drones_new.push(Drone {
                        adjent: slint::ModelRc::new(slint::VecModel::from(adjent)),
                        not_adjacent: slint::ModelRc::new(slint::VecModel::from(not_a)),
                        crashed: drone.crashed,
                        id: drone.id,
                        pdr: drone.pdr,
                    });
                } else {
                    drones_new.push(Drone {
                        adjent: drone.adjent.clone(),
                        not_adjacent: drone.not_adjacent.clone(),
                        crashed: drone.crashed,
                        id: drone.id,
                        pdr: drone.pdr,
                    });
                }
            }
            window.set_drones(slint::ModelRc::new(slint::VecModel::from(drones_new)));
        }
    });

    // TODO: add channel
    main_window.on_add_edge(move || {
        println!("[SIMULATION CONTROLLER ] ADD EDGE");
        if let Some(window) = weak4.upgrade() {
            let mut id_1 = window.get_id_selected_drone();
            let mut id_2 = window.get_edge_selected();

            // ADD EDGE
            let mut edges = window.get_edges();
            if let Some(edge) = edges.as_any().downcast_ref::<VecModel<Edge>>() {
                if id_1<id_2{
                    edge.push(Edge{id1: id_1, id2: id_2});
                }else{
                    edge.push(Edge{id1: id_2, id2: id_1});
                }
            }

            // ADD ADJCENT TO NODE (and remove from not_adjacent)
            let mut drones = window.get_drones();
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

            // TODO: create channels in reality from NI
            let mut sender_id_1: Option<Sender<Packet>> = None;
            let mut sender_id_2: Option<Sender<Packet>> = None;

            if let Some(ref mut channel) = *channels2.lock().unwrap() {

                if let Some(ch_1) = (*channel).get(&(id_1 as u8)) {
                    sender_id_1 = Some(ch_1.sender.clone());
                } else {
                    // TODO: channel to create and insert
                }

                if let Some(ch_2) = channel.get(&(id_2 as u8)) {
                    sender_id_2 = Some(ch_2.sender.clone());
                } else {
                    // TODO: channel to create and insert
                }
            } else {
                println!("No senders map loaded");
            }


            // COMMUNICATE ADDITION TO DRONES to id_1
            if let Some(ref mut s) = *senders4.lock().unwrap() {
                println!("Senders map {:?}", s);
                if let Some(sender) = s.get(&(id_1 as u8)) {
                    if let Some(s_id_2)= sender_id_2{
                    let res = sender.send(DroneCommand::AddSender(id_2 as u8, s_id_2)); 
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
            }
            } else {
                println!("No senders map loaded");
            }

            // COMMUNICATE ADDITION TO DRONES to id_2
            if let Some(ref mut s) = *senders4.lock().unwrap() {
                println!("Senders map {:?}", s);
                if let Some(sender) = s.get(&(id_2 as u8)) {
                    if let Some(s_id_1)= sender_id_1{
                    let res = sender.send(DroneCommand::AddSender(id_1 as u8, s_id_1)); 
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
            }
            } else {
                println!("No senders map loaded");
            }

        }

    });

    main_window.run();

    // println!("Set config to :{:?} ", general_receiver);
    Ok(())
}

// TODO:
// - Handle ControllerShortcut(packet) message
// - Add sending -> RemoveSender(NodeId),
//               -> AddSender(NodeId, Sender<Packet>),
//               -> SetPacketDropRate(f32),
// both graphically and from the controller
