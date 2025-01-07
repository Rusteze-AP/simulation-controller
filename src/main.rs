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

// POPULATE drones, clients and servers
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

        servers.push(ClientServer {
            drones_adjacent: slint::ModelRc::new(slint::VecModel::from(adjent)),
            drones_not_adjacent: slint::ModelRc::new(slint::VecModel::from(not_adj)),
            id: server.id as i32,
        });
    }
    return servers;
}

// send drone commands 
fn send_drone_command(senders: &Arc<Mutex<Option<HashMap<u8, Sender<DroneCommand>>>>>, id:u8, command: Box<DroneCommand>)->Result<(), String>{
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

        println!("[SIMULATION CONTROLLER] initilializing nodes");
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

    // callback called when crashing a drone
    let weak = main_window.as_weak();
    let senders = sc_senders.clone();
    let channels_ = channels.clone();
    main_window.on_crash(move || {
        if let Some(window) = weak.upgrade() {
            let id = window.get_id_selected_drone();

            // SEND COMMAND TO DRONE
            match send_drone_command(&senders, id as u8, Box::new(DroneCommand::Crash)){
                Ok(_) => {
                    println!("[SIMULATION CONTROLLER] Drone {} crashed", id);
                }
                Err(e) => {
                    println!("[SIMULATION CONTROLLER] Error crashing drone {}: {:?}", id, e);
                }
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


            // COMMUNICATE REMOTION TO DRONES to id_1 and id_2
            match send_drone_command(&senders, id_1 as u8, Box::new(DroneCommand::RemoveSender(id_2 as u8))){
                Ok(_) => {
                    println!("[SIMULATION CONTROLLER] Drone {} removed edge to drone {}", id_1, id_2);
                }
                Err(e) => {
                    println!("[SIMULATION CONTROLLER] Error removing edge from drone {} to drone {}: {:?}", id_1, id_2, e);
                }
            }

            match send_drone_command(&senders, id_2 as u8, Box::new(DroneCommand::RemoveSender(id_1 as u8))){
                Ok(_) => {
                    println!("[SIMULATION CONTROLLER] Drone {} removed edge to drone {}", id_2, id_1);
                }
                Err(e) => {
                    println!("[SIMULATION CONTROLLER] Error removing edge from drone {} to drone {}: {:?}", id_2, id_1, e);
                }
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

            // COMMUNICATE REMOTION TO DRONES to id_1 and id_2
            match send_drone_command(&senders, id_1 as u8, Box::new(DroneCommand::RemoveSender(id_2 as u8))){
                Ok(_) => {
                    println!("[SIMULATION CONTROLLER] Drone {} removed edge to drone {}", id_1, id_2);
                }
                Err(e) => {
                    println!("[SIMULATION CONTROLLER] Error removing edge from drone {} to drone {}: {:?}", id_1, id_2, e);
                }
            }

            match send_drone_command(&senders, id_2 as u8, Box::new(DroneCommand::RemoveSender(id_1 as u8))){
                Ok(_) => {
                    println!("[SIMULATION CONTROLLER] Drone {} removed edge to drone {}", id_2, id_1);
                }
                Err(e) => {
                    println!("[SIMULATION CONTROLLER] Error removing edge from drone {} to drone {}: {:?}", id_2, id_1, e);
                }
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


            // COMMUNICATE ADDITION TO DRONES to id_1 and id_2
            if let Some(s_id_2)= sender_id_2{
                match send_drone_command(&senders, id_1 as u8, Box::new(DroneCommand::AddSender(id_2 as u8, s_id_2))){
                    Ok(_) => {
                        println!("[SIMULATION CONTROLLER] Drone {} add edge to drone {}", id_1, id_2);
                    }
                    Err(e) => {
                        println!("[SIMULATION CONTROLLER] Error adding edge from drone {} to drone {}: {:?}", id_1, id_2, e);
                    }
                }
            } else {
                println!("[SIMULATION CONTROLLER] No sender for drone {} to add", id_2);
            }

            if let Some(s_id_1)= sender_id_1{
                match send_drone_command(&senders, id_2 as u8, Box::new(DroneCommand::AddSender(id_1 as u8, s_id_1))){
                    Ok(_) => {
                        println!("[SIMULATION CONTROLLER] Drone {} add edge to drone {}", id_2, id_1);
                    }
                    Err(e) => {
                        println!("[SIMULATION CONTROLLER] Error adding edge from drone {} to drone {}: {:?}", id_2, id_1, e);
                    }
                }
            } else {
                println!("[SIMULATION CONTROLLER] No sender for drone {} to add", id_1);
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


            // COMMUNICATE ADDITION TO DRONES to id_1 and id_2
            if let Some(s_id_2)= sender_id_2{
                match send_drone_command(&senders, id_1 as u8, Box::new(DroneCommand::AddSender(id_2 as u8, s_id_2))){
                    Ok(_) => {
                        println!("[SIMULATION CONTROLLER] Node with id {} add edge to node with id {}", id_1, id_2);
                    }
                    Err(e) => {
                        println!("[SIMULATION CONTROLLER] Error adding edge from node with id {} to node with id {}: {:?}", id_1, id_2, e);
                    }
                }
            } else {
                println!("[SIMULATION CONTROLLER] No sender for node with id {} to add", id_2);
            }

            if let Some(s_id_1)= sender_id_1{
                match send_drone_command(&senders, id_2 as u8, Box::new(DroneCommand::AddSender(id_1 as u8, s_id_1))){
                    Ok(_) => {
                        println!("[SIMULATION CONTROLLER] Node with id {} add edge to node with id {}", id_2, id_1);
                    }
                    Err(e) => {
                        println!("[SIMULATION CONTROLLER] Error adding edge from node with id {} to node with id {}: {:?}", id_2, id_1, e);
                    }
                }
            } else {
                println!("[SIMULATION CONTROLLER] No sender for node with id {} to add", id_1);
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

            match send_drone_command(&senders, id as u8, Box::new(DroneCommand::SetPacketDropRate(new_pdr))){
                Ok(_) => {
                    println!("[SIMULATION CONTROLLER] Drone {} pdr changed to {}", id, new_pdr);
                }
                Err(e) => {
                    println!("[SIMULATION CONTROLLER] Error changing drone {} pdr to {}: {:?}", id, new_pdr, e);
                }
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