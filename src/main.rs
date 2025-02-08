slint::include_modules!();
use crossbeam::channel::{ Receiver, Sender, TryRecvError};
use network_initializer::{errors::ConfigError, NetworkInitializer, parsed_nodes::{ParsedDrone, ParsedClient, ParsedServer}, DroneType};
use slint::{Model, ModelRc, VecModel};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::thread;
use wg_internal::controller::{DroneCommand, DroneEvent};
use wg_internal::network::NodeId;
use wg_internal::packet::{Packet, PacketType, NackType};
use network_initializer::channel::Channel;
use rfd::FileDialog;
use std::ffi::OsString;
use packet_forge::ClientType;
use logger::{Logger, LogLevel};


const PATH: &str = "star.toml";    
const CLIENT_T : ClientType = ClientType::Video;
const DRONE : DroneType = DroneType::NullPointerDrone;

fn check_edges(edges: &Vec<Edge>, id1: i32, id2: i32) -> bool {
    for edge in edges {
        if (edge.id1 == id1 && edge.id2 == id2) || (edge.id1 == id2 && edge.id2 == id1) {
            return true;
        }
    }
    return false;
}

fn populate_all(parsed_drones: &Vec<ParsedDrone>, parsed_servers:&Vec<ParsedServer>, parsed_clients: &Vec<ParsedClient>, edges: &mut Vec<Edge>, id_to_type: &Arc<Mutex<HashMap<i32, (NodeType, i32)>>>)-> (Vec<Drone>, Vec<ClientServer>, Vec<ClientServer>){
    let mut drones: Vec<Drone> = vec![];
    let mut clients: Vec<ClientServer> = vec![];
    let mut servers: Vec<ClientServer> = vec![];
    
    // populate id_to_type
    let mut i = 0;
    for drone in parsed_drones{
        id_to_type.lock().unwrap().insert(drone.id as i32, (NodeType::Drone, i));
        i = i+1;
    }
    i = 0;
    for client in parsed_clients{
        id_to_type.lock().unwrap().insert(client.id as i32, (NodeType::Client, i));
        i = i+1;
    }
    i = 0;
    for server in parsed_servers{
        id_to_type.lock().unwrap().insert(server.id as i32, (NodeType::Server, i));
        i = i+1;
    }

    // populate drones
    i = 0;
    for drone in parsed_drones {
        let mut adjent = vec![];
        for adj in &drone.connected_drone_ids {
            adjent.push(*adj as i32);
            if !check_edges(&edges, drone.id as i32, *adj as i32) {
                let (node_type2_, index) = get_node_type(*adj as i32, id_to_type); 
                edges.push(Edge {
                    id1: drone.id as i32,
                    id2: *adj as i32,
                    node_type1: 0,
                    node_type2: node_type2_,
                    index1: i,
                    index2: index,
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
            crashed: false,
            position_in_vector: i,
        });
        i = i+1;
    }

    // populate clients
    i = 0;
    for client in parsed_clients{
        let mut adjent = vec![];
        for adj in &client.connected_drone_ids {
            adjent.push(*adj as i32);
            if !check_edges(&edges, client.id as i32, *adj as i32) {
                let (node_type2_, index) = get_node_type(*adj as i32, id_to_type);
                edges.push(Edge {
                    id1: client.id as i32,
                    id2: *adj as i32,
                    node_type1 : 1,
                    node_type2 : node_type2_,
                    index1: i,
                    index2: index,
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
            position_in_vector: i,
        });
        i = i+1;
    }

    // populate servers
    i = 0;
    for server in parsed_servers{
        let mut adjent = vec![];
        for adj in &server.connected_drone_ids {
            adjent.push(*adj as i32);
            if !check_edges(&edges, server.id as i32, *adj as i32) {
                let (node_type2_, index) = get_node_type(*adj as i32, id_to_type);
                edges.push(Edge {
                    id1: server.id as i32,
                    id2: *adj as i32,
                    node_type1 : 2,
                    node_type2 : node_type2_,
                    index1: i,
                    index2: index,
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
            position_in_vector: i,
        });
        i = i+1;
    }

    println!("Drones: {:?}", parsed_drones);
    println!("Client: {:?}", parsed_clients);
    println!("Servers: {:?}", parsed_servers);


    return (drones, clients, servers);
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

fn get_node_type(id: i32, id_to_type: &Arc<Mutex<HashMap<i32, (NodeType, i32)>>>) -> (i32, i32) {
    let id_to_type_ = id_to_type.lock().unwrap();
    match id_to_type_.get(&id){
        Some((NodeType::Drone, index)) => return (0, *index),
        Some((NodeType::Client, index)) => return (1, *index),
        Some((NodeType::Server, index)) => return (2, *index),
        None => return (-1, -1),
    }
}
// println!("Drones: {:?}", parsed_drones);
// println!("Client: {:?}", parsed_clients);
// println!("Servers: {:?}", parsed_servers);
// // FOR PARTITION CONTROL WHEN CRASHING NODES
// fn bfs(graph: &HashMap<i32, Vec<i32>>, drones: &vec![Drone] , clients id_start: i32) {
//     // create the graph representation


//     let mut queue = VecDeque::new();
//     let mut visited = std::collections::HashSet::new();

//     queue.push_back(id_start);
//     visited.insert(id_start);

//     while let Some(node) = queue.pop_front() {
//         println!("Visited: {}", node);

//         if let Some(neighbors) = graph.get(&node) {
//             for &neighbor in neighbors {
//                 if !visited.contains(&neighbor) {
//                     queue.push_back(neighbor);
//                     visited.insert(neighbor);
//                 }
//             }
//         }
//     }
// }

#[derive(Debug)]
enum NodeType{
    Drone,
    Client,
    Server,
}

fn main() -> Result<(), slint::PlatformError> {
    let logger: Arc<Mutex<Logger>> = Arc::new(Mutex::new(Logger::new(0, true, "SimulationController".to_string())));
    (*logger).lock().unwrap().add_displayable_flag(LogLevel::All);

    let main_window = Window::new()?;
    let window = main_window.window();
    window.set_fullscreen(true);

    //initial configuration -> default
    let network_initializer: Arc<Mutex<Result<NetworkInitializer, ConfigError>>> = Arc::new(Mutex::new(NetworkInitializer::new(Some(PATH))));
    let mut sc_receiver: Arc<Mutex<Option<Receiver<DroneEvent>>>> = Arc::new(Mutex::new(None));
    let mut sc_senders: Arc<Mutex<Option<HashMap<NodeId, Sender<DroneCommand>>>>> = Arc::new(Mutex::new(None));
    let mut channels: Arc<Mutex<Option<HashMap<NodeId, Channel<Packet>>>>> = Arc::new(Mutex::new(None));
    let id_to_type_pos: Arc<Mutex<HashMap<i32, (NodeType, i32)>>>= Arc::new(Mutex::new(HashMap::new())); // (NodeType, position_in_vector)


    if let Ok(ref mut c)= *network_initializer.lock().unwrap() {
        sc_receiver = Arc::new(Mutex::new(Some((*c).get_controller_recv())));
        sc_senders = Arc::new(Mutex::new(Some((*c).get_controller_senders())));
        channels = Arc::new(Mutex::new(Some((*c).get_channels())));

        let nodes = c.get_nodes();

        let mut edges: Vec<Edge> = vec![];
        let (drones, clients, servers) = populate_all(&nodes.0, &nodes.2, &nodes.1, &mut edges, &id_to_type_pos);
        println!("id_to_type_pos {:?}", id_to_type_pos.lock().unwrap());

        let weak = main_window.as_weak();
        if let Some(window) = weak.upgrade() {
            window.set_edges(slint::ModelRc::new(slint::VecModel::from(edges)));
            window.set_clients(slint::ModelRc::new(slint::VecModel::from(clients)));
            window.set_drones(slint::ModelRc::new(slint::VecModel::from(drones)));
            window.set_servers(slint::ModelRc::new(slint::VecModel::from(servers)));
        }
    }else{
        (*logger).lock().unwrap().log_error(&format!("Error in loading the configuration file"));
    }

    // thread for running the simulation
    let network_initializer_run_simulation = network_initializer.clone();
    let logger_ = logger.clone();
    thread::spawn(move || {
        logger_.lock().unwrap().log_debug("Simulation started");
        if let Ok(ref mut c) = *network_initializer_run_simulation.lock().unwrap() {
            match c.run_simulation(None, Some(vec![CLIENT_T])) {
                Ok(_) => {
                    logger_.lock().unwrap().log_debug("Simulation ended correctly");
                }
                Err(e) => {
                    let error = format!("Simulation ended with error {}", e);
                    logger_.lock().unwrap().log_error(&error);
                }
            }
        }
    });

    // thread for receiving DroneEvent
    let logger_ = logger.clone();
    let sc_receiver_ = sc_receiver.clone();
    let weak = main_window.as_weak();
    let id_to_type_pos_ = id_to_type_pos.clone();

    let downsample_ack = Arc::new(Mutex::new(0));
    let downsample_msg_frag = Arc::new(Mutex::new(0));
    let downsample_nack = Arc::new(Mutex::new(0));
    let downsample_dropped = Arc::new(Mutex::new(0));
    thread::spawn(move ||{
        loop{
            if let Some(sc_rec) = sc_receiver_.lock().unwrap().as_ref(){
                    match sc_rec.try_recv(){
                        // PacketDropped
                        Ok(DroneEvent::PacketDropped(packet)) => {
                            logger_.lock().unwrap().log_debug(&format!("PacketDropped received {:?}", packet));
                            // let logger1 = logger_.clone();
                            let id_to_type_pos1 = id_to_type_pos_.clone();
                            let downsample_dropped1 = downsample_dropped.clone();
                            *downsample_dropped1.lock().unwrap() += 1;
                            if *downsample_dropped1.lock().unwrap() % 10000 == 0 {
                                *downsample_dropped1.lock().unwrap() = 0;
                                match weak.upgrade_in_event_loop(move |window|{
                                    let messages : ModelRc<Message> = window.get_messages();
                                    let (ns1, index1) = get_node_type(packet.routing_header.hops[packet.routing_header.hop_index] as i32, &id_to_type_pos1);
                                    let (ns2, index2) = get_node_type(packet.routing_header.hops[packet.routing_header.hop_index-1] as i32, &id_to_type_pos1);
                                    
                                    if let Some(vec_model) = messages.as_any().downcast_ref::<VecModel<Message>>() {
                                        vec_model.push(Message{id1: packet.routing_header.hops[packet.routing_header.hop_index] as i32 , id2: packet.routing_header.hops[packet.routing_header.hop_index-1] as i32, msg_type:5, node_type1: ns1, node_type2: ns2, index1: index1, index2: index2});
                                    }else{
                                        window.set_messages(slint::ModelRc::new(slint::VecModel::from(vec![
                                            Message { id1: packet.routing_header.hops[packet.routing_header.hop_index] as i32 , id2: packet.routing_header.hops[packet.routing_header.hop_index-1] as i32, msg_type:5, node_type1: ns1, node_type2: ns2, index1: index1, index2: index2},
                                        ])));
                                    }    
                                }){
                                    Ok(_) => {
                                        logger_.lock().unwrap().log_debug("Message sent to window");
                                    },
                                    Err(e) => {
                                        logger_.lock().unwrap().log_error(&format!("Error sending message to window: {}", e));
                                    }
                                }
                            }
                        }
                        // PacketSent
                        Ok(DroneEvent::PacketSent(packet)) => {
                            logger_.lock().unwrap().log_debug(&format!("PacketSent received {:?}", packet));
                            let id_to_type_pos1 = id_to_type_pos_.clone();
                            let downsample_msg_frag1 = downsample_msg_frag.clone();
                            let downsample_ack1 = downsample_ack.clone();
                            let downsample_nack1 = downsample_nack.clone();
                            match weak.upgrade_in_event_loop(move |window|
                                {let messages : ModelRc<Message> = window.get_messages();
                                        let message; 

                                            match packet.pack_type{
                                                PacketType::MsgFragment(_) => {
                                                    let (ns1, index1) = get_node_type(packet.routing_header.hops[packet.routing_header.hop_index-1] as i32, &id_to_type_pos1);
                                                    let (ns2, index2) = get_node_type(packet.routing_header.hops[packet.routing_header.hop_index] as i32, &id_to_type_pos1);
                                                    message = Message { id1: packet.routing_header.hops[packet.routing_header.hop_index-1] as i32 , id2: packet.routing_header.hops[packet.routing_header.hop_index] as i32, msg_type:0, node_type1: ns1, node_type2: ns2, index1: index1, index2: index2};
                                                    *downsample_msg_frag1.lock().unwrap() +=  1;
                                                    if *downsample_msg_frag1.lock().unwrap() %1000==0{
                                                        *downsample_msg_frag1.lock().unwrap() = 0;
                                                        if let Some(vec_model) = messages.as_any().downcast_ref::<VecModel<Message>>() {
                                                            vec_model.push(message);
                                                        }else{
                                                            window.set_messages(slint::ModelRc::new(slint::VecModel::from(vec![
                                                                message,
                                                            ])));
                                                        }
                                                    }
                                                },
                                                PacketType::Ack(_)=> {
                                                    let (ns1, index1) = get_node_type(packet.routing_header.hops[packet.routing_header.hop_index-1] as i32, &id_to_type_pos1);
                                                    let (ns2, index2) = get_node_type(packet.routing_header.hops[packet.routing_header.hop_index] as i32, &id_to_type_pos1);
                                                    message = Message { id1: packet.routing_header.hops[packet.routing_header.hop_index-1] as i32 , id2: packet.routing_header.hops[packet.routing_header.hop_index] as i32, msg_type:1,  node_type1: ns1, node_type2: ns2, index1: index1, index2: index2};
                                                    *downsample_ack1.lock().unwrap() += 1;
                                                    if *downsample_ack1.lock().unwrap()%1000==0{
                                                        *downsample_ack1.lock().unwrap() = 0;
                                                        if let Some(vec_model) = messages.as_any().downcast_ref::<VecModel<Message>>() {
                                                            vec_model.push(message);
                                                        }else{
                                                            window.set_messages(slint::ModelRc::new(slint::VecModel::from(vec![
                                                                message,
                                                            ])));
                                                        }
                                                    }
                                                },
                                                PacketType::Nack(e)=>{
                                                    match e.nack_type{
                                                        NackType::Dropped=>{},
                                                        _ =>{
                                                            let (ns1, index1) = get_node_type(packet.routing_header.hops[packet.routing_header.hop_index-1] as i32, &id_to_type_pos1);
                                                            let (ns2, index2) = get_node_type(packet.routing_header.hops[packet.routing_header.hop_index] as i32, &id_to_type_pos1);
                                                            message = Message { id1: packet.routing_header.hops[packet.routing_header.hop_index-1] as i32 , id2: packet.routing_header.hops[packet.routing_header.hop_index] as i32, msg_type:2,  node_type1: ns1, node_type2: ns2, index1: index1, index2: index2};
                                                            *downsample_nack1.lock().unwrap() += 1;
                                                            if *downsample_nack1.lock().unwrap()%1000==0{
                                                                *downsample_nack1.lock().unwrap() = 0;
                                                                if let Some(vec_model) = messages.as_any().downcast_ref::<VecModel<Message>>() {
                                                                    vec_model.push(message);
                                                                }else{
                                                                    window.set_messages(slint::ModelRc::new(slint::VecModel::from(vec![
                                                                        message,
                                                                    ])));
                                                                }
                                                            }
                                                        }
                                                    }
                                                }, 
                                                PacketType::FloodRequest(flood)=>{
                                                    if flood.path_trace.len()>=2{
                                                        let id_1 = flood.path_trace.get(flood.path_trace.len()-2).unwrap().0;
                                                        let id_2 = flood.path_trace.get(flood.path_trace.len()-1).unwrap().0;
                                                        let (ns1, index1) = get_node_type(id_1 as i32, &id_to_type_pos1);
                                                        let (ns2, index2) = get_node_type(id_2  as i32, &id_to_type_pos1);
                                                        // println!("message: {} {} : flood {:?}", id_1, id_2, flood);
                                                        message = Message { id1: id_1 as i32, id2: id_2 as i32, msg_type:3,  node_type1: ns1, node_type2: ns2, index1: index1, index2: index2};
                                                        if let Some(vec_model) = messages.as_any().downcast_ref::<VecModel<Message>>() {
                                                            // println!("message_pushed {:?}", message);
                                                            vec_model.push(message);
                                                            
                                                        }else{
                                                            // println!("message_pushed {:?}", message);
                                                            window.set_messages(slint::ModelRc::new(slint::VecModel::from(vec![
                                                                message,
                                                            ])));
                                                        }
                                                    }
                                                }
                                                PacketType::FloodResponse(_)=>{
                                                    let (ns1, index1) = get_node_type(packet.routing_header.hops[packet.routing_header.hop_index-1] as i32, &id_to_type_pos1);
                                                    let (ns2, index2) = get_node_type(packet.routing_header.hops[packet.routing_header.hop_index] as i32, &id_to_type_pos1);
                                                    message = Message { id1: packet.routing_header.hops[packet.routing_header.hop_index-1] as i32 , id2: packet.routing_header.hops[packet.routing_header.hop_index] as i32, msg_type:4,  node_type1: ns1, node_type2: ns2, index1: index1, index2: index2};
                                                    if let Some(vec_model) = messages.as_any().downcast_ref::<VecModel<Message>>() {
                                                        vec_model.push(message);
                                                    }else{
                                                        window.set_messages(slint::ModelRc::new(slint::VecModel::from(vec![
                                                            message,
                                                        ])));
                                                    }
                                                }
                                            }
                                    
                                }
                            ){
                                Ok(_) => {
                                    logger_.lock().unwrap().log_debug("Message sent to window");
                                },
                                Err(e) => {
                                    logger_.lock().unwrap().log_error(&format!("Error sending message to window: {}", e));
                                }
                            }
                        }
                        // ControllerShortcut
                        Ok(DroneEvent::ControllerShortcut(packet)) => {
                            logger_.lock().unwrap().log_debug(&format!("ControllerShortcut received {:?}", packet));
                            // let logger1 = logger_.clone();
                            let id_to_type_pos1 = id_to_type_pos_.clone();
                            match weak.upgrade_in_event_loop(move |window|
                                {let messages : ModelRc<Message> = window.get_messages();
                                    let (ns1, index1 ) = get_node_type(packet.routing_header.hops[packet.routing_header.hop_index-1] as i32, &id_to_type_pos1);
                                    let (ns2, index2 )= get_node_type(packet.routing_header.hops[packet.routing_header.hop_index] as i32, &id_to_type_pos1);
                                    
                                    if let Some(vec_model) = messages.as_any().downcast_ref::<VecModel<Message>>() {
                                        vec_model.push(Message{id1: packet.routing_header.hops[packet.routing_header.hop_index-1] as i32 , id2: packet.routing_header.hops[packet.routing_header.hop_index] as i32, msg_type:6, node_type1: ns1, node_type2: ns2, index1: index1, index2: index2});
                                    }else{
                                        window.set_messages(slint::ModelRc::new(slint::VecModel::from(vec![
                                            Message { id1: packet.routing_header.hops[packet.routing_header.hop_index-1] as i32 , id2: packet.routing_header.hops[packet.routing_header.hop_index] as i32, msg_type:6, node_type1: ns1, node_type2: ns2, index1: index1, index2: index2},
                                        ])));
                                    }
   
                                }
                            ){
                                Ok(_) => {
                                    logger_.lock().unwrap().log_debug("Message sent to window");
                                },
                                Err(e) => {
                                    logger_.lock().unwrap().log_error(&format!("Error sending message to window: {}", e));
                                }
                            }
                        },
                        Err(TryRecvError::Empty) => {
                            // logger_.lock().unwrap().log_debug("Empty");
                        },
                        Err(e) => {
                            logger_.lock().unwrap().log_error(&format!("Error receiving message: {}", e));
                        }
                    }
                }
        }
    });

    // callback called when crashing a drone
    let logger_ = logger.clone();
    let weak = main_window.as_weak();
    let senders = sc_senders.clone();
    let channels_ = channels.clone();
    main_window.on_crash(move || {
        if let Some(window) = weak.upgrade() {
            let id = window.get_id_selected_drone();
            logger_.lock().unwrap().log_info("[ON_CRASH]");

            // let drones = window.get_drones();
            // let clients = window.get_clients();
            // let servers = window.get_servers();
            // let net_partition : bool= bfs(drones, clients, servers);


            // SEND COMMAND TO DRONE
            match send_drone_command(&senders, id as u8, Box::new(DroneCommand::Crash)){
                Ok(_) => {
                    logger_.lock().unwrap().log_debug(&format!("[ON_CRASH] Crash command sent to drone {}", id));
                }
                Err(e) => {
                    logger_.lock().unwrap().log_error(&format!("[ON_CRASH] Error sending crash to drone {}: {:?}", id, e));
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
                    logger_.lock().unwrap().log_warn(&format!("[ON_CRASH] Problem in downcasting edges"));
                }
            }

            // REMOVE ALL ADJACENT of others
            let drones = window.get_drones();
            for drone in drones.iter() {
                let mut i = 0;
                for adj in drone.adjent.iter() {
                    if adj == id {
                        if let Some(vec_model) = drone.adjent.as_any().downcast_ref::<VecModel<i32>>() {
                            vec_model.remove(i);
                        }else{
                            logger_.lock().unwrap().log_warn(&format!("[ON_CRASH] Problem in downcasting adjacents"));
                        }

                        match send_drone_command(&senders, drone.id as u8, Box::new(DroneCommand::RemoveSender(id as u8))){
                            Ok(_)=>{
                                logger_.lock().unwrap().log_debug(&format!("[ON_CRASH] RemoveSender sent to Drone {} to remove sender of id {}", drone.id, id));
                            },
                            Err(e)=>{
                                logger_.lock().unwrap().log_error(&format!("[ON_CRASH] Error in sending RemoveSender Drone {} to remove sender of id {} : {}", drone.id, id, e));
                            }
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
                            logger_.lock().unwrap().log_warn(&format!("[ON_CRASH] Problem in downcasting not_adjacents"));

                        }
                        break;
                    }
                    i = i+1;
                }
            }

            //same for servers and clients
            let servers = window.get_servers();
            for server in servers.iter(){
                let mut i = 0;
                for adj in server.drones_adjacent.iter(){
                    if adj == id{
                        if let Some(vec_model) = server.drones_adjacent.as_any().downcast_ref::<VecModel<i32>>() {
                            vec_model.remove(i);
                        }else{
                            logger_.lock().unwrap().log_warn(&format!("[ON_CRASH] Problem in downcasting adjacents"));
                        }

                        match send_drone_command(&senders, server.id as u8, Box::new(DroneCommand::RemoveSender(id as u8))){
                            Ok(_)=>{
                                logger_.lock().unwrap().log_debug(&format!("[ON_CRASH] RemoveSender sent to Server {} to remove sender of id {}", server.id, id));
                            },
                            Err(e)=>{
                                logger_.lock().unwrap().log_error(&format!("[ON_CRASH] Error in sending RemoveSender Server {} to remove sender of id {} : {}", server.id, id, e));

                            }
                        }
                        break;
                    }
                    i = i+1;
                }
                i = 0;
                for not_adj in server.drones_not_adjacent.iter() {
                    if not_adj == id {
                        if let Some(vec_model) = server.drones_not_adjacent.as_any().downcast_ref::<VecModel<i32>>() {
                            vec_model.remove(i);
                        }else{
                            logger_.lock().unwrap().log_warn(&format!("[ON_CRASH] Problem in downcasting not_adjacents"));
                        }
                        break;
                    }
                    i = i+1;
                }
            }

            //same for servers and clients
            let clients = window.get_clients();
            for client in clients.iter(){
                let mut i = 0;
                for adj in client.drones_adjacent.iter(){
                    if adj == id{
                        if let Some(vec_model) = client.drones_adjacent.as_any().downcast_ref::<VecModel<i32>>() {
                            vec_model.remove(i);
                        }else{
                            logger_.lock().unwrap().log_warn(&format!("[ON_CRASH] Problem in downcasting adjacents"));
                        }

                        match send_drone_command(&senders, client.id as u8, Box::new(DroneCommand::RemoveSender(id as u8))){
                            Ok(_)=>{
                                logger_.lock().unwrap().log_debug(&format!("[ON_CRASH] RemoveSender sent to Client {} to remove sender of id {}", client.id, id));
                            },
                            Err(e)=>{
                                logger_.lock().unwrap().log_error(&format!("[ON_CRASH] Error in sending RemoveSender Client {} to remove sender of id {} : {}", client.id, id, e));

                            }
                        }
                        break;
                    }
                    i = i+1;
                }
                i = 0;
                for not_adj in client.drones_not_adjacent.iter() {
                    if not_adj == id {
                        if let Some(vec_model) = client.drones_not_adjacent.as_any().downcast_ref::<VecModel<i32>>() {
                            vec_model.remove(i);
                        }else{
                            logger_.lock().unwrap().log_warn(&format!("[ON_CRASH] Problem in downcasting not_adjacents"));
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
                logger_.lock().unwrap().log_warn(&format!("[ON_CRASH] No channels map loaded"));
            }
            if let Some(ref mut s) = *senders.lock().unwrap() {
                s.remove(&(id as u8));
            } else {
                logger_.lock().unwrap().log_warn(&format!("[ON_CRASH] No sender map loaded"));
            }


        }
    });

    // Note: the channels are not removed becuase the connection cannot be deleted if it is the last of the drone
    let logger_= logger.clone();
    let weak = main_window.as_weak();
    let senders = sc_senders.clone();
    main_window.on_remove_edge(move || {
        logger_.lock().unwrap().log_info("[ON_REMOVE_EDGE]");

        if let Some(window) = weak.upgrade() {
            let id_1 = window.get_sender_id();
            let id_2 = window.get_receiver_id();


            // COMMUNICATE REMOTION TO DRONES to id_1 and id_2
            match send_drone_command(&senders, id_1 as u8, Box::new(DroneCommand::RemoveSender(id_2 as u8))){
                Ok(_) => {
                    logger_.lock().unwrap().log_info(&format!("[ON_REMOVE_EDGE] Command RemoveSender sent to Node {} to remove sender to Node {}", id_1, id_2));
                }
                Err(e) => {
                    logger_.lock().unwrap().log_error(&format!("[ON_REMOVE_EDGE] Error in sending command RemoveSender sent to Node {} to remove sender to Node {} : {}", id_1, id_2, e));
                }
            }

            match send_drone_command(&senders, id_2 as u8, Box::new(DroneCommand::RemoveSender(id_1 as u8))){
                Ok(_) => {
                    logger_.lock().unwrap().log_info(&format!("[ON_REMOVE_EDGE] Command RemoveSender sent to Node {} to remove sender to Node {}", id_2, id_1));
                }
                Err(e) => {
                    logger_.lock().unwrap().log_error(&format!("[ON_REMOVE_EDGE] Error in sending command RemoveSender sent to Node {} to remove sender to Node {} : {}", id_2, id_1, e));
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
                        logger_.lock().unwrap().log_warn("[ON_REMOVE_EDGE] Problems in downcasting edges");
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
                                logger_.lock().unwrap().log_warn("[ON_REMOVE_EDGE] Problems in downcasting adjacent");

                            }
                            if let Some(vec_model) = drone.not_adjacent.as_any().downcast_ref::<VecModel<i32>>() {
                                vec_model.push(id_2);
                            }else{
                                logger_.lock().unwrap().log_warn("[ON_REMOVE_EDGE] Problems in downcasting not_adjacent");
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
                                logger_.lock().unwrap().log_warn("[ON_REMOVE_EDGE] Problems in downcasting adjacent");

                            }
                            if let Some(vec_model) = drone.not_adjacent.as_any().downcast_ref::<VecModel<i32>>() {
                                vec_model.push(id_1);
                            }else{
                                logger_.lock().unwrap().log_warn("[ON_REMOVE_EDGE] Problems in downcasting not_adjacent");
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
    let logger_= logger.clone();
    let weak = main_window.as_weak();
    let senders = sc_senders.clone();
    main_window.on_remove_edge_client_server(move || {
        logger_.lock().unwrap().log_info("[ON_REMOVE_EDGE_CLIENT_SERVER]");

        if let Some(window) = weak.upgrade() {
            let id_1 = window.get_sender_id();
            let id_2 = window.get_receiver_id();

            // COMMUNICATE REMOTION TO DRONES to id_1 and id_2
            match send_drone_command(&senders, id_1 as u8, Box::new(DroneCommand::RemoveSender(id_2 as u8))){
                Ok(_) => {
                    logger_.lock().unwrap().log_info(&format!("[ON_REMOVE_EDGE_CLIENT_SERVER] Command RemoveSender sent to Node {} to remove sender to Node {}", id_1, id_2));
                }
                Err(e) => {
                    logger_.lock().unwrap().log_error(&format!("[ON_REMOVE_EDGE_CLIENT_SERVER] Error in sending command RemoveSender sent to Node {} to remove sender to Node {} : {}", id_1, id_2, e));
                }
            }

            match send_drone_command(&senders, id_2 as u8, Box::new(DroneCommand::RemoveSender(id_1 as u8))){
                Ok(_) => {
                    logger_.lock().unwrap().log_info(&format!("[ON_REMOVE_EDGE_CLIENT_SERVER] Command RemoveSender sent to Node {} to remove sender to Node {}", id_2, id_1));
                }
                Err(e) => {
                    logger_.lock().unwrap().log_error(&format!("[ON_REMOVE_EDGE_CLIENT_SERVER] Error in sending command RemoveSender sent to Node {} to remove sender to Node {} : {}", id_2, id_1, e));
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
                        logger_.lock().unwrap().log_warn("[ON_REMOVE_EDGE_CLIENT_SERVER] Problems in downcasting edges");                    }
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
        }
    });

    // Note: the channels are not created becuase the connection the drones already exists, they are only cloned
    let logger_ = logger.clone();
    let weak = main_window.as_weak();
    let senders = sc_senders.clone();
    let channels_ = channels.clone();
    let id_to_type_pos_ = id_to_type_pos.clone();
    main_window.on_add_edge(move || {
        logger_.lock().unwrap().log_info("[ON_ADD_EDGE]");
    
        if let Some(window) = weak.upgrade() {
            let id_1 = window.get_sender_id();
            let id_2 = window.get_receiver_id();

            let mut sender_id_1: Option<Sender<Packet>> = None;
            let mut sender_id_2: Option<Sender<Packet>> = None;

            if let Some(ref mut channel) = *channels_.lock().unwrap() {

                if let Some(ch_1) = (*channel).get(&(id_1 as u8)) {
                    sender_id_1 = Some(ch_1.sender.clone());
                } else {
                    logger_.lock().unwrap().log_error("[ON_ADD_EDGE] No sender for id1");
                }

                if let Some(ch_2) = channel.get(&(id_2 as u8)) {
                    sender_id_2 = Some(ch_2.sender.clone());
                } else {
                    logger_.lock().unwrap().log_error("[ON_ADD_EDGE] No sender for id2");
                }
            } else {
                logger_.lock().unwrap().log_error("[ON_ADD_EDGE] No sender map loaded");
            }


            // COMMUNICATE ADDITION TO DRONES to id_1 and id_2
            if let Some(s_id_2)= sender_id_2{
                match send_drone_command(&senders, id_1 as u8, Box::new(DroneCommand::AddSender(id_2 as u8, s_id_2))){
                    Ok(_) => {
                        logger_.lock().unwrap().log_debug(&format!("[ON_ADD_EDGE] Command AddSender to Node {} to add edge to Node {}", id_1, id_2));
                    }
                    Err(e) => {
                        logger_.lock().unwrap().log_error(&format!("[ON_ADD_EDGE] Error sending command AddSender to Node {} to add edge to Node {} : {}", id_1, id_2, e));
                    }
                }
            } else {
                logger_.lock().unwrap().log_error(&format!("[ON_ADD_EDGE] No sender for drone {}", id_2));
            }

            if let Some(s_id_1)= sender_id_1{
                match send_drone_command(&senders, id_2 as u8, Box::new(DroneCommand::AddSender(id_1 as u8, s_id_1))){
                    Ok(_) => {
                        logger_.lock().unwrap().log_debug(&format!("[ON_ADD_EDGE] Command AddSender to Node {} to add edge to Node {}", id_2, id_1));
                    }
                    Err(e) => {
                        logger_.lock().unwrap().log_error(&format!("[ON_ADD_EDGE] Error sending command AddSender to Node {} to add edge to Node {} : {}", id_2, id_1, e));
                    }
                }
            } else {
                logger_.lock().unwrap().log_error(&format!("[ON_ADD_EDGE] No sender for drone {}", id_1));
            }

            // ADD EDGE
            let edges = window.get_edges();
            let (_, index1) = get_node_type(id_1, &id_to_type_pos_);
            let (_, index2) = get_node_type(id_2, &id_to_type_pos_);
            if let Some(edge) = edges.as_any().downcast_ref::<VecModel<Edge>>() {
                if id_1<id_2{
                    edge.push(Edge{id1: id_1, id2: id_2, node_type1: 0, node_type2: 0, index1: index1, index2: index2});
                }else{
                    edge.push(Edge{id1: id_2, id2: id_1, node_type1: 0, node_type2: 0, index1: index2, index2: index1});
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
    let logger_ = logger.clone();
    let weak = main_window.as_weak();
    let senders = sc_senders.clone();
    let channels_ = channels.clone();
    let id_to_type_pos_ = id_to_type_pos.clone();
    main_window.on_add_edge_client_server(move || {
        logger_.lock().unwrap().log_info("[ON_ADD_EDGE_CLIENT_SERVER]");
        if let Some(window) = weak.upgrade() {
            let id_1 = window.get_sender_id();
            let id_2 = window.get_receiver_id();

            let mut sender_id_1: Option<Sender<Packet>> = None;
            let mut sender_id_2: Option<Sender<Packet>> = None;

            if let Some(ref mut channel) = *channels_.lock().unwrap() {

                if let Some(ch_1) = (*channel).get(&(id_1 as u8)) {
                    sender_id_1 = Some(ch_1.sender.clone());
                } else {
                    logger_.lock().unwrap().log_error("[ON_ADD_EDGE_CLIENT_SERVER] No sender for id1");
                }

                if let Some(ch_2) = channel.get(&(id_2 as u8)) {
                    sender_id_2 = Some(ch_2.sender.clone());
                } else {
                    logger_.lock().unwrap().log_error("[ON_ADD_EDGE_CLIENT_SERVER] No sender for id2");
                }
            } else {
                logger_.lock().unwrap().log_error("[ON_ADD_EDGE_CLIENT_SERVER] No sender map loaded");
            }


            // COMMUNICATE ADDITION TO DRONES to id_1 and id_2
            if let Some(s_id_2)= sender_id_2{
                match send_drone_command(&senders, id_1 as u8, Box::new(DroneCommand::AddSender(id_2 as u8, s_id_2))){
                    Ok(_) => {
                        logger_.lock().unwrap().log_debug(&format!("[ON_ADD_EDGE_CLIENT_SERVER] Command AddSender to Node {} to add edge to Node {}", id_1, id_2));
                    }
                    Err(e) => {
                        logger_.lock().unwrap().log_debug(&format!("[ON_ADD_EDGE_CLIENT_SERVER] Error sending command AddSender to Node {} to add edge to Node {}: {}", id_1, id_2, e));
                    }
                }
            }

            if let Some(s_id_1)= sender_id_1{
                match send_drone_command(&senders, id_2 as u8, Box::new(DroneCommand::AddSender(id_1 as u8, s_id_1))){
                    Ok(_) => {
                        logger_.lock().unwrap().log_debug(&format!("[ON_ADD_EDGE_CLIENT_SERVER] Command AddSender to Node {} to add edge to Node {}", id_2, id_1));

                    }
                    Err(e) => {
                        logger_.lock().unwrap().log_debug(&format!("[ON_ADD_EDGE_CLIENT_SERVER] Error sending command AddSender to Node {} to add edge to Node {}: {}", id_2, id_1, e));
                    }
                }
            }

            // ADD EDGE
            let edges = window.get_edges();
            if let Some(edge) = edges.as_any().downcast_ref::<VecModel<Edge>>() {
                let (nt1, index1) = get_node_type(id_1, &id_to_type_pos_);
                let (nt2, index2) = get_node_type(id_2, &id_to_type_pos_);

                if id_1<id_2{
                    edge.push(Edge{id1: id_1, id2: id_2, node_type1: nt1, node_type2: nt2, index1: index1, index2: index2});
                }else{
                    edge.push(Edge{id1: id_2, id2: id_1, node_type1: nt2, node_type2: nt1, index1: index2, index2: index1});
                }
            }else{
                logger_.lock().unwrap().log_warn("[ON_ADD_EDGE_CLIENT_SERVER] Problems in downcasting edges");
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


    let logger_ = logger.clone();
    let weak = main_window.as_weak();
    let senders = sc_senders.clone();

    main_window.on_change_pdr(move || {
        logger_.lock().unwrap().log_info("[ON_CHANGE_PDR]");
        if let Some(window) = weak.upgrade() {
            let id = window.get_id_selected_drone();
            let new_pdr = window.get_new_pdr();

            match send_drone_command(&senders, id as u8, Box::new(DroneCommand::SetPacketDropRate(new_pdr))){
                Ok(_) => {
                    logger_.lock().unwrap().log_debug(&format!("[ON_CHANGE_PDR] SetPacketDropRate sent to drone {} to set pdr to {}", id, new_pdr));
                }
                Err(e) => {
                    logger_.lock().unwrap().log_error(&format!("[ON_CHANGE_PDR] Error sending SetPacketDropRate sent to drone {} to set pdr to {} : {}", id, new_pdr, e));
                }
            }
        }
    });

    // NON VA NULLA
    let logger_ = logger.clone();
    let weak = main_window.as_weak();
    let senders = sc_senders.clone();
    let sc_receiver_=sc_receiver.clone();
    let channels_ = channels.clone();
    let id_to_type_pos_ = id_to_type_pos.clone();
    let network_initializer_: Arc<Mutex<Result<NetworkInitializer, ConfigError>>> = network_initializer.clone();
    main_window.on_select_new_file(move || {
        logger_.lock().unwrap().log_info("[ON_SELECT_NEW_FILE]");
        let mut new_path: String = String::from("");
        let mut failed : bool = true;

        // // take new path
        let file = FileDialog::new().pick_file();
        if let Some(mut path) = file {
            let path_string = <OsString as Clone>::clone(&path.as_mut_os_string()).into_string();
            if let Ok(path_string) = path_string {
                logger_.lock().unwrap().log_info(&format!("[ON_SELECT_NEW_FILE] File selected : {}", path_string));
                new_path = path_string;
            }
        }

        
        // if it is valid, clean up all structures
        match NetworkInitializer::new(Some(&new_path)){
            Ok(net_init)=>{

                if let Some(window) = weak.upgrade() {
                    let drones = window.get_drones();

                    for drone in drones.iter(){
                        match send_drone_command(&senders, drone.id as u8, Box::new(DroneCommand::Crash)){
                            Ok(_) => {
                                println!("[SIMULATION CONTROLLER] Drone {} crashed", drone.id);
                                logger_.lock().unwrap().log_debug(&format!("[ON_SELECT_NEW_FILE] Command Crash to Drone {}", drone.id));
                            }
                            Err(e) => {
                                logger_.lock().unwrap().log_error(&format!("[ON_SELECT_NEW_FILE] Error sending Crash to Drone {}: {}", drone.id, e));
                            }
                        }

                        for adj in drone.adjent.iter(){
                            match send_drone_command(&senders, adj as u8, Box::new(DroneCommand::RemoveSender(drone.id as u8))){
                                Ok(_) => {
                                    logger_.lock().unwrap().log_debug(&format!("[ON_SELECT_NEW_FILE] Command RemoveSender to Drone {} to remove {}", drone.id, adj));
                                }
                                Err(e) => {
                                    logger_.lock().unwrap().log_error(&format!("[ON_SELECT_NEW_FILE] Error sending command RemoveSender to Drone {} to remove {}: {}", drone.id, adj, e));
                                }
                            }
                        }

                        if let Some(ref mut channel) = *channels_.lock().unwrap() {
                            channel.remove(&(drone.id as u8));
                        } else {
                            logger_.lock().unwrap().log_warn("[ON_SELECT_NEW_FILE] No channels map loaded");
                        }
                        if let Some(ref mut s) = *senders.lock().unwrap() {
                            s.remove(&(drone.id as u8));
                        } else {
                            logger_.lock().unwrap().log_warn("[ON_SELECT_NEW_FILE] No sender map loaded");
                        }
                    }
                    window.set_drones(slint::ModelRc::new(slint::VecModel::from(vec![])));

                    let clients = window.get_clients();
                    for client in clients.iter(){
                        match send_drone_command(&senders, client.id as u8, Box::new(DroneCommand::Crash)){
                            Ok(_) => {
                                logger_.lock().unwrap().log_debug(&format!("[ON_SELECT_NEW_FILE] Command Crash to Client {}", client.id));
                            }
                            Err(e) => {
                                logger_.lock().unwrap().log_error(&format!("[ON_SELECT_NEW_FILE] Error sending Crash to Client {}: {}", client.id, e));
                            }
                        }

                        if let Some(ref mut channel) = *channels_.lock().unwrap() {
                            channel.remove(&(client.id as u8));
                        } else {
                            logger_.lock().unwrap().log_warn("[ON_SELECT_NEW_FILE] No channels map loaded");
                        }
                        if let Some(ref mut s) = *senders.lock().unwrap() {
                            s.remove(&(client.id as u8));
                        } else {
                            logger_.lock().unwrap().log_warn("[ON_SELECT_NEW_FILE] No senders map loaded");
                        }
                    }
                    window.set_clients(slint::ModelRc::new(slint::VecModel::from(vec![])));

                    let servers = window.get_servers();
                    for server in servers.iter(){
                        match send_drone_command(&senders, server.id as u8, Box::new(DroneCommand::Crash)){
                            Ok(_) => {
                                logger_.lock().unwrap().log_debug(&format!("[ON_SELECT_NEW_FILE] Command Crash to Server {}", server.id));
                            }
                            Err(e) => {
                                logger_.lock().unwrap().log_error(&format!("[ON_SELECT_NEW_FILE] Error sending Crash to Server {}: {}", server.id, e));
                            }
                        }

                        if let Some(ref mut channel) = *channels_.lock().unwrap() {
                            channel.remove(&(server.id as u8));
                        } else {
                            logger_.lock().unwrap().log_warn("[ON_SELECT_NEW_FILE] No channels map loaded");
                        }
                        if let Some(ref mut s) = *senders.lock().unwrap() {
                            s.remove(&(server.id as u8));
                        } else {
                            logger_.lock().unwrap().log_warn("[ON_SELECT_NEW_FILE] No senders map loaded");
                        }
                    }
                    window.set_servers(slint::ModelRc::new(slint::VecModel::from(vec![])));
                    window.set_edges(slint::ModelRc::new(slint::VecModel::from(vec![])));
                }

                // set new config
                while network_initializer_.try_lock().is_err() {
                    // thread::sleep(Duration::from_millis(100));
                }
                *network_initializer_.lock().unwrap() = Ok(net_init);
                failed = false;
            },
            Err(e)=>{
                logger_.lock().unwrap().log_error(&format!("[ON_SELECT_NEW_FILE] Error in loading new configuration: {}", e));
            }
        }


        

        if !failed{
            if let Ok(ref mut c)= *network_initializer_.lock().unwrap() {
                *sc_receiver_.lock().unwrap() = Some((*c).get_controller_recv());
                *senders.lock().unwrap() = Some((*c).get_controller_senders());
                *channels_.lock().unwrap() = Some((*c).get_channels());
                id_to_type_pos_.lock().unwrap().clear();

                let nodes = c.get_nodes();

                let mut edges: Vec<Edge> = vec![];
                // let clients = populate_clients(&nodes.1, &mut edges, &nodes.0, &id_to_type_);
                // let drones = populate_drones(&nodes.0, &mut edges, &id_to_type_);
                // let servers = populate_servers(&nodes.2, &mut edges, &nodes.0, &id_to_type_);
                let (drones, clients, servers) = populate_all(nodes.0, nodes.2, nodes.1, &mut edges, &id_to_type_pos_);
                println!("id_to_type {:?}", *id_to_type_pos_.lock().unwrap());

                if let Some(window) = weak.upgrade() {
                    window.set_edges(slint::ModelRc::new(slint::VecModel::from(edges)));
                    window.set_clients(slint::ModelRc::new(slint::VecModel::from(clients)));
                    window.set_drones(slint::ModelRc::new(slint::VecModel::from(drones)));
                    window.set_servers(slint::ModelRc::new(slint::VecModel::from(servers)));
                    window.set_messages(slint::ModelRc::new(slint::VecModel::from(vec![])));
                }
                
            }

            // run configuration
            let network_initializer_run_simulation = network_initializer_.clone();
            let logger1= logger_.clone();
            thread::spawn(move || {
                if let Ok(ref mut c)= *network_initializer_run_simulation.lock().unwrap() {
                    match c.run_simulation(None, Some(vec![CLIENT_T])){
                        Ok(_)=>{
                            logger1.lock().unwrap().log_debug("[ON_SELECT_NEW_FILE] Simulation correctly ended");
                        },
                        Err(e)=>{
                            logger1.lock().unwrap().log_error(&format!("[ON_SELECT_NEW_FILE] Simulation uncorrectly ended : {}", e));
                        }
                    }
                }else{
                    println!("[SIMULATION CONTROLLER] Error getting network initializer lock in run sim");
                    logger1.lock().unwrap().log_error("[ON_SELECT_NEW_FILE] Error getting network initializer lock in run sim");
                }
            });

        }
    });


    // Set up Ctrl+C handler
    let logger_ = logger.clone();
    let weak = main_window.as_weak();
    let channels_ = channels.clone();
    let senders = sc_senders.clone();

    ctrlc::set_handler(move || {
        logger_.lock().unwrap().log_info("Received Ctrl+C, shutting down gracefully...");

        let senders1 = senders.clone();
        let channels1 = channels_.clone();
        let logger1 = logger_.clone();
        match weak.upgrade_in_event_loop(move |window|{
            let drones = window.get_drones();

            for drone in drones.iter(){
                match send_drone_command(&senders1, drone.id as u8, Box::new(DroneCommand::Crash)){
                    Ok(_) => {
                        logger1.lock().unwrap().log_debug(&format!("[cntrl+c] Command Crash to Drone {}", drone.id));
                    }
                    Err(e) => {
                        logger1.lock().unwrap().log_error(&format!("[cntrl+c] Error in sending command Crash to Drone {}: {}", drone.id, e));
                    }
                }

                for adj in drone.adjent.iter(){
                    match send_drone_command(&senders1, adj as u8, Box::new(DroneCommand::RemoveSender(drone.id as u8))){
                        Ok(_) => {
                            logger1.lock().unwrap().log_debug(&format!("[cntrl+c] Command RemoveSender to Drone {} to remove {}", drone.id, adj));
                        }
                        Err(e) => {
                            logger1.lock().unwrap().log_error(&format!("[cntrl+c] Error sending command RemoveSender to Drone {} to remove {}: {}", drone.id, adj, e));
                        }
                    }
                }

                if let Some(ref mut channel) = *channels1.lock().unwrap() {
                    channel.remove(&(drone.id as u8));
                } else {
                    logger1.lock().unwrap().log_warn("[cntrl+c] No channels map loaded");
                }
                if let Some(ref mut s) = *senders1.lock().unwrap() {
                    s.remove(&(drone.id as u8));
                } else {
                    logger1.lock().unwrap().log_warn("[cntrl+c] No sender map loaded");
                }
            }

            let clients = window.get_clients();
            for client in clients.iter(){
                match send_drone_command(&senders1, client.id as u8, Box::new(DroneCommand::Crash)){
                    Ok(_) => {
                        logger1.lock().unwrap().log_debug(&format!("[cntrl+c] Command Crash to Drone {}", client.id));
                    }
                    Err(e) => {
                        logger1.lock().unwrap().log_error(&format!("[cntrl+c] Error in sending command Crash to Drone {}: {}", client.id, e));
                    }
                }

                if let Some(ref mut channel) = *channels1.lock().unwrap() {
                    channel.remove(&(client.id as u8));
                } else {
                    logger1.lock().unwrap().log_warn("[cntrl+c] No channels map loaded");
                }
                if let Some(ref mut s) = *senders1.lock().unwrap() {
                    s.remove(&(client.id as u8));
                } else {
                    logger1.lock().unwrap().log_warn("[cntrl+c] No senders map loaded");
                }
            }


            let servers = window.get_servers();
            for server in servers.iter(){
                match send_drone_command(&senders1, server.id as u8, Box::new(DroneCommand::Crash)){
                    Ok(_) => {
                        logger1.lock().unwrap().log_debug(&format!("[cntrl+c] Command Crash to Server {}", server.id));
                    }
                    Err(e) => {
                        logger1.lock().unwrap().log_error(&format!("[cntrl+c] Error in sending command Crash to Server {}: {}", server.id, e));
                    }
                }

                if let Some(ref mut channel) = *channels1.lock().unwrap() {
                    channel.remove(&(server.id as u8));
                } else {
                    logger1.lock().unwrap().log_warn("[cntrl+c] No channels map loaded");
                }
                if let Some(ref mut s) = *senders1.lock().unwrap() {
                    s.remove(&(server.id as u8));
                } else {
                    logger1.lock().unwrap().log_warn("[cntrl+c] No senders map loaded");
                }
            }

        }){
            Ok(_)=>{
                logger_.lock().unwrap().log_debug("[cntrl+c] Shutting down gracefully...");
            },
            Err(x)=>{
               logger_.lock().unwrap().log_error(&format!("[cntrl+c] Error shutting down... {}",x));
            }
        }

        std::process::exit(0);

    }).expect("Error setting Ctrl+C handler");


    let _res = main_window.run();

    Ok(())
}



// TODO (annina):
// - sarebbbe figo avere controlli network partitions (non necesssario)
// - volendo fare link basati su pdr