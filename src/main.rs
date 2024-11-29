slint::include_modules!();
use rfd::FileDialog;
use slint::{Model, Window};
use std::{cell::{RefCell, RefMut}, ffi::OsString, rc::Rc};
use network_initializer::{NetworkInitializer};
use crossbeam::channel::{unbounded, Receiver, Sender};
use wg_internal::packet::Packet;
use wg_internal::controller::{DroneCommand, NodeEvent};
use std::thread;
// use slint::Model;

fn check_edges(edges : &Vec<Edge>, id1: i32, id2: i32) -> bool {
    for edge in edges {
        if (edge.id1 == id1 && edge.id2 == id2) || (edge.id1 == id2 && edge.id2 == id1) {
            return true;
        }
    }
    return false;
}

fn main() -> Result<(), slint::PlatformError> {

    let mut main_window = MainWindow::new()?;
    let weak = main_window.as_weak();
    let weak1 = main_window.as_weak();

    let mut general_receiver: Rc<RefCell<Option<Receiver<NodeEvent>>>> = Rc::new(RefCell::new(None));
    let mut general_receiver1 = general_receiver.clone();


    // thread::spawn(move || {
    //     loop {
    //         // thread per arrivo messaggi

    //     }
    // });
    

    main_window.on_select_file(move || {
        println!("[] FILE SELECTION");
        let file = FileDialog::new().pick_file();
        if let Some(mut path)= file {
            let path_string = <OsString as Clone>::clone(&path.as_mut_os_string()).into_string();
            if let Ok(path_string) = path_string {
                println!("Selected file: {}", path_string);

                let mut network = NetworkInitializer::new(Some(path_string.as_str())); // da cambiare con set_path
                let config = NetworkInitializer::new(Some(&path_string));
                assert!(config.is_ok(), "{}", config.err().unwrap());
                let config = config.unwrap();
                println!("{config:#?}");

                *general_receiver1.borrow_mut() = Some(config.get_controller_recv());
                // config.run_simulation();

                
                if let Some(window) = weak.upgrade() {

                    let from_network_initializer = config.get_nodes();
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
        
            }else{
                println!("Error converting path to string");
            }
        }else{
            println!("No file selected\nTry again");
        }
    });



    main_window.on_remove_edges(move || {
        println!("[] REMOVE EDGES");
        if let Some(window) = weak1.upgrade() {
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
