slint::include_modules!();
use rfd::FileDialog;
use slint::Window;
use std::{cell::{RefCell, RefMut}, ffi::OsString, rc::Rc};
use network_initializer::{NetworkInitializer};
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
                // config.run_simulation();

                
                if let Some(window) = weak.upgrade() {

                    let from_network_initializer = config.get_nodes();
                    let mut edges_dd : Vec<Edge> = vec![];
                    let mut edges : Vec<Edge> = vec![];

                    let mut drones : Vec<Drone> = vec![];
                    for drone in from_network_initializer.0 {
                        let mut adjent = vec![];
                        for adj in &drone.connected_drone_ids {
                            adjent.push(*adj as i32);
                            if !check_edges(&edges_dd, drone.id as i32, *adj as i32) {
                                if drone.id<20 && *adj<20 {
                                    edges_dd.push(Edge{id1:drone.id as i32, id2:*adj as i32});
                                }
                            }
                        }
                        drones.push(Drone{adjent:slint::ModelRc::new(slint::VecModel::from(adjent)), crashed: false, id:drone.id as i32, pdr:drone.pdr});
                    }
                    window.set_drones(slint::ModelRc::new(slint::VecModel::from(drones)));


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

                    window.set_edges_dd(slint::ModelRc::new(slint::VecModel::from(edges_dd)));
                }
        
            }else{
                println!("Error converting path to string");
            }
        }else{
            println!("No file selected\nTry again");
        }
    });



    // main_window.on_clone_file(move || {
    //     // println!("CLOnE");
    //     // change_test(rc_refcell_main_window.clone().borrow_mut(), "ciao");
    // });



    main_window.run()
}
