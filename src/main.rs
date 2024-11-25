slint::include_modules!();
use rfd::FileDialog;
use slint::Window;
use std::{cell::{RefCell, RefMut}, ffi::OsString, rc::Rc};
use network_initializer::{NetworkInitializer};
// use slint::Model;


fn main() -> Result<(), slint::PlatformError> {

    let mut main_window = MainWindow::new()?;
    // let mut rc_refcell_main_window = Rc::new(RefCell::new(main_window));
    let weak = main_window.as_weak();

    main_window.on_select_file(move || {
        println!("[] FILE SELECTION");
        let file = FileDialog::new().pick_file();
        if let Some(mut path)= file {
            let path_string = <OsString as Clone>::clone(&path.as_mut_os_string()).into_string();
            if let Ok(path_string) = path_string {
                println!("Selected file: {}", path_string);
                let mut network = NetworkInitializer::new(Some(path_string.as_str()));
                let config = NetworkInitializer::new(Some(&path_string));
                assert!(config.is_ok(), "{}", config.err().unwrap());
                let config = config.unwrap();
                println!("{config:#?}");
                // config.run_simulation();

                // change_test(&mut rc_refcell_main_window.clone(), "ciao");
                if let Some(window) = weak.upgrade() {
                    // examples of creating drones, client and servers -> to change with the ones from the file
                    let mut drones = vec![Drone{adjent:[1,2,30].into(), crashed: false, id:0, pdr:0.3}, Drone{adjent:[0,2].into(), crashed: false, id:1, pdr:0.5}];
                    drones.push(Drone{adjent:[0,1, 21].into(), crashed: false, id:2, pdr:0.6});
                    drones.push(Drone{adjent:[0,1,31].into(), crashed: false, id:3, pdr:0.7});
                    drones.push(Drone{adjent:[0,1,9].into(), crashed: false, id:4, pdr:0.8});
                    drones.push(Drone{adjent:[0,1,30].into(), crashed: false, id:5, pdr:0.9});
                    drones.push(Drone{adjent:[0,1,20].into(), crashed: false, id:6, pdr:0.3});
                    drones.push(Drone{adjent:[0,1,7,5].into(), crashed: false, id:7, pdr:0.2});
                    drones.push(Drone{adjent:[0,1,6,30].into(), crashed: false, id:8, pdr:0.15});
                    drones.push(Drone{adjent:[0,1,4,20].into(), crashed: false, id:9, pdr:0.87});
                    window.set_drones(slint::ModelRc::new(slint::VecModel::from(drones)));

                    window.set_clients(slint::ModelRc::new(slint::VecModel::from(vec![Drone{adjent:[6,9].into(), crashed: false, id:20, pdr:0.3}, Drone{adjent:[2].into(), crashed: false, id:21, pdr:0.5}])));
                    window.set_servers(slint::ModelRc::new(slint::VecModel::from(vec![Drone{adjent:[5,8,0].into(), crashed: false, id:30, pdr:0.3}, Drone{adjent:[3].into(), crashed: false, id:31, pdr:0.5}])));
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



    main_window.run();
    Ok(())
}

// NetworkInitializer {
//     parser: Parser {
//         drones: [
//             ParsedDrone {
//                 id: 1,
//                 connected_drone_ids: [
//                     2,
//                 ],
//                 pdr: 0.05,
//             },
//             ParsedDrone {
//                 id: 2,
//                 connected_drone_ids: [
//                     1,
//                     5,
//                 ],
//                 pdr: 0.03,
//             },
//         ],
//         clients: [
//             ParsedClient {
//                 id: 4,
//                 connected_drone_ids: [
//                     2,
//                 ],
//             },
//             ParsedClient {
//                 id: 5,
//                 connected_drone_ids: [
//                     2,
//                 ],
//             },
//         ],
//         servers: [
//             ParsedServer {
//                 id: 6,
//                 connected_drone_ids: [
//                     2,
//                 ],
//             },
//         ],
//     },
// }