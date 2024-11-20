slint::include_modules!();
use rfd::FileDialog;
use network_initializer::{NetworkInitializer};
// use slint::Model;


fn main() -> Result<(), slint::PlatformError> {

    let main_window = MainWindow::new()?;

    main_window.on_clicked(move || {
        println!("CIAO");
    });

    main_window.on_select_file(move || {
        println!("[] FILE SELECTION");
        let file = FileDialog::new().pick_file();
        if let Some(mut path)= file {
            println!("File selected: {:?}", path.as_mut_os_str());
            // INTEGRARE CODICE NEWTWORK INITIALIZER

            // // TO IMPLEMENT the spawn of the drones
            // let mut network = NetworkInitializer::new(Some(path));
            // let config = NetworkInitializer::new(Some(path));
            // assert!(config.is_ok(), "{}", config.err().unwrap());
            // let config = config.unwrap();
            // // println!("{config:#?}");
            // config.run_simulation();


        }else{
            println!("No file selected\nTry again");
        }
    });

    main_window.run()
}