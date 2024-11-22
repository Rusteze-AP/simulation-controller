slint::include_modules!();
use rfd::FileDialog;
use std::ffi::OsString;
use network_initializer::{NetworkInitializer};
// use slint::Model;


fn main() -> Result<(), slint::PlatformError> {

    let main_window = MainWindow::new()?;

    main_window.on_open_right_window(move || {
        println!("[] OPEN RIGHT WINDOW");
        // TODO -> spawn the rigth window

        // let right_menu : RightMenu = RightMenu::new();
        // right_menu.run(); // Show as a separate window

    });

    main_window.on_select_file(move || {
        println!("[] FILE SELECTION");
        let file = FileDialog::new().pick_file();
        if let Some(mut path)= file {
            let path_string = <OsString as Clone>::clone(&path.as_mut_os_string()).into_string();
            if let Ok(path_string) = path_string {
                println!("Selected file: {}", path_string);
                let mut network = NetworkInitializer::new(Some(path_string.as_str()));
                let config = NetworkInitializer::new(Some(&path_string));
                // assert!(config.is_ok(), "{}", config.err().unwrap());
                // let config = config.unwrap();
                // println!("{config:#?}");
                // config.run_simulation();
            }else{
                println!("Error converting path to string");
            }
        }else{
            println!("No file selected\nTry again");
        }
    });

    main_window.run()
}