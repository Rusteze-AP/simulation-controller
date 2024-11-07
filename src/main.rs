use crossbeam::channel::{self, Sender, Receiver};
use std::thread;
use std::io;

#[derive(Debug)]
struct Message{
    // text: String,
    from : From,
}

// proposta di messaggio
#[derive(Debug)]
enum From{
    Internal(InternalEvent),
    External(ExternalEvent),
}

#[derive(Debug)]
enum ExternalEvent{
    ToCrash(u64), // u64 = NodeId
    //... -> da aggiungere tutti gli altri che ci sono nel protocollo appena sono sistemati (ora sono un po sus)
}

#[derive(Debug)]
enum InternalEvent{
    Topology(u64, Vec<u64>, String), // esempio (come sopra) 
}

// TODO: everything will be executed on a thread meanwhile I will test it without it :)
fn main() {

    // channel where the simulation controller can receive messages --> non so se debba essere fatto qua questo canale o me lo passi qualcuno
    let (tx, rx) : (Sender<Message>, Receiver<Message>) = channel::unbounded(); // questo dipenderà dal protocollo 

    // thread per ricevere input da command-line
    thread::spawn(move || {
        let mut input = String::from("");
        loop{
            input = String::from("");
            io::stdin().read_line(&mut input).expect("Failed to read line");
            input = String::from(input.trim());
            
            // TODO: parse the input and
            // - option 1 -> call the handler from there
            // - option 2 -> craft a message and send it to the thread handling messages from the nodes 
            // let _ = tx.send(Message{text:input.clone()}); // op2
        }
    });

    // loop that calls the handler -> si può fare un thread se preferite ma secondo me non è necessario perchè è quello che deve fare
    loop{
        for rec in rx.iter(){
            // call the handler based on the message
        }
    }



}
