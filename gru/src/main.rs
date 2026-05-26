mod app;
mod protocol;
mod ui;

use crate::app::App;
use crate::protocol::{HEADER, PACKET_SIZE, Packet};

use serialport::SerialPort;
use std::{
    env,
    io::{self, Read},
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
    },
    thread,
    time::Duration,
};

fn main() -> io::Result<()> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        eprintln!("Usage: {} <port> <baud>", args[0]);
        eprintln!("Example: {} /dev/ttyUSB0 9600", args[0]);
        std::process::exit(1);
    }

    let buffer: Arc<Mutex<Vec<f32>>> = Arc::new(Mutex::new(Vec::new()));
    let buffer_clone = Arc::clone(&buffer);
    let last_packet: Arc<Mutex<Option<Packet>>> = Arc::new(Mutex::new(None));
    let last_packet_clone = Arc::clone(&last_packet);
    let running = Arc::new(AtomicBool::new(true));
    let running_clone = Arc::clone(&running);

    let handle = thread::spawn(move || {
        let port_name = &args[1];
        let baud: u32 = args[2].parse().expect("Invalid baud rate");
        let mut port = serialport::new(port_name, baud)
            .timeout(Duration::from_millis(500))
            .open()
            .unwrap_or_else(|e| {
                eprintln!("Failed to open {port_name}: {e}");
                std::process::exit(1);
            });

        let mut buf = [0u8; PACKET_SIZE];
        while running_clone.load(Ordering::Relaxed) {
            if let Err(e) = sync_to_header(&mut port) {
                eprintln!("Sync error: {e}");
                continue;
            }

            buf[0] = HEADER;
            if let Err(e) = read_exact(&mut port, &mut buf[1..]) {
                eprintln!("Read error: {e}");
                continue;
            }

            match Packet::from_bytes(&buf[..]) {
                Ok(pkt) => {
                    let mut buf = buffer_clone.lock().unwrap();
                    buf.push(pkt.altitude);
                    if buf.len() > 150 {
                        buf.remove(0);
                    }
                    *last_packet_clone.lock().unwrap() = Some(pkt);
                }
                Err(e) => eprintln!("Parse error: {e:?}"),
            }
        }
    });

    let terminal = ratatui::init();
    let result = App::build(buffer, last_packet).run(terminal);
    ratatui::restore();
    running.store(false, Ordering::Relaxed);
    handle.join().ok();
    result
}

fn sync_to_header(port: &mut Box<dyn SerialPort>) -> io::Result<()> {
    let mut byte = [0u8; 1];
    loop {
        port.read_exact(&mut byte)?;
        if byte[0] == HEADER {
            return Ok(());
        }
    }
}

fn read_exact(port: &mut Box<dyn SerialPort>, buf: &mut [u8]) -> io::Result<()> {
    let mut total = 0;
    while total < buf.len() {
        match port.read(&mut buf[total..]) {
            Ok(0) => return Err(io::Error::new(io::ErrorKind::UnexpectedEof, "port closed")),
            Ok(n) => total += n,
            Err(e) if e.kind() == io::ErrorKind::TimedOut => continue,
            Err(e) => return Err(e),
        }
    }
    Ok(())
}
