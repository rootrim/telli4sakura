mod protocol;

use protocol::{HEADER, PACKET_SIZE, Packet, ParseError};
use serialport::SerialPort;
use std::{
    env,
    io::{self, Read},
    time::Duration,
};

fn main() -> io::Result<()> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        eprintln!("Usage: {} <port> <baud>", args[0]);
        eprintln!("Example: {} /dev/ttyUSB0 9600", args[0]);
        std::process::exit(1);
    }

    let port_name = &args[1];
    let baud: u32 = args[2].parse().expect("Invalid baud rate");

    let mut port = serialport::new(port_name, baud)
        .timeout(Duration::from_millis(500))
        .open()
        .unwrap_or_else(|e| {
            eprintln!("Failed to open {port_name}: {e}");
            std::process::exit(1);
        });

    println!("GRU connected: {port_name} @ {baud} baud");

    let mut buf = [0u8; PACKET_SIZE];

    loop {
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
            Ok(pkt) => print_packet(&pkt),
            Err(e) => eprintln!("Parse error: {e:?}"),
        }
    }
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

fn print_packet(pkt: &Packet) {
    println!(
        "alt={:.2}m  press={:.2}Pa  \
         accel=({:.3},{:.3},{:.3})g  \
         angle=({:.3},{:.3},{:.3})°  \
         gps=({:.6},{:.6})",
        pkt.altitude,
        pkt.pressure,
        pkt.accel_x,
        pkt.accel_y,
        pkt.accel_z,
        pkt.angle_x,
        pkt.angle_y,
        pkt.angle_z,
        pkt.gps_lat,
        pkt.gps_lon,
    );
}
