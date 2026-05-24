pub const PACKET_SIZE: usize = 44;
pub const HEADER: u8 = 0xAB;
pub const FOOTER_1: u8 = 0x0D;
pub const FOOTER_2: u8 = 0x0A;

#[derive(Debug, Clone, Copy)]
pub struct Packet {
    pub altitude: f32,
    pub pressure: f32,
    pub accel_x: f32,
    pub accel_y: f32,
    pub accel_z: f32,
    pub angle_x: f32,
    pub angle_y: f32,
    pub angle_z: f32,
    pub gps_lat: f32,
    pub gps_lon: f32,
}

#[derive(Debug)]
pub enum ParseError {
    InvalidLength,
    BadHeader,
    BadFooter,
    ChecksumMismatch,
}

impl Packet {
    pub fn from_bytes(buf: &[u8]) -> Result<Self, ParseError> {
        if buf.len() != PACKET_SIZE {
            return Err(ParseError::InvalidLength);
        }
        if buf[0] != HEADER {
            return Err(ParseError::BadHeader);
        }
        if buf[42] != FOOTER_1 || buf[43] != FOOTER_2 {
            return Err(ParseError::BadFooter);
        }

        let expected: u8 = buf[..41].iter().fold(0u8, |acc, &b| acc.wrapping_add(b));
        if expected != buf[41] {
            return Err(ParseError::ChecksumMismatch);
        }

        Ok(Self {
            altitude: f32::from_le_bytes(buf[1..5].try_into().unwrap()),
            pressure: f32::from_le_bytes(buf[5..9].try_into().unwrap()),
            accel_x: f32::from_le_bytes(buf[9..13].try_into().unwrap()),
            accel_y: f32::from_le_bytes(buf[13..17].try_into().unwrap()),
            accel_z: f32::from_le_bytes(buf[17..21].try_into().unwrap()),
            angle_x: f32::from_le_bytes(buf[21..25].try_into().unwrap()),
            angle_y: f32::from_le_bytes(buf[25..29].try_into().unwrap()),
            angle_z: f32::from_le_bytes(buf[29..33].try_into().unwrap()),
            gps_lat: f32::from_le_bytes(buf[33..37].try_into().unwrap()),
            gps_lon: f32::from_le_bytes(buf[37..41].try_into().unwrap()),
        })
    }
}
