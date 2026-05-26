use crate::protocol::Packet;
use ratatui::{
    DefaultTerminal,
    crossterm::event::{Event, KeyCode},
};
use std::{
    io,
    sync::{Arc, Mutex},
    time::Duration,
};

pub struct App {
    running: bool,
    pub buffer: Arc<Mutex<Vec<f32>>>,
    pub last_packet: Arc<Mutex<Option<Packet>>>,
}

impl App {
    pub fn build(buffer: Arc<Mutex<Vec<f32>>>, last_packet: Arc<Mutex<Option<Packet>>>) -> Self {
        Self {
            running: true,
            buffer,
            last_packet,
        }
    }

    pub fn run(mut self, mut terminal: DefaultTerminal) -> io::Result<()> {
        self.running = true;
        while self.running {
            terminal.draw(|frame| self.render(frame))?;
            if ratatui::crossterm::event::poll(Duration::from_millis(200))? {
                if let Event::Key(key) = ratatui::crossterm::event::read()? {
                    if key.code == KeyCode::Char('q') {
                        break;
                    }
                }
            }
        }

        Ok(())
    }
}
