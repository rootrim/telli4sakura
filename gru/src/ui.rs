use crate::app::App;
use ratatui::{
    Frame,
    layout::{Constraint, Direction, Layout},
    prelude::Stylize,
    style::{Color, Style},
    symbols::Marker,
    text::{Line, Span},
    widgets::{Axis, Block, Borders, Chart, Dataset, GraphType, Paragraph},
};

impl App {
    pub(crate) fn render(&mut self, frame: &mut Frame) {
        let layout = Layout::default()
            .direction(Direction::Vertical)
            .margin(1)
            .constraints(vec![
                Constraint::Max(1),
                Constraint::Fill(1),
                Constraint::Max(3),
            ])
            .split(frame.area());

        let locked_buffer = self.buffer.lock().unwrap();
        if locked_buffer.is_empty() {
            return;
        }

        let (_min, max) = locked_buffer
            .iter()
            .fold((f32::MAX, f32::MIN), |(min, max), &v| {
                (min.min(v), max.max(v))
            });

        let points: Vec<(f64, f64)> = locked_buffer
            .iter()
            .enumerate()
            .map(|(i, &alt)| (i as f64, alt as f64))
            .collect();

        let dataset = Dataset::default()
            .name("Altitude")
            .marker(Marker::Braille)
            .graph_type(GraphType::Line)
            .style(Style::default().fg(Color::Blue))
            .data(&points);

        let x_axis = Axis::default()
            .title("Time".blue())
            .bounds([0.0, points.len() as f64])
            .labels(["0s", "15s", "30s"]);

        let y_axis = Axis::default()
            .title("Altitude".blue())
            .bounds([0.0, max as f64])
            .labels(vec![
                Span::from("0"),
                Span::from(format!("{:.0}", max / 2.0)),
                Span::from(format!("{:.0}", max)),
            ]);

        let chart = Chart::new(vec![dataset]).x_axis(x_axis).y_axis(y_axis);

        let title = Line::from_iter([
            Span::from("Telli IV: ").bold(),
            Span::styled("Sakura ", Style::default().fg(Color::Magenta)).bold(),
            Span::from("Altitude Chart").bold(),
            Span::from(" (Press 'q' to quit)"),
        ]);
        frame.render_widget(title.centered(), layout[0]);

        frame.render_widget(chart, layout[1]);

        let telemetry_block = Block::default()
            .borders(Borders::ALL)
            .title("Last Telemetry Packet");

        let packet = self.last_packet.lock().unwrap();
        if let Some(packet) = *packet {
            frame.render_widget(
                Paragraph::new(format!(
                    "alt:{:.2}m  press:{:.2}Pa ax:{:.3} ay:{:.3} az:{:.3}  gx:{:.3} gy:{:.3} gz:{:.3}  lat:{:.6} lon:{:.6}",
                    packet.altitude,
                    packet.pressure,
                    packet.accel_x,
                    packet.accel_y,
                    packet.accel_z,
                    packet.angle_x,
                    packet.angle_y,
                    packet.angle_z,
                    packet.gps_lat,
                    packet.gps_lon,
                )).block(telemetry_block) ,
                layout[2],
            );
        }
    }
}
