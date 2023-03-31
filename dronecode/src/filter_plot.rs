// use protocol::Message;
// use tui::{
//     backend::CrosstermBackend,
//     layout::{Constraint, Direction, Layout},
//     symbols::Marker,
//     text::Span,
//     widgets::{Axis, Block, Borders, Chart, Dataset, Paragraph},
//     Terminal,
// };
// use crossterm::{execute, terminal, Result};
// use std::{io::stdout, time::{Duration, Instant}};
// use tudelft_quadrupel::mpu::{read_raw, structs::{Accel, Gyro}};
// mod interface;

// fn plot() -> Result<()> {
//     let backend = CrosstermBackend::new(stdout());
//     let mut terminal = Terminal::new(backend)?;
//     terminal.clear()?;

//      let mut raw_data: Vec<f32, f32> = Vec::new();
//      let mut filtered_data: Vec<f32, f32> = Vec::new();
//     // let mut data: Vec<(String, Vec<(f64, f64)>)> = vec![
//     // ("Sine Wave".to_string(), Vec::new()),
//     // ("Cosine Wave".to_string(), Vec::new()),];


//     let mut x = 0.0;

//     execute!(stdout(), terminal::EnterAlternateScreen)?;

//     loop {
//         let start = Instant::now();

//         //Generate sine wave data
//         if let Message::Datalogging(d) = packet.message {
//         let raw_d = d_pitch_r;
//         raw_data.push((x, raw_d));
//         let filtered_d = d.pitch_f;
//         filtered_data.push((x, filtered_d));
//         }
// //         let sine_y = (x * PI / 10.0).sin();
// // let cosine_y = (x * PI / 10.0).cos();
// // data[0].1.push((x, sine_y));
// // data[1].1.push((x, cosine_y));

//         x += 0.1;

//         // Truncate data to last 100 points
//         if raw_data.len() > 100 {
//             raw_data.remove(0);
//         }
//          if filtered_data.len() > 100 {
//             filtered_data.remove(0);
//         }

//         // Draw plot
//         terminal.draw(|f| {
//             let size = f.size();
//             let constraints = [
//                 Constraint::Percentage(70),
//                 Constraint::Percentage(30),
//             ];
//             let slices = Layout::default()
//                 .direction(Direction::Vertical)
//                 .constraints(constraints)
//                 .split(size);

//             let chart = Chart::new(
//                 vec![
//                     Dataset::default()
//                     .marker(Marker::Braille)
//                     .style(tui::style::Style::default().fg(tui::style::Color::Yellow))
//                     .data(&sine_data),

//                     Dataset::default()
//                     .marker(Marker::Braille)
//                     .style(tui::style::Style::default().fg(tui::style::Color::Green))
//                     .data(&cosine_data),
//                 ]
//         //             Dataset::default()
//         //     .name(&data[0].0)
//         //     .marker(Marker::Braille)
//         //     .style(tui::style::Style::default().fg(tui::style::Color::Yellow))
//         //     .data(&data[0].1),
//         // Dataset::default()
//         //     .name(&data[1].0)
//         //     .marker(Marker::Braille)
//         //     .style(tui::style::Style::default().fg(tui::style::Color::Cyan))
//         //     .data(&data[1].1),
//         //             ],
//             )
//             .block(
//                 Block::default()
//                     .title(Span::styled(
//                         "Raw and Filtered data",
//                         tui::style::Style::default()
//                         .fg(tui::style::Color::White),
//                     ))
//                     .borders(Borders::ALL)
//                     .border_style(tui::style::Style::default()
//                     .fg(tui::style::Color::Yellow)),
//             )
//             .x_axis(
//                 Axis::default()
//                     .bounds([x - 10.0, x])
//                     .title("X Axis")
//                     .style(tui::style::Style::default().fg(tui::style::Color::White)),
//             )
//             .y_axis(
//                 Axis::default()
//                     .bounds([-2.0, 2.0])
//                     .title("Y Axis")
//                     .style(tui::style::Style::default().fg(tui::style::Color::White)),
//             );

//             f.render_widget(chart, slices[0]);

//             // let sine_y = data[0].1.last().unwrap().1;
//             // let cosine_y = data[1].1.last().unwrap().1;
//             // let text = format!(
//             //     "x = {:.2}, y (sine) = {:.2}, y (cosine) = {:.2}",
//             //     x, sine_y, cosine_y
//             // );
            
//             let text = format!("x = {:.2}, raw data = {:.2}, filtered data = {:.2}", x, raw_d, filtered_d);
//             let text_widget = Paragraph::new(text).block(
//                 Block::default()
//                     .title(Span::styled(
//                         "Current Data Point",
//                         tui::style::Style::default().fg(tui::style::Color::Green),
//                     ))
//                     .borders(Borders::ALL),
//             );
//             f.render_widget(text_widget, slices[1]);
//         })?;

//         // Wait for next frame
//         let elapsed = start.elapsed();
//         if elapsed < Duration::from_millis(20) {
//             std::thread::sleep(Duration::from_millis(20) - elapsed);
//         }
//     }
// }


// #[cfg(test)]
// mod tests{
//     use super::*;
//     mod interface;
//     #[test]
//     fn test_plot(){
//         let result = plot();
//         println!("{}", result);
//     }
// }