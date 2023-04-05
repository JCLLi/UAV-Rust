use std::{thread::sleep, time::Duration};
use protocol::Datalog;
use eframe::egui;
use egui::plot::{Line, Plot, PlotPoints};

use super::settings_logic::SettingsBundle;

pub struct QuadrupelGUI {
    rx_gui_pc_command: single_value_channel::Receiver<Option<SettingsBundle>>,
    rx_gui_datalog: single_value_channel::Receiver<Option<Datalog>>,
    settings: SettingsBundle,
    datalog: Datalog,
    battery_vec: Vec<f64>,
    pressure_vec: Vec<f64>,
}

impl QuadrupelGUI {
    pub fn new(cc: &eframe::CreationContext<'_>, rx_gui_pc_command: single_value_channel::Receiver<Option<SettingsBundle>>, rx_gui_datalog: single_value_channel::Receiver<Option<Datalog>>) -> Self {
        Self { 
            rx_gui_pc_command: rx_gui_pc_command,
            rx_gui_datalog: rx_gui_datalog,
            settings: SettingsBundle::default(),
            datalog: Datalog::new(),
            battery_vec: vec![0.0; 100],
            pressure_vec: vec![0.0; 100],
        }
    }
}

impl eframe::App for QuadrupelGUI {
   fn update(&mut self, ctx: &egui::Context, frame: &mut eframe::Frame) {

        // GUI always on top
        frame.set_always_on_top(true);

        // Receive data
        let settings = *self.rx_gui_pc_command.latest();
        match settings {
            Some(settings) => {
                self.settings = settings;
            }
            None => ()
        }

        let datalog = *self.rx_gui_datalog.latest();
        match datalog {
            Some(datalog) => {
                self.datalog = datalog;
            }
            None => ()
        }

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.visuals_mut().override_text_color = Some(egui::Color32::from_rgb(255, 255, 255));

            ui.horizontal(|ui| {
                ui.allocate_ui_with_layout(eframe::egui::vec2(8000.0, 8000.0), egui::Layout::top_down(egui::Align::LEFT), |ui| {

                    ui.allocate_ui_with_layout(eframe::egui::vec2(8000.0, 8000.0), egui::Layout::left_to_right(egui::Align::LEFT), |ui| {

                        // Command to drone
                        ui.allocate_ui_with_layout(eframe::egui::vec2(8000.0, 8000.0), egui::Layout::top_down(egui::Align::LEFT), |ui| {
                        ui.heading("Command to drone                                          ");
                        ui.visuals_mut().override_text_color = Some(egui::Color32::from_rgb(200, 200, 200));

                        ui.label("Pitch:".to_string() + "    " + self.settings.pitch.to_string().as_str());
                        ui.label("Roll:".to_string() + "      " +  self.settings.roll.to_string().as_str());
                        ui.label("Yaw:".to_string() + "      " +  self.settings.yaw.to_string().as_str());
                        ui.label("Lift:".to_string() + "       " +  self.settings.lift.to_string().as_str());
                        ui.label("Mode:".to_string() + "   " +  self.settings.mode.to_string().as_str());
                        ui.label("Yaw P:".to_string() + "  " +  self.settings.yaw_control_p.to_string().as_str());
                        ui.label("R/P P1:".to_string() + " " +  self.settings.roll_pitch_control_p1.to_string().as_str());
                        ui.label("R/P P2:".to_string() + " " +  self.settings.roll_pitch_control_p2.to_string().as_str()); 
                        ui.heading("");
                        });

                        // Drone datalog
                        ui.allocate_ui_with_layout(eframe::egui::vec2(8000.0, 8000.0), egui::Layout::top_down(egui::Align::LEFT), |ui| {
                            ui.heading("Drone datalog".to_string() + "             ");
                            ui.visuals_mut().override_text_color = Some(egui::Color32::from_rgb(200, 200, 200));
                            ui.label("Mode:   ".to_string() + self.datalog.workingmode.to_string().as_str());
                            ui.label("Motors: ".to_string() + self.datalog.motor1.to_string().as_str() + ", " + self.datalog.motor2.to_string().as_str() + ", " + self.datalog.motor3.to_string().as_str() + ", " + self.datalog.motor4.to_string().as_str());
                            ui.label("YPR: ".to_string() + self.datalog.yaw.to_string().as_str() + ", " + self.datalog.pitch.to_string().as_str() + ", " + self.datalog.roll.to_string().as_str());
                            ui.label("ACC: ".to_string() + self.datalog.yaw_r.to_string().as_str() + ", " + self.datalog.pitch_r.to_string().as_str() + ", " + self.datalog.roll_r.to_string().as_str());
                            ui.label("Bat: ".to_string() + self.datalog.bat.to_string().as_str() + " mV");
                            ui.label("Pressure: ".to_string() + self.datalog.bar.to_string().as_str() + " 10^-5 bar");        
                            ui.label("Looptime: ".to_string() + self.datalog.control_loop_time.to_string().as_str() + " us");        
                        });

                    });

                    // Graphs
                    ui.allocate_ui_with_layout(eframe::egui::vec2(8000.0, 8000.0), egui::Layout::left_to_right(egui::Align::LEFT), |ui| {
                        ui.allocate_ui_with_layout(eframe::egui::vec2(8000.0, 8000.0), egui::Layout::top_down(egui::Align::LEFT), |ui| {
                            ui.heading("Battery graph");
                            self.battery_vec.rotate_left(1);
                            self.battery_vec[99] = self.datalog.bat as f64;
                            let points: PlotPoints = self.battery_vec.iter().enumerate().map(|(i, &y)| [i as f64, y]).collect();
                            let line = Line::new(points);
                            Plot::new("battery2")
                            .allow_drag(false)
                            .view_aspect(2.0)
                            .width(200.0)
                            .show_x(false)
                            .show_y(false)
                            .show_background(false)
                            .allow_scroll(false)
                            .show(ui, |plot_ui| plot_ui.line(line));
                        });
                        ui.heading("    ");

                        ui.allocate_ui_with_layout(eframe::egui::vec2(8000.0, 8000.0), egui::Layout::top_down(egui::Align::LEFT), |ui| {
                            ui.heading("Pressure graph");
                            self.pressure_vec.rotate_left(1);
                            self.pressure_vec[99] = self.datalog.bar as f64;
                            let points: PlotPoints = self.pressure_vec.iter().enumerate().map(|(i, &y)| [i as f64, y]).collect();
                            let line = Line::new(points);
                            Plot::new("pressure2")
                                .allow_drag(false)
                                .view_aspect(2.0)
                                .width(200.0)
                                .show_x(false)
                                .show_y(false)
                                .show_background(false)
                                .allow_scroll(false)
                                .show(ui, |plot_ui| plot_ui.line(line));
                        });
                    });

                    ui.heading("    ");
                    ui.heading("Motor display");
                    ui.label("           ".to_string() + self.datalog.motor1.to_string().as_str());
                    ui.label("");
                    ui.label(self.datalog.motor4.to_string() + "      o      " + self.datalog.motor2.to_string().as_str());
                    ui.label("");
                    ui.label("           ".to_string() + self.datalog.motor3.to_string().as_str());
                });
            });
            
        sleep(Duration::from_millis(15));

        // Close GUI
        if self.settings.exit == true {
            frame.close();
        }

        // Update GUI
        ctx.request_repaint();
       });
   }
}
