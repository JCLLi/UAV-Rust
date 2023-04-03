
use std::{thread::{self, sleep}, time::{SystemTime, Duration}};
use rand::Rng;
use eframe::{egui, emath::Align};
use egui::plot::{Line, Plot, PlotPoints};

#[derive(Default)]
pub struct MyEguiApp {
    time: u32,
}

impl MyEguiApp {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        // Customize egui here with cc.egui_ctx.set_fonts and cc.egui_ctx.set_visuals.
        // Restore app state using cc.storage (requires the "persistence" feature).
        // Use the cc.gl (a glow::Context) to create graphics shaders and buffers that you can use
        // for e.g. egui::PaintCallback.
        // Self::default()
        
        Self { 
            time: 2 
        }
        
    }
}

impl eframe::App for MyEguiApp {
   fn update(&mut self, ctx: &egui::Context, frame: &mut eframe::Frame) {
       egui::CentralPanel::default().show(ctx, |ui| {

            ui.horizontal(|ui| {
                // ui.heading("Command to drone");

                // ui.vertical_centered(|ui| {
                //     ui.heading("Datalog");
                // });

                // ui.with_layout(egui::Layout::top_down(egui::Align::Center), |ui| {
                //     ui.heading("Drone datalog");
                //     ui.heading("Motor 1");
                // });

                // ui.with_layout(egui::Layout::top_down(egui::Align::RIGHT), |ui| {
                //     ui.heading("Motor values");
                //     ui.heading("Motor 1");
                // });

                ui.allocate_ui_with_layout(eframe::egui::vec2(500.0, 100.0), egui::Layout::top_down(egui::Align::LEFT), |ui| {
                    ui.heading("Command to drone");
                    let now = SystemTime::now();
                    sleep(Duration::from_millis(rand::thread_rng().gen_range(0..100)));
                    let time = now.elapsed().unwrap().as_millis();
                    ui.label(time.to_string());
                    ui.label(self.time.to_string());
                });

                ui.allocate_ui_with_layout(eframe::egui::vec2(500.0, 100.0), egui::Layout::top_down(egui::Align::Center), |ui| {
                    ui.heading("Drone datalog");
                    ui.heading("Motor 1");
                });

                ui.allocate_ui_with_layout(eframe::egui::vec2(200.0, 100.0), egui::Layout::top_down(egui::Align::RIGHT), |ui| {
                    ui.heading("Motor values");
                    ui.heading("Motor 1");
                });

            });
            
           
        //    let sin: PlotPoints = (0..1000).map(|i| {
        //         let x = i as f64 * 0.01;
        //         [x, x.sin()]
        //     }).collect();
        //     let line = Line::new(sin);
        //     Plot::new("my_plot")
        //     .allow_drag(false)
        //     .view_aspect(2.0)
        //     .width(300.0)
        //     .show_x(false)
        //     .show_y(false)
        //     .show_background(false)
        //     .allow_scroll(false)
        //     .show(ui, |plot_ui| plot_ui.line(line));

        //     let sin: PlotPoints = (0..1000).map(|i| {
        //         let x = i as f64 * 0.01;
        //         [x, x.sin()]
        //     }).collect();
        //     let line = Line::new(sin);
        //     Plot::new("my_plot")
        //     .allow_drag(false)
        //     .view_aspect(2.0)
        //     .width(300.0)
        //     .show_x(false)
        //     .show_y(false)
        //     .show_background(false)
        //     .allow_scroll(false)
        //     .show(ui, |plot_ui| plot_ui.line(line));
        sleep(Duration::from_millis(1000));
        println!("update done");
        ctx.request_repaint();
       });
   }
}