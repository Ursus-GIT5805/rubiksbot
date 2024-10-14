/*
rpi credentials
rubiksbot
godsnumberis20
*/

mod camera;
mod gpio;
mod motor;

use rubikscube::prelude::*;

use camera::*;
use gpio::*;
use motor::StepperController;

use opencv as cv;
use std::time::Duration;

const DATA_PATH: &str = "kociemba.dat";

const CAMID1: i32 = 0;
const CAMROT1: i32 = cv::core::ROTATE_90_CLOCKWISE;

const CAMID2: i32 = 2;
const CAMROT2: i32 = cv::core::ROTATE_90_COUNTERCLOCKWISE;

/// Final code that runs on the Raspberry Pi
#[cfg(not(debug_assertions))]
fn main() {
	println!("Init LED handler...");
	let mut leds = LEDHandler::try_new().expect("Could not init LEDs");

	leds.set_led_msg(LEDMessage::Wait);

	println!("Init stepper motor PCA...");
	let mut steppers = StepperController::try_new()
		.map_err(|e| {
			leds.set_led_msg(LEDMessage::InternalError);
			e
		})
		.expect("Could not init stepper motors");

	println!("Init GPIO input handler");
	let mut input = InputHandler::try_new()
		.map_err(|e| {
			leds.set_led_msg(LEDMessage::InternalError);
			e
		})
		.expect("Could not init input!");

	println!("Init camera 1");
	let mut cam1 = Camera::try_new(CAMID1, CAMROT1)
		.map_err(|e| {
			leds.set_led_msg(LEDMessage::InternalError);
			e
		})
		.expect("Could not init camera 1");

	println!("Init camera 2");
	let mut cam2 = Camera::try_new(CAMID2, CAMROT2)
		.map_err(|e| {
			leds.set_led_msg(LEDMessage::InternalError);
			e
		})
		.expect("Could not init camera 2");

	println!("Load solver data...");
	let data = match KociembaData::load(DATA_PATH) {
		Ok(d) => d,
		Err(e) => {
			eprintln!("Could not load data! {}", e);
			println!("Generate solver data, this may take a while...");
			let data = KociembaData::generate(true);
			println!("Generated!");
			if let Err(e) = data.save(DATA_PATH) {
				eprintln!("Could not save data: {}", e);
			}
			data
		}
	};
	println!("Init solver...");
	let mut solver = Solver::new(data);
	println!("Init Calibrationdata...");
	let mut data = CalibrationData::default();

	println!("Finished! Start checking for cube!");

	loop {
		let lid_open = input.lid_open();
		if lid_open {
			leds.set_led_msg(LEDMessage::LidOpen);
		} else {
			leds.set_led_msg(LEDMessage::Ok);
		}

		if input.just_pressed() {
			leds.set_led_msg(LEDMessage::Wait);
			if let Err(e) = calibrate(&mut cam1, &mut cam2, &mut data) {
				eprintln!("Could not calibrate: {}", e);
				leds.set_led_msg_for_duration(LEDMessage::InternalError, Duration::from_secs(3));
			}
			leds.set_led_msg(LEDMessage::Ok);
		}

		let grab1 = cam1.grab();
		let grab2 = cam2.grab();

		if grab1 && grab2 && !lid_open {
			let fr1 = cam1.retrieve();
			let fr2 = cam2.retrieve();

			if fr1.is_ok() && fr2.is_ok() {
				// Both are OK values, unwrap should never panic
				let mut f1 = fr1.unwrap();
				let mut f2 = fr2.unwrap();

				if let Ok(Some(cube)) = read_cube(&mut f1, &mut f2, data.clone()) {
					if !cube.is_solved() {
						leds.set_led_msg(LEDMessage::Wait);
						std::thread::sleep(std::time::Duration::from_millis(500));
						leds.quit_message();

						if let Some(turns) = solver.solve(cube) {
							for turn in turns {
								if let Err(e) = steppers.execute_turn(turn) {
									eprintln!("Error during turning: {}", e);
									eprintln!("Stop turning!");
									break;
								}
							}
							println!("Done!, sleep for 500ms!");
							std::thread::sleep(std::time::Duration::from_millis(500));
						}
					}
				}
			}
		}
	}
}

#[cfg(debug_assertions)]
fn main() -> Result<(), Box<dyn std::error::Error>> {
	camera_testing()
}
