/*
 * This files contains structs used for GPIO handling.
 */

use std::time::Duration;

use rppal::gpio::{Gpio, InputPin, OutputPin};

type GPIOResult<T> = rppal::gpio::Result<T>;

// GPIO pins on the RPI
const LED_RED: u8 = 17;
const LED_GREEN: u8 = 15;
const LED_BLUE: u8 = 18;

/// The different messages you can send through the LEDs
#[derive(Clone, Copy)]
pub enum LEDMessage {
	None,
	Ok,
	Wait,
	LidOpen,
	InternalError,
}

pub struct LEDHandler {
	msg: LEDMessage,

	pin_red: OutputPin,
	pin_green: OutputPin,
	pin_blue: OutputPin,
}

/// Simple LEDHandler struct
impl LEDHandler {
	pub fn try_new() -> GPIOResult<Self> {
		let gpio = Gpio::new()?;

		let out = Self {
			msg: LEDMessage::None,

			pin_red: gpio.get(LED_RED)?.into_output(),
			pin_green: gpio.get(LED_GREEN)?.into_output(),
			pin_blue: gpio.get(LED_BLUE)?.into_output(),
		};

		Ok(out)
	}

	/// Set the current message to None
	pub fn quit_message(&mut self) {
		match self.msg {
			LEDMessage::Ok => self.pin_green.set_low(),
			LEDMessage::Wait => self.pin_blue.set_low(),
			LEDMessage::LidOpen => self.pin_red.set_low(),
			LEDMessage::None => {}
			LEDMessage::InternalError => {
				self.pin_green.set_low();
				self.pin_red.set_low();
			}
		}

		self.msg = LEDMessage::None;
	}

	/// Set or replace the current message
	pub fn set_led_msg(&mut self, msg: LEDMessage) {
		self.quit_message();

		self.msg = msg;
		match self.msg {
			LEDMessage::None => {}
			LEDMessage::Ok => self.pin_green.set_high(),
			LEDMessage::Wait => self.pin_blue.set_high(),
			LEDMessage::LidOpen => self.pin_red.set_high(),
			LEDMessage::InternalError => {
				self.pin_green.set_high();
				self.pin_red.set_high();
			}
		}
	}

	pub fn set_led_msg_for_duration(&mut self, msg: LEDMessage, duration: Duration) {
		let bef = self.msg;
		self.set_led_msg(msg);
		std::thread::sleep(duration);
		self.set_led_msg(bef);
	}
}

impl Drop for LEDHandler {
	fn drop(&mut self) {
		self.set_led_msg(LEDMessage::InternalError);
	}
}

const LID_PIN: u8 = 22;
const BUTTON_PIN: u8 = 23;

pub struct InputHandler {
	pub lid_pin: InputPin,
	pub button_pin: InputPin,
	button_on: bool,
}

impl InputHandler {
	pub fn try_new() -> GPIOResult<Self> {
		let gpio = Gpio::new()?;

		let out = Self {
			lid_pin: gpio.get(LID_PIN)?.into_input_pulldown(),
			button_pin: gpio.get(BUTTON_PIN)?.into_input_pulldown(),

			button_on: false,
		};

		Ok(out)
	}

	pub fn lid_open(&self) -> bool {
		self.lid_pin.is_low()
	}
	pub fn lid_closed(&self) -> bool {
		self.lid_pin.is_high()
	}

	pub fn just_pressed(&mut self) -> bool {
		let res: bool = !self.button_on && self.button_pin.is_high();
		self.button_on = self.button_pin.is_high();
		res
	}
}
