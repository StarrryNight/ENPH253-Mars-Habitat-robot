#pragma once
#include <Adafruit_SSD1306.h>

// Thin wrapper around the SSD1306 OLED (I2C, see OLED_SDA/OLED_SCK in
// pins.h). Currently only draws a static face when a Teletubby is detected.
class Display
{
public:
	Display();

	// Starts I2C on OLED_SDA/OLED_SCK and initializes the SSD1306. Must be
	// called once, after Arduino init (matches Arm::begin()/MotorController::setup()).
	void begin();

	// Clears the screen and draws a simple smiley face.
	void showTeletubbyFace();

	// Clears the screen and draws an angry face (frown + angled eyebrows).
	void showAngryFace();

private:
	Adafruit_SSD1306 display_;
};
