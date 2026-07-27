#include "oled.h"
#include "pins.h"
#include <Arduino.h>
#include <Wire.h>
#include <cmath>

namespace {
constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 64;
constexpr uint8_t SCREEN_I2C_ADDRESS = 0x3C;
}

Display::Display() : display_(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1) {}

void Display::begin()
{
	Wire.begin(OLED_SDA, OLED_SCK);
	if (!display_.begin(SSD1306_SWITCHCAPVCC, SCREEN_I2C_ADDRESS)) {
		Serial.println("[Display] SSD1306 init failed");
		return;
	}
	display_.clearDisplay();
	display_.display();
}

void Display::showTeletubbyFace()
{
	display_.clearDisplay();

	// Eyes
	display_.fillCircle(40, 22, 8, SSD1306_WHITE);
	display_.fillCircle(88, 22, 8, SSD1306_WHITE);

	// Smiling mouth: a downward arc traced pixel-by-pixel across the bottom half.
	for (int x = 28; x <= 100; x++) {
		double t = static_cast<double>(x - 28) / (100 - 28);
		int y = 42 + static_cast<int>(16.0 * sin(M_PI * t));
		display_.drawPixel(x, y, SSD1306_WHITE);
		display_.drawPixel(x, y + 1, SSD1306_WHITE);
	}

	display_.display();
}

void Display::showAngryFace()
{
	display_.clearDisplay();

	// Eyes
	display_.fillCircle(40, 26, 7, SSD1306_WHITE);
	display_.fillCircle(88, 26, 7, SSD1306_WHITE);

	// Angled eyebrows, slanting down toward the center (angry V shape).
	display_.drawLine(24, 8, 48, 16, SSD1306_WHITE);
	display_.drawLine(104, 8, 80, 16, SSD1306_WHITE);
	display_.drawLine(24, 9, 48, 17, SSD1306_WHITE);
	display_.drawLine(104, 9, 80, 17, SSD1306_WHITE);

	// Frowning mouth: an upward arc (opposite curvature of the smile).
	for (int x = 28; x <= 100; x++) {
		double t = static_cast<double>(x - 28) / (100 - 28);
		int y = 52 - static_cast<int>(12.0 * sin(M_PI * t));
		display_.drawPixel(x, y, SSD1306_WHITE);
		display_.drawPixel(x, y + 1, SSD1306_WHITE);
	}

	display_.display();
}
