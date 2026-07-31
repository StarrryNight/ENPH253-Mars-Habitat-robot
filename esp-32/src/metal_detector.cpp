#pragma once
#include "metal_detector.h"
#include "pins.h"
#include <Arduino.h>
#include "esp_timer.h"

MetalDetector::MetalDetector():metal_detected_(false), detected_at_us_(0) {
	pinMode(METAL_DETECTOR_PIN, INPUT_PULLUP);

	attachInterruptArg(METAL_DETECTOR_PIN, [](void *arg) IRAM_ATTR
					   {
	    MetalDetector* m = static_cast<MetalDetector*>(arg);
	    m->metal_detected_=true;
	    m->detected_at_us_=esp_timer_get_time();
	    Serial.print("[MetalDetector] RISING interrupt\n"); }, this, FALLING);

}

void MetalDetector::clear(){
	metal_detected_ = false;
	detected_at_us_ = 0;
}

bool MetalDetector::getMetalDetectorState(){
	if (metal_detected_ && (esp_timer_get_time() - detected_at_us_) >= kDetectionHoldUs){
		metal_detected_ = false;
	}
	return metal_detected_;
}

