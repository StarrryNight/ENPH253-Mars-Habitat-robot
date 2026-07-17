#pragma once
#include "metal_detector.h"
#include "pins.h"
#include <Arduino.h>

MetalDetector::MetalDetector():metal_detected_(false) {	
	attachInterruptArg(METAL_DETECTOR_PIN, [](void *arg) IRAM_ATTR
					   {
	    MetalDetector* m = static_cast<MetalDetector*>(arg);
	    m->metal_detected_=true; }, this, RISING);

	attachInterruptArg(METAL_DETECTOR_PIN, [](void *arg) IRAM_ATTR
					   {
	    MetalDetector* m = static_cast<MetalDetector*>(arg);
	    m->metal_detected_=false; }, this, FALLING);
}	

bool MetalDetector::getMetalDetectorState(){
return metal_detected_;
}

