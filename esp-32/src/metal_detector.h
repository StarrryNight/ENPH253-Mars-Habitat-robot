#include "constants.h"
#include <cstdint>

class MetalDetector{
	public:
	MetalDetector();
	bool getMetalDetectorState();


	private:
	volatile bool metal_detected_;

	// esp_timer_get_time() timestamp of the most recent RISING interrupt.
	// getMetalDetectorState() auto-clears metal_detected_ once
	// kDetectionHoldUs has elapsed since then — there's no FALLING interrupt
	// to reset it anymore.
	volatile uint64_t detected_at_us_;
	static constexpr uint64_t kDetectionHoldUs = 5000000; // 5 s
};
