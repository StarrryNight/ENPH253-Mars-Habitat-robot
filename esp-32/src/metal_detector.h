#include "constants.h"
#include <cstdint>

class MetalDetector{
	public:
	MetalDetector();
	bool getMetalDetectorState();

	// Drops a latched detection immediately, without waiting out
	// kDetectionHoldUs. Call before a deliberate measurement so the answer
	// describes that measurement: the latch holds a hit for 5 s, which is long
	// enough that a detection from the previous rock (or from the arm swinging
	// past metal on the way in) would otherwise still read true when the probe
	// pose lands. See AI::tickMetalDetecting.
	void clear();


	private:
	volatile bool metal_detected_;

	// esp_timer_get_time() timestamp of the most recent RISING interrupt.
	// getMetalDetectorState() auto-clears metal_detected_ once
	// kDetectionHoldUs has elapsed since then — there's no FALLING interrupt
	// to reset it anymore.
	volatile uint64_t detected_at_us_;
	static constexpr uint64_t kDetectionHoldUs = 5000000; // 5 s
};
