#include "constants.h"

class MetalDetector{
	public:
	MetalDetector();
	bool getMetalDetectorState();


	private:
	volatile bool metal_detected_;

};
