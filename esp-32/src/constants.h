// Control
static constexpr double CONTROL_LOOP_PERIOD = 1e-6;

// Photoresistors
static constexpr int LEFT_PHOTORESISTOR = 10;
static constexpr int MID_LEFT_PHOTORESISTOR = 11;
static constexpr int MID_RIGHT_PHOTORESISTOR = 12;
static constexpr int RIGHT_PHOTORESISTOR = 13;

static constexpr double LIGHT_THRESHOLD_V = 0.6;
static constexpr double SMALL_ERROR_VALUE = 1;
static constexpr double BIG_ERROR_VALUE = 5;
// Motor
// Pins
static constexpr int WHEEL_1_PWM_CHANNEL_0 = 0;
static constexpr int WHEEL_1_PWM_CHANNEL_1 = 1;

static constexpr int WHEEL_1_PWM_PIN_0 = 1;
static constexpr int WHEEL_1_PWM_PIN_1 = 46;

static constexpr int WHEEL_2_PWM_CHANNEL_0 = 2;
static constexpr int WHEEL_2_PWM_CHANNEL_1 = 3;

static constexpr int WHEEL_2_PWM_PIN_0 = 2;
static constexpr int WHEEL_2_PWM_PIN_1 = 45;

static constexpr int WHEEL_3_PWM_CHANNEL_0 = 4;
static constexpr int WHEEL_3_PWM_CHANNEL_1 = 5;

static constexpr int WHEEL_3_PWM_PIN_0 = 3;
static constexpr int WHEEL_3_PWM_PIN_1 = 44;
// Encoder
static constexpr int WHEEL_1_ENCODER_0 = 3;
static constexpr int WHEEL_1_ENCODER_1 = 4;

static constexpr int WHEEL_2_ENCODER_0 = 0;
static constexpr int WHEEL_2_ENCODER_1 = 1;

static constexpr int WHEEL_3_ENCODER_0 = 0;
static constexpr int WHEEL_3_ENCODER_1 = 1;

static constexpr double ENCODER_RESOLUTION_DISTANCE_M = 0.3;

// Values
static constexpr int speed8bit = 230; // speed scales linear from 0 to 256

// Arm
