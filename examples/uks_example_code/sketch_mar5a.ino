#include <SoftwareSerial.h> // Software serial for the UART to TMC2209 - https://www.arduino.cc/en/Reference/softwareSerial
#include <Streaming.h>      // For serial debugging output - https://www.arduino.cc/reference/en/libraries/streaming/
#include <TMCStepper.h>     // TMCstepper - https://github.com/teemuatlut/TMCStepper

#define EN_PIN 2            // Enable - PURPLE
#define DIR_PIN 3           // Direction - WHITE
#define STEP_PIN 4          // Step - ORANGE
#define SW_SCK 5            // Software Slave Clock (SCK) - BLUE
#define SW_TX 6             // SoftwareSerial receive pin - BROWN
#define SW_RX 7             // SoftwareSerial transmit pin - YELLOW
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f       // SilentStepStick series use 0.11 ...and so does my fysetc TMC2209 (?)

SoftwareSerial SoftSerial(SW_RX, SW_TX); // Be sure to connect RX to TX and TX to RX between both devices

TMC2209Stepper TMCdriver(&SoftSerial, R_SENSE, DRIVER_ADDRESS); // Create TMC driver

int accel;
long maxSpeed;
int speedChangeDelay;
bool dir = false;

//== Setup ===============================================================================

void setup() {
	Serial.begin(115200);          // initialize hardware serial for debugging
	SoftSerial.begin(115200);      // initialize software serial for UART motor control
	TMCdriver.beginSerial(115200); // Initialize UART

	pinMode(EN_PIN, OUTPUT); // Set pinmodes
	pinMode(STEP_PIN, OUTPUT);
	pinMode(DIR_PIN, OUTPUT);
	digitalWrite(EN_PIN, LOW); // Enable TMC2209 board

	TMCdriver.begin();           // UART: Init SW UART (if selected) with default 115200 baudrate
	TMCdriver.toff(5);           // Enables driver in software
	TMCdriver.rms_current(1500); // Set motor RMS current
	TMCdriver.microsteps(256);   // Set microsteps
	TMCdriver.en_spreadCycle(true);
	TMCdriver.pwm_autoscale(true); // Needed for stealthChop
}

//== Loop =================================================================================

void loop() {
	accel = 8000;      // Speed increase/decrease amount
	maxSpeed = 160000; // Maximum speed to be reached
	speedChangeDelay = 100;
	// Delay between speed changes

	TMCdriver.shaft(dir); // SET DIRECTION

	for (long i = 0; i <= maxSpeed; i = i + accel) { // Speed up to maxSpeed
		TMCdriver.VACTUAL(i);                        // Set motor speed
		Serial << "v=" << TMCdriver.VACTUAL() << ", c=" << TMCdriver.DRV_STATUS() << endl;
		delay(speedChangeDelay);
	}
	if (dir == true) {
		delay(100);
	} else {
		delay(250);
	}

	for (long i = maxSpeed; i >= 0; i = i - accel) { // Decrease speed to zero
		TMCdriver.VACTUAL(i);
		// Serial << TMCdriver.VACTUAL() << endl;
		Serial << "v=" << TMCdriver.VACTUAL() << ", c=" << TMCdriver.DRV_STATUS() << endl;
		delay(speedChangeDelay);
	}

	delay(1200);
	dir = !dir; // REVERSE DIRECTION
}