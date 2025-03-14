// #define USE_SOFTWARE_SPI

#ifndef USE_SOFTWARE_SPI
	// #define HSPI_INTERFACE HSPI
	#define HSPI_INTERFACE VSPI

	// HACK: fixes SPI.beginTransaction() hangs=ing on SPI_PARAM_LOCK()
    // even though nothing else should be acquiring the lock beforehand.
    // we don't have any concurrent code here so it should be fine
    // TODO: double check if something in the u8g2 constructor is getting the lock before serial is up?
	#define CONFIG_DISABLE_HAL_LOCKS 1
#endif

#include <Arduino.h>
#include <Bounce2.h>
#include <EEPROM.h>
#include <ESP32Servo.h>
#include <ESPUI.h>
#include <ESPmDNS.h>
#include <FastAccelStepper.h>
#include <SPI.h>
#include <TMCStepper.h>
#include <U8g2lib.h>
#include <WiFi.h>

#include "./assets/c-base-logo.h"
#include "./assets/c-base-shape.h"

// Settings
// #define SLOW_BOOT 0
#define HOSTNAME "monorail"
#define FORCE_USE_HOTSPOT 0

#define MICROSTEPS 16

#define EN_PIN 13           // Enable
#define DIR_PIN 27          // Direction
#define STEP_PIN 14         // Step
#define SW_SCK 12           // Software Slave Clock (SCK)
#define SERIAL_PORT Serial2 // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2

#define R_SENSE 0.11f // Match to your driver, SilentStepStick series use 0.11

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

#define SERVO_PIN 15

Servo doorServo;

// #define

// 1 = orange = led+
// 2 = c = yellow
// 3 = nc = green
// 4 = blue = led-

#define DOOR_LED_PIN 0
#define DOOR_BUTTON_PIN 4

Bounce2::Button doorButton = Bounce2::Button();

#define LED_DC_PIN 32
#define LED_RESET_PIN 21

#ifdef USE_SOFTWARE_SPI

	#define LED_COPI_PIN 26
	#define LED_SCK_PIN 25
	#define LED_CS_PIN 33

// software SPI (slow)
// mode 1 -- small buffer to allow non-blocking drawing (others: 2/F)
U8G2_SSD1322_NHD_256X64_1_4W_SW_SPI u8g2(
	U8G2_R0,
	/* sck */ LED_SCK_PIN,
	/* data */ LED_COPI_PIN,
	/* cs */ LED_CS_PIN,
	/* dc */ LED_DC_PIN,
	/* reset */ LED_RESET_PIN
);
#else
	#if HSPI_INTERFACE == HSPI
	// builtin pins for hardware SPI #1
	// https://www.teachmemicro.com/esp32-pinout-diagram-wroom-32/
	// #define LED_CIPO_PIN 12 // unused
		#define LED_COPI_PIN 13 // "SDIN"
		#define LED_SCK_PIN 14
		#define LED_CS_PIN 15
	#elif HSPI_INTERFACE == VSPI
		// builtin pins for hardware SPI #2 (VSPI)
	    // #define LED_CIPO_PIN 19 // unused
		#define LED_COPI_PIN 23 // "SDIN"
		#define LED_SCK_PIN 18
		#define LED_CS_PIN 5
	#endif

// Hardware SPI
U8G2_SSD1322_NHD_256X64_F_4W_HW_SPI

u8g2(
	U8G2_R0,
	/* cs */ LED_CS_PIN,
	/* dc */ LED_DC_PIN,
	/* reset */ LED_RESET_PIN
);
#endif

// Function Prototypes
void connectWifi();
void setUpUI();
void enterWifiDetailsCallback(Control *sender, int type);
void generalCallback(Control *sender, int type);

void noopCallback(Control *sender, int type) {}

// UI handles
uint16_t wifi_ssid_text, wifi_pass_text;
uint16_t maxSpeedInput, accelerationInput, decelerationInput;
uint16_t servoClosedInput, servoOpenInput;
uint16_t forwardButton, stopButton, backButton;
uint16_t doorToggleButton;

long long currentSpeed = 0;
long long targetSpeed = 0;

long long targetSpeedSps = 0;

bool targetDir = true;

long long accel;
long long accelSps2;

bool doorOpen = false;
long long doorTarget = 0;

// This is the main function which builds our GUI
void setUpUI() {
	// Turn off verbose debugging
	ESPUI.setVerbosity(Verbosity::Quiet);

	// Make sliders continually report their position as they are being dragged.
	ESPUI.sliderContinuous = true;

	/*
	 * Tab: Monorail Controls
	 *
	 *-----------------------------------------------------------------------------------------------------------*/
	auto maintab = ESPUI.addControl(Tab, "", "Monorail Controls");

	backButton = ESPUI.addControl(
		ControlType::Button,
		"Controls",
		"<< Back",
		ControlColor::Dark,
		maintab,
		[](Control *sender, int type) {
			if (type == B_DOWN) {
				Serial.println("bck pressed");

				targetSpeedSps = -ESPUI.getControl(maxSpeedInput)->value.toInt();
				// targetDir = false;
			} else {
				Serial.println("bck released");

				targetSpeedSps = 0;
			}
		}
	);

	stopButton =
		ESPUI
			.addControl(ControlType::Button, "", "Stop", ControlColor::None, backButton, [](Control *sender, int type) {
				if (type == B_DOWN) {
					Serial.println("stop pressed");

					targetSpeedSps = 0;
				}
			});

	forwardButton = ESPUI.addControl(
		ControlType::Button,
		"",
		"Forward >>",
		ControlColor::None,
		backButton,
		[](Control *sender, int type) {
			if (type == B_DOWN) {
				Serial.println("fwd pressed");

				targetSpeedSps = ESPUI.getControl(maxSpeedInput)->value.toInt();
				// targetDir = true;
			} else {
				Serial.println("fwd released");

				targetSpeedSps = 0;
			}
		}
	);

	doorToggleButton = ESPUI.addControl(
		ControlType::Button,
		"",
		"Toggle Door",
		ControlColor::None,
		maintab,
		[](Control *sender, int type) {
			if (type == B_DOWN) {
				Serial.println("toggle door pressed");

				doorOpen = !doorOpen;

				if (doorOpen) {
					doorTarget = ESPUI.getControl(servoOpenInput)->value.toInt();
				} else {
					doorTarget = ESPUI.getControl(servoClosedInput)->value.toInt();
				}

				doorServo.write(doorTarget);
			}
		}
	);

	ESPUI.addControl(Separator, "Parameters", "", None, maintab);

	maxSpeedInput = ESPUI.addControl(Number, "Max Speed (Steps/s)", "500", Emerald, maintab, generalCallback);
	ESPUI.addControl(Min, "", "10", None, maxSpeedInput);
	ESPUI.addControl(Max, "", "10000", None, maxSpeedInput);

	accelerationInput = ESPUI.addControl(Number, "Acceleration (Steps/s^2)", "200", Emerald, maintab, generalCallback);
	ESPUI.addControl(Min, "", "10", None, accelerationInput);
	ESPUI.addControl(Max, "", "5000", None, accelerationInput);

	decelerationInput = ESPUI.addControl(Number, "Deceleration (Steps/s^2)", "1000", Emerald, maintab, generalCallback);
	ESPUI.addControl(Min, "", "10", None, decelerationInput);
	ESPUI.addControl(Max, "", "5000", None, decelerationInput);

	servoClosedInput = ESPUI.addControl(Number, "Servo Closed Position", "34", Emerald, maintab, generalCallback);
	// ESPUI.addControl(Min, "", "0", None, decelerationInput);
	// ESPUI.addControl(Max, "", "360", None, decelerationInput);

	servoOpenInput = ESPUI.addControl(Number, "Servo Open Position", "110", Emerald, maintab, generalCallback);
	// ESPUI.addControl(Min, "", "0", None, decelerationInput);
	// ESPUI.addControl(Max, "", "360", None, decelerationInput);

	/*
	 * Tab: WiFi Credentials
	 * You use this tab to enter the SSID and password of a wifi network to autoconnect to.
	 *-----------------------------------------------------------------------------------------------------------*/
	auto wifitab = ESPUI.addControl(Tab, "", "WiFi Credentials");
	wifi_ssid_text = ESPUI.addControl(Text, "SSID", "", Alizarin, wifitab, noopCallback);
	// Note that adding a "Max" control to a text control sets the max length
	ESPUI.addControl(Max, "", "32", None, wifi_ssid_text);
	wifi_pass_text = ESPUI.addControl(Text, "Password", "", Alizarin, wifitab, noopCallback);
	ESPUI.addControl(Max, "", "64", None, wifi_pass_text);
	ESPUI.addControl(Button, "Save", "Save", Peterriver, wifitab, enterWifiDetailsCallback);

	// Finally, start up the UI.
	// This should only be called once we are connected to WiFi.
	ESPUI.begin(HOSTNAME);
}

// Most elements in this test UI are assigned this generic callback which prints some
// basic information. Event types are defined in ESPUI.h
void generalCallback(Control *sender, int type) {
	Serial.print("CB: id(");
	Serial.print(sender->id);
	Serial.print(") Type(");
	Serial.print(type);
	Serial.print(") '");
	Serial.print(sender->label);
	Serial.print("' = ");
	Serial.println(sender->value);
}

//============================================
// LCD drawing
//============================================

// 40MHz, taken from here, but i don't know if it applies to hardware SPI
// https://docs.espressif.com/projects/esp-idf/en/release-v5.2/esp32/api-reference/peripherals/spi_master.html#gpio-matrix-and-io-mux
long SPI_CLOCK_SPEED_HZ = 40 * 1000 * 1000;

void setupLCD() {
#ifndef USE_SOFTWARE_SPI
	// https://github.com/olikraus/u8g2/blob/2b75f932b5ef4b4de8edf73e1a690702a35b1976/doc/faq.txt#L35-L36
	SPI = SPIClass(HSPI_INTERFACE);
#endif

	// TODO: maybe this can improve the speed? try different values
	// u8g2.setBusClock(SPI_CLOCK_SPEED_HZ);
	Serial.println("u8g2.begin()");
	u8g2.begin();
	Serial.println("u8g2.begin() done");
}

template<typename T>
struct IncrementalDraw {
	bool inProgress;
	unsigned long frameStartTime;

	T arg;
	void (*drawCallback)(unsigned long time, T *arg);

	IncrementalDraw(void (*drawCallback)(unsigned long time, T *arg)) {
		this->drawCallback = drawCallback;
		this->inProgress = false;
		this->frameStartTime = 0;
	}

	unsigned int drawForDuration(unsigned long maxDuration, T *arg) {
		if (!this->inProgress) {
			this->inProgress = true;
			this->frameStartTime = millis();
			this->arg = *arg;
			u8g2.firstPage();
		}
		auto start = millis();
		auto end = -1;

		while (true) {
			this->drawCallback(this->frameStartTime, &this->arg);

			// blocks for ~50ms
			auto stillDrawing = u8g2.nextPage();
			end = millis();
			auto duration = end - start;

			// Serial.print("drew ");
			// Serial.print(this->arg);
			// Serial.print(" ");
			// Serial.print(stillDrawing ? "not done yet" : "done");
			// Serial.println();

			if (!stillDrawing) {
				// we're done frawing this frame
				this->inProgress = false;
				break;
			}
			if (duration >= maxDuration) {
				// we're not done drawing the frame, but we exceeded maxDuration, so it's time to yield
				break;
			}
		}
		return end - start;
	}
};

void drawSetup(void) {
	u8g2.setFont(u8g2_font_6x10_tf);
	u8g2.setFontRefHeightExtendedText();
	u8g2.setDrawColor(1);
	u8g2.setFontPosTop();
	u8g2.setFontDirection(0);
}

typedef long long DrawArg;

int sequenceDurationMs = 1000;

double square(double x) {
	return x * x;
}

bool DRAW_TEST_SPEED = false;

void draw(unsigned long time, DrawArg *rawSpeed) {
	drawSetup();

	char speedStr[64];
	if (DRAW_TEST_SPEED) {
		// changes the speed according to a sine wave, ignoring any input
		long long loopDurationMs = 2000;
		auto seqTime = time / 10;
		double loopPercent = (double)(seqTime % loopDurationMs) / (double)loopDurationMs;
		double speed = 100 * square(sin(loopPercent * PI));

		snprintf(speedStr, (sizeof speedStr), "%05.2f", speed);
	} else {
		auto speed = *rawSpeed;
		snprintf(speedStr, (sizeof speedStr), "%4lld", speed);
	}

	u8g2.drawXBMP(2, 2, c_base_shape.width, c_base_shape.height, c_base_shape.data);
	u8g2.setFont(u8g2_font_6x10_mf);
	u8g2.drawStr(c_base_shape.width + 2 + 16, 8, "Monorail speed");

	u8g2.setFont(u8g2_font_inb24_mn);
	u8g2.drawStr(c_base_shape.width + 2 + 16, 20, speedStr);

	u8g2.setFont(u8g2_font_12x6LED_tf);
	u8g2.drawStr(3 * 64 - 16, 64 - 30, "steps/s");
}

auto incrementalDraw = new IncrementalDraw<DrawArg>(&draw);

//============================================
// Setup + loop
//============================================

int ledState = LOW;

void setup() {
	randomSeed(0);
	Serial.begin(115200);
	// while (!Serial)
	// 	;
	// if (SLOW_BOOT)
	// 	delay(5000); // Delay booting to give time to connect a serial monitor
	connectWifi();

	WiFi.setSleep(false); // For the ESP32: turn off sleeping to increase UI responsivness (at the cost of power use)

	setUpUI();
	setupLCD();

	// STEPPER

	SERIAL_PORT.begin(115200);

	pinMode(EN_PIN, OUTPUT); // Set pinmodes
	pinMode(STEP_PIN, OUTPUT);
	pinMode(DIR_PIN, OUTPUT);
	// digitalWrite(EN_PIN, LOW); // Enable TMC2209 board

	driver.begin();                // UART: Init SW UART (if selected) with default 115200 baudrate
	driver.toff(5);                // Enables driver in software
	driver.rms_current(1500);      // Set motor RMS current
	driver.microsteps(MICROSTEPS); // Set microstepss

	driver.en_spreadCycle(true);
	driver.pwm_autoscale(true); // Needed for stealthChop

	driver.hold_multiplier(0.01);
	// driver.TPWMTHRS(500);

	engine.init();

	stepper = engine.stepperConnectToPin(STEP_PIN, DRIVER_DONT_CARE);
	stepper->setDirectionPin(DIR_PIN);
	stepper->setEnablePin(EN_PIN);
	stepper->setAutoEnable(true);
	stepper->enableOutputs();

	// Serial.print("FACTORY_CONF ");
	// Serial.println(driver.FACTORY_CONF());
	// Serial.print("PWM_SCALE ");
	// Serial.println(driver.PWM_SCALE());

	// SERVO

	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

	doorServo.setPeriodHertz(50); // standard 50 hz servo
	// doorServo.attach(SERVO_PIN, 1000, 2000);
	doorServo.attach(SERVO_PIN);

	// BUTTON

	pinMode(DOOR_LED_PIN, OUTPUT);
	digitalWrite(DOOR_LED_PIN, ledState);

	doorButton.attach(DOOR_BUTTON_PIN, INPUT_PULLUP); // USE EXTERNAL PULL-UP

	// DEBOUNCE INTERVAL IN MILLISECONDS
	doorButton.interval(5);

	// INDICATE THAT THE HIGH STATE CORRESPONDS TO PHYSICALLY PRESSING THE BUTTON
	doorButton.setPressedState(LOW);
}

void loop() {
	doorButton.update();

	if (doorButton.pressed()) {
		ledState = !ledState;                 // SET ledState TO THE OPPOSITE OF ledState
		digitalWrite(DOOR_LED_PIN, ledState); // WRITE THE NEW ledState

		doorOpen = !doorOpen;

		if (doorOpen) {
			doorTarget = ESPUI.getControl(servoOpenInput)->value.toInt();
		} else {
			doorTarget = ESPUI.getControl(servoClosedInput)->value.toInt();
		}

		doorServo.write(doorTarget);
	}

	static long unsigned lastTime = 0;
	if (lastTime == 0) {
		lastTime = micros();
	}

	static long unsigned delta = micros() - lastTime;
	lastTime = micros();

	bool signsDifferent = std::signbit(currentSpeed) != std::signbit(targetSpeed);

	targetSpeed = targetSpeedSps * MICROSTEPS;
	if (currentSpeed != 0 && signsDifferent) {
		targetSpeed = 0;
	}

	if (targetSpeed == 0) {
		accelSps2 = ESPUI.getControl(decelerationInput)->value.toInt();
	} else {
		accelSps2 = ESPUI.getControl(accelerationInput)->value.toInt();
	}

	// if (targetSpeed < currentSpeed) {
	// 	accelSps2 = -accelSps2;
	// }

	accel = (accelSps2 * MICROSTEPS * delta) / 1000;

	if (currentSpeed != targetSpeed) {
		currentSpeed += accel;
	}

	if (accelSps2 < 0 && currentSpeed < targetSpeed) {
		currentSpeed = targetSpeed;
	}
	if (accelSps2 > 0 && currentSpeed > targetSpeed) {
		currentSpeed = targetSpeed;
	}

	stepper->setAcceleration(accelSps2 * MICROSTEPS);
	stepper->setSpeedInHz(abs(targetSpeed));
	stepper->setLinearAcceleration(100);

	if (targetSpeed == 0) {
		stepper->stopMove();
	} else if (targetSpeed > 0) {
		stepper->runForward();
	} else if (targetSpeed < 0) {
		stepper->runBackward();
	}

	unsigned targetLoopDuration = 50;
#ifdef USE_SOFTWARE_SPI
	auto drawDuration = incrementalDraw->drawForDuration(targetLoopDuration, &currentSpeed);
#else
	auto drawStart = micros();
	u8g2.clearBuffer();
	draw(millis(), &currentSpeed);
	u8g2.sendBuffer();
	auto drawDuration = (micros() - drawStart) / 1000;
#endif

	// driver.VACTUAL(currentSpeed);

	// Serial.print("TSTEP: ");
	// Serial.println(driver.TSTEP());

	// Serial.print("Speed: ");
	// Serial.print(currentSpeed);
	// Serial.print(", Target Speed: ");
	// Serial.print(targetSpeed);
	// Serial.print(", Accel: ");
	// Serial.print(accel);
	// Serial.print(", Delta: ");
	Serial.print("draw duration: ");
	Serial.println(drawDuration);

	// if (drawDuration < targetLoopDuration) {
	// 	delay(targetLoopDuration - drawDuration);
	// }
}

void readStringFromEEPROM(String &buf, int baseaddress, int size) {
	buf.reserve(size);
	for (int i = baseaddress; i < baseaddress + size; i++) {
		char c = EEPROM.read(i);
		buf += c;
		if (!c)
			break;
	}
}

void connectWifi() {
	int connect_timeout;

	WiFi.setHostname(HOSTNAME);

	Serial.println("Begin wifi...");

	// Load credentials from EEPROM
	if (!(FORCE_USE_HOTSPOT)) {
		yield();
		EEPROM.begin(100);
		String stored_ssid, stored_pass;
		readStringFromEEPROM(stored_ssid, 0, 32);
		readStringFromEEPROM(stored_pass, 32, 96);
		EEPROM.end();

		// Try to connect with stored credentials, fire up an access point if they don't work.
		WiFi.begin(stored_ssid.c_str(), stored_pass.c_str());

		connect_timeout = 28; // 7 seconds
		while (WiFi.status() != WL_CONNECTED && connect_timeout > 0) {
			delay(250);
			Serial.print(".");
			connect_timeout--;
		}
	}

	if (WiFi.status() == WL_CONNECTED) {
		Serial.println(WiFi.localIP());
		Serial.println("Wifi started");

		if (!MDNS.begin(HOSTNAME)) {
			Serial.println("Error setting up MDNS responder!");
		}
	} else {
		Serial.println("\nCreating access point...");
		WiFi.mode(WIFI_AP);
		WiFi.softAPConfig(IPAddress(192, 168, 1, 1), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
		WiFi.softAP(HOSTNAME);

		connect_timeout = 20;
		do {
			delay(250);
			Serial.print(",");
			connect_timeout--;
		} while (connect_timeout);
	}
}

void enterWifiDetailsCallback(Control *sender, int type) {
	if (type == B_UP) {
		Serial.println("Saving credentials to EPROM...");
		Serial.println(ESPUI.getControl(wifi_ssid_text)->value);
		Serial.println(ESPUI.getControl(wifi_pass_text)->value);
		unsigned int i;
		EEPROM.begin(100);
		for (i = 0; i < ESPUI.getControl(wifi_ssid_text)->value.length(); i++) {
			EEPROM.write(i, ESPUI.getControl(wifi_ssid_text)->value.charAt(i));
			if (i == 30)
				break; // Even though we provided a max length, user input should never be trusted
		}
		EEPROM.write(i, '\0');

		for (i = 0; i < ESPUI.getControl(wifi_pass_text)->value.length(); i++) {
			EEPROM.write(i + 32, ESPUI.getControl(wifi_pass_text)->value.charAt(i));
			if (i == 94)
				break; // Even though we provided a max length, user input should never be trusted
		}
		EEPROM.write(i + 32, '\0');
		EEPROM.end();
	}
}
