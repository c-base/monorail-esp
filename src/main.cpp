#include <Arduino.h>
#include <ESPmDNS.h>
#include <EEPROM.h>
#include <ESPUI.h>
#include <TMCStepper.h>
#include <WiFi.h>

// Settings
#define SLOW_BOOT 0
#define HOSTNAME "Monorail"
#define FORCE_USE_HOTSPOT 1

#define MICROSTEPS 64

#define EN_PIN           13 // Enable
#define DIR_PIN          27 // Direction
#define STEP_PIN         14 // Step
#define SW_SCK           12 // Software Slave Clock (SCK)
#define SERIAL_PORT Serial2 // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2

#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);


// Function Prototypes
void connectWifi();
void setUpUI();
void generalCallback(Control *sender, int type);

// uint16_t wifi_ssid_text, wifi_pass_text;
// uint16_t mainLabel, mainSwitcher, mainSlider, mainText, mainNumber, mainScrambleButton, mainTime;
// uint16_t styleButton, styleLabel, styleSwitcher, styleSlider, styleButton2, styleLabel2, styleSlider2;
// uint16_t graph;
// volatile bool updates = false;

// UI handles
uint16_t maxSpeedInput;
uint16_t accelerationInput;
uint16_t decelerationInput;

uint16_t forwardButton;
uint16_t backButton;

long long currentSpeed = 0;
long long targetSpeed = 0;

long long targetSpeedSps = 0;

bool targetDir = true;

long long accel;
long long accelSps2;

// This is the main function which builds our GUI
void setUpUI()
{
  // Turn off verbose debugging
  ESPUI.setVerbosity(Verbosity::Quiet);

  // Make sliders continually report their position as they are being dragged.
  ESPUI.sliderContinuous = true;

  backButton = ESPUI.addControl(ControlType::Button, "Controls", "<< Back", ControlColor::Dark, Control::noParent, [](Control *sender, int type){
    if (type == B_DOWN) {
      Serial.println("bck pressed");

      targetSpeedSps = -ESPUI.getControl(maxSpeedInput)->value.toInt();
      // targetDir = false;
    } else {
      Serial.println("bck released");

      targetSpeedSps = 0;
    }
  });
  forwardButton = ESPUI.addControl(ControlType::Button, "", "Forward >>", ControlColor::None, backButton, [](Control *sender, int type){
    if (type == B_DOWN) {
      Serial.println("fwd pressed");

      targetSpeedSps = ESPUI.getControl(maxSpeedInput)->value.toInt();
      // targetDir = true;
    } else {
      Serial.println("fwd released");

      targetSpeedSps = 0;
    }
  });

  ESPUI.addControl(Separator, "Parameters", "", None, Control::noParent);

  maxSpeedInput = ESPUI.addControl(Number, "Max Speed (Steps/s)", "1000", Emerald, Control::noParent, generalCallback);
  ESPUI.addControl(Min, "", "10", None, maxSpeedInput);
  ESPUI.addControl(Max, "", "10000", None, maxSpeedInput);

  accelerationInput = ESPUI.addControl(Number, "Acceleration (Steps/s^2)", "1000", Emerald, Control::noParent, generalCallback);
  ESPUI.addControl(Min, "", "10", None, accelerationInput);
  ESPUI.addControl(Max, "", "5000", None, accelerationInput);

  decelerationInput = ESPUI.addControl(Number, "Deceleration (Steps/s^2)", "1000", Emerald, Control::noParent, generalCallback);
  ESPUI.addControl(Min, "", "10", None, decelerationInput);
  ESPUI.addControl(Max, "", "5000", None, decelerationInput);

  // Finally, start up the UI.
  // This should only be called once we are connected to WiFi.
  ESPUI.begin(HOSTNAME);
}

// Most elements in this test UI are assigned this generic callback which prints some
// basic information. Event types are defined in ESPUI.h
void generalCallback(Control *sender, int type)
{
  Serial.print("CB: id(");
  Serial.print(sender->id);
  Serial.print(") Type(");
  Serial.print(type);
  Serial.print(") '");
  Serial.print(sender->label);
  Serial.print("' = ");
  Serial.println(sender->value);
}

void setup()
{
  randomSeed(0);
  Serial.begin(115200);
  while (!Serial)
    ;
  if (SLOW_BOOT)
    delay(5000); // Delay booting to give time to connect a serial monitor
  connectWifi();
#if defined(ESP32)
  WiFi.setSleep(false); // For the ESP32: turn off sleeping to increase UI responsivness (at the cost of power use)
#endif
  setUpUI();

  // SoftSerial.begin(115200);           // initialize software serial for UART motor control
  // TMCdriver.beginSerial(115200);      // Initialize UART
  SERIAL_PORT.begin(115200);

  pinMode(EN_PIN, OUTPUT);           // Set pinmodes
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);         // Enable TMC2209 board

  driver.begin();                                                                                                                                                                                                                                                                                                                            // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.toff(5);                  // Enables driver in software
  driver.rms_current(2000);        // Set motor RMS current
  driver.microsteps(MICROSTEPS);   // Set microsteps

  driver.en_spreadCycle(true);
  driver.pwm_autoscale(true);     // Needed for stealthChop
}

void loop()
{
  static long unsigned lastTime = 0;
  if (lastTime == 0) {
    lastTime = micros();
  }

  static long unsigned delta = micros() - lastTime;
  lastTime = micros();

  targetSpeed = targetSpeedSps * MICROSTEPS;

  if (currentSpeed < targetSpeed) {
    accelSps2 = ESPUI.getControl(accelerationInput)->value.toInt();
  } else {
    accelSps2 = -ESPUI.getControl(decelerationInput)->value.toInt();
  }

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

  // driver.shaft(targetDir);
  driver.VACTUAL(currentSpeed);

  // Serial.print("Speed: ");
  // Serial.print(currentSpeed);
  // Serial.print(", Target Speed: ");
  // Serial.print(targetSpeed);
  // Serial.print(", Accel: ");
  // Serial.print(accel);
  // Serial.print(", Delta: ");
  // Serial.println(delta);

  delay(50);
}

// Utilities
//
// If you are here just to see examples of how to use ESPUI, you can ignore the following functions
//------------------------------------------------------------------------------------------------
void readStringFromEEPROM(String &buf, int baseaddress, int size)
{
  buf.reserve(size);
  for (int i = baseaddress; i < baseaddress + size; i++)
  {
    char c = EEPROM.read(i);
    buf += c;
    if (!c)
      break;
  }
}

void connectWifi()
{
  int connect_timeout;

  WiFi.setHostname(HOSTNAME);

  Serial.println("Begin wifi...");

  // Load credentials from EEPROM
  if (!(FORCE_USE_HOTSPOT))
  {
    yield();
    EEPROM.begin(100);
    String stored_ssid, stored_pass;
    readStringFromEEPROM(stored_ssid, 0, 32);
    readStringFromEEPROM(stored_pass, 32, 96);
    EEPROM.end();

    // Try to connect with stored credentials, fire up an access point if they don't work.
    WiFi.begin(stored_ssid.c_str(), stored_pass.c_str());

    connect_timeout = 28; // 7 seconds
    while (WiFi.status() != WL_CONNECTED && connect_timeout > 0)
    {
      delay(250);
      Serial.print(".");
      connect_timeout--;
    }
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println(WiFi.localIP());
    Serial.println("Wifi started");

    if (!MDNS.begin(HOSTNAME))
    {
      Serial.println("Error setting up MDNS responder!");
    }
  }
  else
  {
    Serial.println("\nCreating access point...");
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(IPAddress(192, 168, 1, 1), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
    WiFi.softAP(HOSTNAME);

    connect_timeout = 20;
    do
    {
      delay(250);
      Serial.print(",");
      connect_timeout--;
    } while (connect_timeout);
  }
}

// void enterWifiDetailsCallback(Control *sender, int type)
// {
//   if (type == B_UP)
//   {
//     Serial.println("Saving credentials to EPROM...");
//     Serial.println(ESPUI.getControl(wifi_ssid_text)->value);
//     Serial.println(ESPUI.getControl(wifi_pass_text)->value);
//     unsigned int i;
//     EEPROM.begin(100);
//     for (i = 0; i < ESPUI.getControl(wifi_ssid_text)->value.length(); i++)
//     {
//       EEPROM.write(i, ESPUI.getControl(wifi_ssid_text)->value.charAt(i));
//       if (i == 30)
//         break; // Even though we provided a max length, user input should never be trusted
//     }
//     EEPROM.write(i, '\0');

//     for (i = 0; i < ESPUI.getControl(wifi_pass_text)->value.length(); i++)
//     {
//       EEPROM.write(i + 32, ESPUI.getControl(wifi_pass_text)->value.charAt(i));
//       if (i == 94)
//         break; // Even though we provided a max length, user input should never be trusted
//     }
//     EEPROM.write(i + 32, '\0');
//     EEPROM.end();
//   }
// }
