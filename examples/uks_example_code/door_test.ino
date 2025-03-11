#include <Servo.h>

Servo myservo; // create servo object to control a servo
// twelve servo objects can be created on most boards
String readString = "";
const int DOOR_OPEN = 120;
const int DOOR_CLOSED = 33;
int new_pos = DOOR_CLOSED;
int pos = 0; // variable to store the servo position

void setup() {
	myservo.attach(9); // attaches the servo on pin 9 to the servo object
	Serial.begin(9600);
	myservo.write(DOOR_CLOSED);
	pos = DOOR_CLOSED;
	delay(100);
	Serial.print("Servo moved to: ");
	Serial.println(pos);
}

void slowly(int to_pos, int delay_ms) {
	if (to_pos > pos) {
		for (int x = pos; x <= to_pos; x++) {
			myservo.write(x);
			pos = x;
			delay(delay_ms);
		}
	} else {
		for (int x = pos; x >= to_pos; x--) {
			myservo.write(x);
			pos = x;
			delay(delay_ms);
		}
	}
}

void loop() {
	while (Serial.available()) {
		delay(3); // delay to allow buffer to fill
		if (Serial.available() > 0) {
			char c = Serial.read(); // gets one byte from serial buffer
			readString += c;        // makes the string readString
		}
	}
	if (readString.length() > 0) { // Check if data is available to read
		Serial.print("read ");
		Serial.println(readString);
		if (readString.substring(0, 4) == "open") {
			slowly(DOOR_OPEN, 10);
		} else if (readString.substring(0, 4) == "demo") {
			delay(3000);
			slowly(DOOR_OPEN, 10);
			delay(2000);
			slowly(DOOR_CLOSED, 20);
		} else if (readString.substring(0, 5) == "close") {
			slowly(DOOR_CLOSED, 20);
		} else {
			new_pos = readString.toInt();         // Read the integer value from the serial port
			if (new_pos >= 0 && new_pos <= 180) { // Ensure the angle is within the valid range for a servo
				delay(1000);
				myservo.write(new_pos); // Move the servo to the specified angle
				pos = new_pos;
				Serial.print("Servo moved to: ");
				Serial.println(pos); // Print the angle to the serial monitor
			} else {
				Serial.println("Invalid angle. Please enter a value between 0 and 180.");
			}
		}
	}
	readString = "";
}
