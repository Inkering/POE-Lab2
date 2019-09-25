
/*
 * DIY 3D Scanner Controller
 * By Sam Daitzman and Dieter Brehm
 */
#include "Arduino.h"
#include "Servo.h"

Servo yawServo;  // create servo object to control a servo
Servo pitchServo;

const int IRPin = 0;
const float alpha = 0.3;
int IRVal = 0;
uint8_t distance;
unsigned long lastTime;

// run time for experiment
unsigned long runTime;

int incomingByte = 0;
int runProgram = 0;

void setup()
{
		pinMode(IRPin, INPUT);
		yawServo.attach(3);
		pitchServo.attach(6);
		Serial.begin(115200);
}

float sensorRead() {
		// get data from the sensor

		// get current IR Reading
		int IRValReading = analogRead(IRPin);

		if (IRValReading == 0) {
				IRValReading = 1;
		}

		float dist = IRValReading;
		return dist;
}

void run_3d() {
		// reset servos
		yawServo.write(0);
		pitchServo.write(0);
		// yaw movement (theta)
		for(int i=0; i <= 180; i+= 1) {
				// write angle to servo
				yawServo.write(i);
				delay(200);
				// pitch movement (phi)
				for(int j=0; j <=  180; j+= 2) {
						// get time for relative time plotting
						runTime = millis();

						// logging
						Serial.print(runTime);
						Serial.print(",");
						Serial.print(sensorRead());
						Serial.print(",");
						Serial.print(i);
						Serial.print(",");
						Serial.println(j);

						//write angle to servo
						pitchServo.write(j);
						delay(60);
				}
				// smoothly move back up, just for the looks!
				for(int j=0; j >=  180; j-= 2) {
						//write angle to servo
						pitchServo.write(j);
						delay(60);
				}
		}
}

void run_2d() {
		// reset servos
		yawServo.write(0);
		pitchServo.write(110);

		// yaw movement
		for (int i = 0; i < 180; i += 1) {
				// write angle to servo
				yawServo.write(i);
				delay(200);

				// get time for relative time plotting
				runTime = millis();

				// logging
				Serial.print(runTime);
				Serial.print(",");
				Serial.print(sensorRead());
				Serial.print(",");
				Serial.print(i);
				Serial.print(",");
				Serial.println(0);
		}
		// send a stop signal to the python program
}

void servo_test(){
    // reset servos
    yawServo.write(0);
		pitchServo.write(0);
		delay(4000);
		yawServo.write(90);
		pitchServo.write(90);
		delay(4000);
		yawServo.write(180);
		pitchServo.write(180);
		delay(4000);
}

void sensor_test() {
		yawServo.write(90);
		pitchServo.write(110);
		Serial.println(sensorRead());
		delay(300);
}

void loop() {
		// check for run signal
		// does it block at Serial.available?
		if (Serial.available() > 0 && runProgram != 1) {
				runProgram = 1;
		}

		// run the program if a run signal was received
		if (runProgram == 1) {
				//run_3d();
				// run_2d();
				//servo_test();
				sensor_test();
		}
}
