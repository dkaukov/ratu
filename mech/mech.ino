// Motor 1 = Ch1 = set 1 = Stepper 1 = "L" Motor
// Motor 2 = Ch2 = set 2 = Stepper 2 = "C" Motor
// Motor C: 200 steps 360 degrees revolution (no gear). In quater-step mode 800 steps 360 degrees revolution (no gear). 
// Measured 2964 steps 360 degrees revolution.

#include "proto/device_id.h"
#include <AccelStepper.h>   // Accel Stepper library

#define PJON_ID ID_MECH
#include "proto/common.h"

AccelStepper stepper1 = AccelStepper(1, 3, 4);  // Custom pinout "L" - Step to D3, Dir to D4 (Default AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5)
AccelStepper stepper2 = AccelStepper(1, 5, 6);  // Custom pinout "C" - Step to D5, Dir to D6 (Default AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5)

const byte ledPin = 13;     // Initialise LED for indication
const byte optInpin1 = A1;  // Signal pin Optical Interruptor Motor 1
const byte optInpin2 = A2;  // Signal pin Optical Interruptor Motor 2


void busReceiver(const TCommand *payload, const PJON_Packet_Info &packet_info) {
	switch (payload->id) {
	case cmdCalibrate:
		switch (payload->cal.channel) {
		case channelC:
			calibrate2();
			break;
		case channelL:
			calibrate1();
			break;
		}
		break;
	case cmdSetPos:
		stepper1.moveTo(payload->pos.lPos);
		stepper2.moveTo(payload->pos.cPos);
		break;
	}
}

void setup()
{
  Serial.begin(9600);                   // Start Serial
  busInit(busReceiver);
  pinMode(ledPin, OUTPUT);              // Defines LED
  pinMode(optInpin1, INPUT);            //  Defines Optical command PIN "L"
  pinMode(optInpin2, INPUT);            //  Defines Optical command PIN "C"

  stepper1.setMaxSpeed(2000);           //  Set maximum roration speed for "L" Motor 1
  stepper1.setSpeed(500);               //  Set maximum calibration speed for "L" Motor 1
  stepper1.setAcceleration(2000);       //  Set maximum acceleration for "L" Motor 1

  stepper2.setMaxSpeed(2000);           //  Set maximum roration speed for "C" Motor 2
  stepper2.setSpeed(500);               //  Set maximum calibration speed for "C" Motor 2
  stepper2.setAcceleration(2000);       //  Set maximum acceleration for "C" Motor 2

  stepper1.setCurrentPosition(0);       // Set "Zero" position "L" Motor 1
  stepper2.setCurrentPosition(0);       // Set "Zero" position "C" Motor 2
  calibrate1();                         // Calibration function "L" Motor 1
  calibrate2();                         // Calibration function "C" Motor 2
}

// Calibration process for "L" Motor 1
void calibrate1() {
  long oldPosition = stepper1.currentPosition();
  stepper1.setSpeed(-600);
  while (digitalRead(optInpin1) == LOW) {
    stepper1.runSpeed();
  }
  stepper1.stop();
  stepper1.setCurrentPosition(0);
  // stepper1.moveTo(oldPosition);
}

// Calibration process for "C" Motor 2
void calibrate2() {
  long oldPosition = stepper2.currentPosition();
  stepper2.setSpeed(-600);
  while (digitalRead(optInpin2) == LOW) {
    stepper2.runSpeed();
  }
  stepper2.stop();
  stepper2.setCurrentPosition(0);
  // stepper2.moveTo(oldPosition);
}

void loop() {
  busLoop();
  stepper1.run();
  stepper2.run();
  digitalWrite(ledPin, digitalRead(optInpin1));
}
