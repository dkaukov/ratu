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
AccelStepper stepper3 = AccelStepper(1, 7, 8);  // Custom pinout "C" - Step to D7, Dir to D8 (Default AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5)
TCommand status;

const byte ledPin = 13;     // Initialise LED for indication
const byte optInpin1 = A1;  // Signal pin Optical Interruptor Motor 1
const byte optInpin2 = A2;  // Signal pin Optical Interruptor Motor 2
const byte optInpin3 = A3;  // Signal pin Optical Interruptor Motor 3 C2

const byte fwdPwr = A5;     // Fwd signal from SWR sensor
const byte rflPwr = A6;     // Rev. signal from SWR sensor

void busReceiver(const TCommand *payload, const PJON_Packet_Info &packet_info) {
	switch (payload->id) {
	case cmdCalibrate:
		switch (payload->cal.channel) {
		case channelC1:
			calibrate2(true);
			break;
		case channelC2:
			calibrate3(true);
			break;
		case channelL:
			calibrate1(true);
			break;
		}
		break;
	case cmdSetPos:
		stepper1.moveTo(payload->pos.lPos);
		stepper2.moveTo(payload->pos.c1Pos);
		stepper3.moveTo(payload->pos.c2Pos);
		break;
	}
}

void setup()
{
  ADCSRA = (ADCSRA & 0xf8) | 0x04;      // Fast ADC

  Serial.begin(9600);                   // Start Serial
  busInit(busReceiver);

  pinMode(fwdPwr, INPUT);
  pinMode(rflPwr, INPUT);

  pinMode(ledPin, OUTPUT);              // Defines LED
  pinMode(optInpin1, INPUT);            //  Defines Optical command PIN "L"
  pinMode(optInpin2, INPUT);            //  Defines Optical command PIN "C"
  pinMode(optInpin3, INPUT);            //  Defines Optical command PIN "C2"

  stepper1.setMaxSpeed(2000);           //  Set maximum roration speed for "L" Motor 1
  stepper1.setSpeed(500);               //  Set maximum calibration speed for "L" Motor 1
  stepper1.setAcceleration(2000);       //  Set maximum acceleration for "L" Motor 1

  stepper2.setMaxSpeed(2000);           //  Set maximum roration speed for "C" Motor 2
  stepper2.setSpeed(500);               //  Set maximum calibration speed for "C" Motor 2
  stepper2.setAcceleration(2000);       //  Set maximum acceleration for "C" Motor 2

  stepper3.setMaxSpeed(2000);           //  Set maximum roration speed for "C" Motor 3
  stepper3.setSpeed(500);               //  Set maximum calibration speed for "C" Motor 3
  stepper3.setAcceleration(2000);       //  Set maximum acceleration for "C" Motor 3

  stepper1.setCurrentPosition(0);       // Set "Zero" position "L" Motor 1
  stepper2.setCurrentPosition(0);       // Set "Zero" position "C" Motor 2
  stepper3.setCurrentPosition(0);       // Set "Zero" position "C" Motor 3
  calibrate1(false);                    // Calibration function "L" Motor 1
  calibrate2(false);                    // Calibration function "C" Motor 2
  //calibrate3(false);                  // Calibration function "C" Motor 3
}

// Calibration process for "L" Motor 1
void calibrate1(boolean run) {
  long oldPosition = stepper1.currentPosition();
  stepper1.setSpeed(-600);
  while (digitalRead(optInpin1) == LOW) {
    stepper1.runSpeed();
    busLoop();
  }
  stepper1.stop();
  stepper1.setCurrentPosition(0);
  if (run) {
    stepper1.moveTo(oldPosition);
  }
}

// Calibration process for "C" Motor 2
void calibrate2(boolean run) {
  long oldPosition = stepper2.currentPosition();
  stepper2.setSpeed(-600);
  while (digitalRead(optInpin2) == LOW) {
    stepper2.runSpeed();
    busLoop();
  }
  stepper2.stop();
  stepper2.setCurrentPosition(0);
  if (run) {
    stepper2.moveTo(oldPosition);
  }
}

// Calibration process for "C" Motor 3
void calibrate3(boolean run) {
  long oldPosition = stepper3.currentPosition();
  stepper3.setSpeed(-600);
  while (digitalRead(optInpin3) == LOW) {
    stepper3.runSpeed();
    busLoop();
  }
  stepper3.stop();
  stepper3.setCurrentPosition(0);
  if (run) {
    stepper3.moveTo(oldPosition);
  }

}

void updateStatus() {
	status.id = cmdStatus;
	analogRead(fwdPwr);	status.status.adc.fwd = analogRead(fwdPwr);
	analogRead(rflPwr);	status.status.adc.rfl = analogRead(rflPwr);
	status.status.flags = stepper1.isRunning() || stepper1.isRunning() << 1 || stepper3.isRunning() << 2;
	status.status.pos.lPos  = stepper1.currentPosition();
	status.status.pos.c1Pos = stepper2.currentPosition();
	status.status.pos.c2Pos = stepper3.currentPosition();
	Serial.print("Value fwd=");
	Serial.println(status.status.adc.fwd);
	Serial.print("Value rfl=");
	Serial.println(status.status.adc.rfl);

	float rfl = status.status.adc.rfl;
	float fwd = status.status.adc.fwd;
	float p = sqrt(rfl / fwd);
	float valueSWR = (1 + p) / (1 - p);
	Serial.print("Value SWR=");
	Serial.println(valueSWR);

}

void sendStatusUpdates() {
	static const unsigned long REFRESH_INTERVAL = 1000; // ms
		static unsigned long lastRefreshTime = 0;
		if(millis() - lastRefreshTime >= REFRESH_INTERVAL)
		{
			lastRefreshTime = millis();
			updateStatus();
			bus.send_packet_blocking(ID_HAL100, (char *)&status, sizeof(status));
		}
}

void loop() {
  busLoop();
  stepper1.run();
  stepper2.run();
  stepper3.run();
  digitalWrite(ledPin, digitalRead(optInpin1));
  sendStatusUpdates();
}
