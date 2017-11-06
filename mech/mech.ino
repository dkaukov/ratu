// Motor 1 = Ch1 = set 1 = Stepper 1 = "L" Motor
// Motor 2 = Ch2 = set 2 = Stepper 2 = "C1" Motor "Cold"
// Motor 3 = Ch3 = set 3 = Stepper 3 = "C2 Motor "Hot"
// Motor C: 200 steps 360 degrees revolution (no gear). In quater-step mode 800 steps 360 degrees revolution (no gear). 
// Measured 2964 steps 360 degrees revolution.

#include "proto/device_id.h"  // communication protocol device ID 
#include <AccelStepper.h>     / Accel Stepper library

#define PJON_ID ID_MECH       // PJON definition (communication protocol)
#include "proto/common.h"     // communication protocol library

#include "adc.h"

AccelStepper stepper1 = AccelStepper(1, 3, 4);  // Custom pinout "L" - Step to D3, Dir to D4 (Default AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5)
AccelStepper stepper2 = AccelStepper(1, 5, 6);  // Custom pinout "C1" - Step to D5, Dir to D6 (Default AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5)
AccelStepper stepper3 = AccelStepper(1, 7, 8);  // Custom pinout "C2" - Step to D7, Dir to D8 (Default AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5)
PROTO_MechStatus status;
uint8_t isAutoTune = 0;

const byte ledPin = 13;     // Initialise LED for indication
const byte optInpin1 = A1;  // Signal pin Optical Interruptor Motor 1
const byte optInpin2 = A2;  // Signal pin Optical Interruptor Motor 2
const byte optInpin3 = A3;  // Signal pin Optical Interruptor Motor 3 C2

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
		stepper1.moveTo(stepper1.targetPosition() + payload->pos.lPos);
		stepper2.moveTo(stepper2.targetPosition() + payload->pos.c1Pos);
		stepper3.moveTo(stepper3.targetPosition() + payload->pos.c2Pos);
		break;
	case cmdAutoTune:
	  autoTune();
	  break;
	case cmdFineTune:
	  fineTune();
	  break;
	}
}

void setup() {
  Serial.begin(TS_PORT_BITRATE);                    // Start Serial
  busInit(busReceiver, 2, &Serial);
  adcInit();

  pinMode(ledPin, OUTPUT);              // Defines LED
  pinMode(optInpin1, INPUT);            //  Defines Optical command PIN "L"
  pinMode(optInpin2, INPUT);            //  Defines Optical command PIN "C"
  pinMode(optInpin3, INPUT);            //  Defines Optical command PIN "C2"

  stepper1.setMaxSpeed(2000);           //  Set maximum roration speed for "L" Motor 1
  stepper1.setSpeed(1500);               //  Set maximum calibration speed for "L" Motor 1
  stepper1.setAcceleration(2000);       //  Set maximum acceleration for "L" Motor 1

  stepper2.setMaxSpeed(2000);           //  Set maximum roration speed for "C" Motor 2
  stepper2.setSpeed(1000);               //  Set maximum calibration speed for "C" Motor 2
  stepper2.setAcceleration(2000);       //  Set maximum acceleration for "C" Motor 2

  stepper3.setMaxSpeed(2000);           //  Set maximum roration speed for "C" Motor 3
  stepper3.setSpeed(1000);               //  Set maximum calibration speed for "C" Motor 3
  stepper3.setAcceleration(2000);       //  Set maximum acceleration for "C" Motor 3

  stepper1.setCurrentPosition(0);       // Set "Zero" position "L" Motor 1
  stepper2.setCurrentPosition(0);       // Set "Zero" position "C" Motor 2
  stepper3.setCurrentPosition(0);       // Set "Zero" position "C" Motor 3
  calibrate1(false);                    // Calibration function "L" Motor 1
  calibrate2(false);                    // Calibration function "C" Motor 2
  calibrate3(false);                    // Calibration function "C" Motor 3
  stepper1.moveTo(3000);
  stepper2.moveTo(10);
  stepper3.moveTo(10);

}

void yeld() {
  while (stepper1.isRunning() || stepper2.isRunning() || stepper3.isRunning()) {
      busLoop();
      adcLoop();
      sendStatusUpdates();
      stepper1.run();
      stepper2.run();
      stepper3.run();
  }
  adcCnt = 0;
  while (adcCnt < 128) {
    busLoop();
    adcLoop();
    sendStatusUpdates();
  }
}

void autoTune() {
  if (fwdPwrVal == 0) {
    return;
  }
  if (isAutoTune != 0) {
      return;
  }
  isAutoTune = 1;
  stepper2.moveTo(10);
  stepper3.moveTo(10);
  stepper1.setAcceleration(20000);
  yeld();
  uint16_t prevSetepPwr = rflPwrVal;
  int16_t step = 1000;
  while (step != 0) {
    stepper1.move(step);
    yeld();
    if (prevSetepPwr < rflPwrVal) {
      step = -(step >> 1);
    }
    prevSetepPwr = rflPwrVal;
  }

  yeld();
  step = 20;
  prevSetepPwr = rflPwrVal;
  while (step != 0) {
      stepper2.move(step);
      yeld();
      if (prevSetepPwr < rflPwrVal) {
        step = -(step >> 1);
      }
      prevSetepPwr = rflPwrVal;
  }

  yeld();
  step = 20;
  prevSetepPwr = rflPwrVal;
  while (step != 0) {
      stepper3.move(step);
      yeld();
      if (prevSetepPwr < rflPwrVal) {
        step = -(step >> 1);
      }
      prevSetepPwr = rflPwrVal;
  }
  isAutoTune = 0;
}

void fineTune() {
  if (fwdPwrVal == 0) {
    return;
  }
  if (isAutoTune != 0) {
      return;
  }
  isAutoTune = 1;
  yeld();
  uint16_t prevSetepPwr = rflPwrVal;
  int16_t step = 10;
  yeld();
  prevSetepPwr = rflPwrVal;
  while (step != 0) {
    stepper2.move(step);
    yeld();
    if (prevSetepPwr < rflPwrVal) {
      step = -(step >> 1);
    }
    prevSetepPwr = rflPwrVal;
  }

  yeld();
  step = 10;
  prevSetepPwr = rflPwrVal;
  while (step != 0) {
    stepper3.move(step);
    yeld();
    if (prevSetepPwr < rflPwrVal) {
      step = -(step >> 1);
    }
    prevSetepPwr = rflPwrVal;
  }

  yeld();
  step = 10;
  prevSetepPwr = rflPwrVal;
  while (step != 0) {
    stepper2.move(step);
    yeld();
    if (prevSetepPwr < rflPwrVal) {
      step = -(step >> 1);
    }
    prevSetepPwr = rflPwrVal;
  }
  isAutoTune = 0;
}

// Calibration process for "L" Motor 1
void calibrate1(boolean run) {
  long oldPosition = stepper1.currentPosition();
  stepper1.setSpeed(-2000);
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
  stepper2.setSpeed(-2000);
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
  stepper3.setSpeed(-2000);
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
	status.adc.fwd = fwdPwrVal;
	if (status.adc.fwd > diodeDropVal) {
	  status.adc.fwd += diodeDropVal;
	}
	status.adc.rfl = rflPwrVal;
  if (status.adc.rfl > diodeDropVal) {
    status.adc.rfl += diodeDropVal;
  }
  status.pos.lPos = stepper1.currentPosition();
  status.pos.c1Pos = stepper2.currentPosition();
  status.pos.c2Pos = stepper3.currentPosition();
  status.flags = stepper1.isRunning() | (stepper1.isRunning() << 1) | (stepper3.isRunning() << 2) | (isAutoTune << 3);
	status.cnt++;
}

void sendStatusUpdates() {
	static const unsigned long REFRESH_INTERVAL = 500; // ms
		static unsigned long lastRefreshTime = 0;
		if(millis() - lastRefreshTime >= REFRESH_INTERVAL)
		{
			lastRefreshTime = millis();
			updateStatus();
			halSendStatusUpdate(status);
		}
}

void loop() {
  busLoop();
  adcLoop();
  stepper1.run();
  stepper2.run();
  stepper3.run();
  digitalWrite(ledPin, digitalRead(optInpin1));
  sendStatusUpdates();
}
