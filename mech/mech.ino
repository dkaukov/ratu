// Motor 1 = Ch1 = set 1 = Stepper 1 = "L" Motor
// Motor 2 = Ch2 = set 2 = Stepper 2 = "C1" Motor "Cold"
// Motor 3 = Ch3 = set 3 = Stepper 3 = "C2 Motor "Hot"
// Motor C: 200 steps 360 degrees revolution (no gear). In quater-step mode 800 steps 360 degrees revolution (no gear). 
// Measured 2964 steps 360 degrees revolution.

#include "proto/device_id.h"  // communication protocol device ID 
#include <AccelStepper.h>     // Accel Stepper library

#define PJON_ID ID_MECH       // PJON definition (communication protocol)
#include "proto/common.h"     // communication protocol library

#include "adc.h"

AccelStepper stepperL  = AccelStepper(1, 3, 4);  // Custom pinout "L" - Step to D3, Dir to D4 (Default AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5)
AccelStepper stepperC1 = AccelStepper(1, 5, 6);  // Custom pinout "C1" - Step to D5, Dir to D6 (Default AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5)
AccelStepper stepperC2 = AccelStepper(1, 7, 8);  // Custom pinout "C2" - Step to D7, Dir to D8 (Default AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5)
PROTO_MechStatus status;
uint8_t isAutoTune = 0;

const byte ledPin = 13;     // Initialise LED for indication
const byte optInpinL = A1;  // Signal pin Optical Interruptor Motor 1
const byte optInpinC1 = A2;  // Signal pin Optical Interruptor Motor 2
const byte optInpinC2 = A3;  // Signal pin Optical Interruptor Motor 3 C2


void calibrate(AccelStepper *chanel, uint8_t sensorPin, boolean run) {
  long oldPosition = chanel->currentPosition();
  chanel->setSpeed(-2000);
  while (digitalRead(sensorPin) == LOW) {
    chanel->runSpeed();
    busLoop();
  }
  chanel->stop();
  chanel->setCurrentPosition(0);
  if (run) {
    chanel->moveTo(oldPosition);
  }
}

void inline calibrateL(boolean run) {
  calibrate(&stepperL, optInpinL, run);
}

void inline calibrateC1(boolean run) {
  calibrate(&stepperC1, optInpinC1, run);
}

void inline calibrateC2(boolean run) {
  calibrate(&stepperC2, optInpinC2, run);
}

void busReceiver(const TCommand *payload, const PJON_Packet_Info &packet_info) {
	switch (payload->id) {
	case cmdCalibrate:
		switch (payload->cal.channel) {
		case channelC1:
			calibrateC1(true);
			break;
		case channelC2:
			calibrateC2(true);
			break;
		case channelL:
			calibrateL(true);
			break;
		}
		break;
	case cmdSetPos:
		stepperL.moveTo(stepperL.targetPosition() + payload->pos.lPos);
		stepperC1.moveTo(stepperC1.targetPosition() + payload->pos.c1Pos);
		stepperC2.moveTo(stepperC2.targetPosition() + payload->pos.c2Pos);
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
  Serial.begin(TS_PORT_BITRATE);
  busInit(busReceiver, 2, &Serial);
  adcInit();

  pinMode(ledPin, OUTPUT);               //  Defines LED
  pinMode(optInpinL, INPUT);             //  Defines Optical command PIN "L"
  pinMode(optInpinC1, INPUT);            //  Defines Optical command PIN "C"
  pinMode(optInpinC2, INPUT);            //  Defines Optical command PIN "C2"

  stepperL.setMaxSpeed(2000);            //  Set maximum roration speed for "L" Motor 1
  stepperL.setSpeed(1500);               //  Set maximum calibration speed for "L" Motor 1
  stepperL.setAcceleration(2000);        //  Set maximum acceleration for "L" Motor 1

  stepperC1.setMaxSpeed(2000);           //  Set maximum roration speed for "C" Motor 2
  stepperC1.setSpeed(1000);              //  Set maximum calibration speed for "C" Motor 2
  stepperC1.setAcceleration(2000);       //  Set maximum acceleration for "C" Motor 2

  stepperC2.setMaxSpeed(2000);           //  Set maximum roration speed for "C" Motor 3
  stepperC2.setSpeed(1000);              //  Set maximum calibration speed for "C" Motor 3
  stepperC2.setAcceleration(2000);       //  Set maximum acceleration for "C" Motor 3

  stepperL.setCurrentPosition(0);        //  Set "Zero" position "L" Motor 1
  stepperC1.setCurrentPosition(0);       //  Set "Zero" position "C" Motor 2
  stepperC2.setCurrentPosition(0);       //  Set "Zero" position "C" Motor 3
  calibrateL(false);                     //  Calibration function "L" Motor 1
  calibrateC1(false);                    //  Calibration function "C" Motor 2
  calibrateC2(false);                    //  Calibration function "C" Motor 3
  stepperL.moveTo(3000);
  stepperC1.moveTo(10);
  stepperC2.moveTo(10);
}

void yeld() {
  while (stepperL.isRunning() || stepperC1.isRunning() || stepperC2.isRunning()) {
      busLoop();
      adcLoop();
      sendStatusUpdates();
      stepperL.run();
      stepperC1.run();
      stepperC2.run();
  }
  adcCnt = 0;
  while (adcCnt < 64) {
    busLoop();
    adcLoop();
    sendStatusUpdates();
  }
}

void optimize(AccelStepper *chanel, int16_t step, int16_t hysteresis) {
  boolean isFirstStep = true;
  yeld();
  uint16_t prevSetepPwr = rflPwrVal;
  while (step != 0) {
    chanel->move(step);
    yeld();
    if (prevSetepPwr < (rflPwrVal + hysteresis)) {
      step = -step;
      if (!isFirstStep) {
        step = step >> 1;
      }
    }
    prevSetepPwr = rflPwrVal;
    if ((fwdPwrVal == 0) || (rflPwrVal == 0)) {
      break;
    }
    isFirstStep = false;
  }
}

void autoTune() {
  if ((fwdPwrVal == 0) || (isAutoTune)) {
    return;
  }
  isAutoTune = 1;
  stepperC1.moveTo(10);
  stepperC2.moveTo(10);
  stepperL.setAcceleration(20000);
  optimize(&stepperL, 1000, 0);
  optimize(&stepperC1, 20, 0);
  optimize(&stepperC2, 20, 0);
  isAutoTune = 0;
}

void fineTune() {
  if ((fwdPwrVal == 0) || (isAutoTune)) {
    return;
  }
  isAutoTune = 1;
  optimize(&stepperC1, 10, 0);
  optimize(&stepperC2, 10, 0);
  optimize(&stepperC1, 10, 0);
  isAutoTune = 0;
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
  status.pos.lPos = stepperL.currentPosition();
  status.pos.c1Pos = stepperC1.currentPosition();
  status.pos.c2Pos = stepperC2.currentPosition();
  status.flags = stepperL.isRunning() | (stepperL.isRunning() << 1) | (stepperC2.isRunning() << 2) | (isAutoTune << 3);
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
  stepperL.run();
  stepperC1.run();
  stepperC2.run();
  digitalWrite(ledPin, digitalRead(optInpinL));
  sendStatusUpdates();
}
