// Motor 1 = Ch1 = set 1 = Stepper 1 = "L" Motor
// Motor 2 = Ch2 = set 2 = Stepper 2 = "C1" Motor "Cold"
// Motor 3 = Ch3 = set 3 = Stepper 3 = "C2 Motor "Hot"
// Motor C: 200 steps 360 degrees revolution (no gear). In quater-step mode 800 steps 360 degrees revolution (no gear). 
// Measured 2964 steps 360 degrees revolution.

#define MECH_DEBUG

#include "proto/device_id.h"  // communication protocol device ID 
#include <AccelStepper.h>     // Accel Stepper library

#define PJON_ID ID_MECH       // PJON definition (communication protocol)
#include "proto/common.h"     // communication protocol library

#include "adc.h"
#include "debug.h"

AccelStepper stepperL  = AccelStepper(1, 3, 4);  // Custom pinout "L" - Step to D3, Dir to D4 (Default AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5)
AccelStepper stepperC1 = AccelStepper(1, 5, 6);  // Custom pinout "C1" - Step to D5, Dir to D6 (Default AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5)
AccelStepper stepperC2 = AccelStepper(1, 7, 8);  // Custom pinout "C2" - Step to D7, Dir to D8 (Default AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5)
PROTO_MechStatus status;
uint8_t isAutoTune = 0;
uint8_t isDriversEnabled = 1;

const byte ledPin = 13;     // Initialise LED for indication
const byte driverEnablePin = 12;
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

inline void driversEnable() {
  if (!isDriversEnabled) {
    digitalWrite(driverEnablePin, LOW);
    delay(5);
    isDriversEnabled = 1;
  }
}

inline void driversDisable() {
  digitalWrite(driverEnablePin, HIGH);
  isDriversEnabled = 0;
}

void busReceiver(char cmd, uint8_t length, const TCommand *payload) {
	switch (cmd) {
	case cmdCalibrate:
	  driversEnable();
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
	  driversEnable();
		stepperL.moveTo(stepperL.targetPosition() + payload->pos.lPos);
		stepperC1.moveTo(stepperC1.targetPosition() + payload->pos.c1Pos);
		stepperC2.moveTo(stepperC2.targetPosition() + payload->pos.c2Pos);
		break;
	case cmdAutoTune:
	  driversEnable();
	  autoTune();
	  break;
	case cmdFineTune:
	  driversEnable();
	  fineTune();
	  break;
	case cmdStatusReq:
	  halSendStatusUpdateReply(status);
	  break;
	}
}

void setup() {
  Serial.begin(TS_PORT_BITRATE);
  busInit(busReceiver, 2, &Serial);
  adcInit();
  debugInit();

  pinMode(driverEnablePin, OUTPUT);
  digitalWrite(driverEnablePin, LOW);
  pinMode(ledPin, OUTPUT);               //  Defines LED
  pinMode(optInpinL, INPUT);             //  Defines Optical command PIN "L"
  pinMode(optInpinC1, INPUT);            //  Defines Optical command PIN "C"
  pinMode(optInpinC2, INPUT);            //  Defines Optical command PIN "C2"

  stepperL.setMaxSpeed(2000);            //  Set maximum roration speed for "L" Motor 1
  stepperL.setSpeed(1500);               //  Set maximum calibration speed for "L" Motor 1
  stepperL.setAcceleration(4000);        //  Set maximum acceleration for "L" Motor 1

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
      debugLoop();
  }
  adcCnt = 0;
  while (adcCnt < adcLpfRaiseCnt()) {
    busLoop();
    adcLoop();
    sendStatusUpdates();
    debugLoop();
  }
}

void optimize(AccelStepper *chanel, int16_t step, int16_t hysteresis) {
  uint8_t stepCount = 0;
  yeld();
  uint16_t prevSetepPwr = rflPwrPercent;
  dprintf("optimize() - initial rflPwrPercent = %d\n", rflPwrPercent);		
  while (step != 0) {
    if ((fwdPwrVal >> 4) == 0) {
      dprintf("optimize() - aborting due power conditions.\n");
      break;
    }
     if ((rflPwrVal >> 4) == 0) {
      dprintf("optimize() - aborting due rflPwrVal precision.\n");
      break;
    }	  	  
    chanel->move(step);
    yeld();
    dprintf("optimize() - [%d]: step=%d, P(n-1)=%d, P(n)=%d\n", stepCount, step, prevSetepPwr, rflPwrPercent);	  
    if (prevSetepPwr < (rflPwrPercent + hysteresis)) {
      step = -step;
      if (stepCount != 0) {
        step = step >> 1;
      } else {
        dprintf("optimize() - wrong direction of 1st step\n");
      }
    }
    prevSetepPwr = rflPwrPercent;
    stepCount++;
  }
  dprintf("optimize() - finished in %d step(s), P(n)=%d\n", stepCount, rflPwrPercent);
}

void autoTune() {
  if (((fwdPwrVal >> 4) == 0) || (isAutoTune)) {
    dprintf("autoTune() - rejected\n");
    return;
  }
  uint32_t startedTime = micros();
  dprintf("autoTune() - started\n");
  isAutoTune = 1;
  stepperC1.moveTo(10);
  stepperC2.moveTo(10);
  optimize(&stepperL, 2000, 0);
  dprintf("autoTune() - stepperL finished in %8d ms\n", (uint32_t) micros() - startedTime);
  optimize(&stepperC1, 20, 0);
  dprintf("autoTune() - stepperC1 finished in %8d ms\n", (uint32_t) micros() - startedTime);
  optimize(&stepperC2, 20, 0);
  dprintf("autoTune() - stepperC2 finished in %8d ms\n", (uint32_t) micros() - startedTime);
  isAutoTune = 0;
}

void fineTune() {
  if (((fwdPwrVal >> 4) == 0) || (isAutoTune)) {
      dprintf("fineTune() - rejected\n");
      return;
    }
    uint32_t startedTime = micros();
    dprintf("fineTune() - started\n");
    isAutoTune = 1;
    stepperL.moveTo(3000);
    stepperC1.moveTo(800);
    stepperC2.moveTo(100);
    optimize(&stepperC1, 20, 0);
    dprintf("fineTune() - stepperC1 finished in %8d ms\n", (uint32_t) micros() - startedTime);
    optimize(&stepperC2, 20, 0);
    dprintf("fineTune() - stepperC2 finished in %8d ms\n", (uint32_t) micros() - startedTime);
    optimize(&stepperL, 2000, 0);
    dprintf("fineTune() - stepperL finished in %8d ms\n", (uint32_t) micros() - startedTime);
    optimize(&stepperC1, 10, 0);
    dprintf("fineTune() - stepperC1 finished in %8d ms\n", (uint32_t) micros() - startedTime);
    optimize(&stepperC2, 10, 0);
    dprintf("fineTune() - stepperC2 finished in %8d ms\n", (uint32_t) micros() - startedTime);
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
	static const unsigned long REFRESH_INTERVAL = 250; // ms
		static unsigned long lastRefreshTime = 0;
		if(millis() - lastRefreshTime >= REFRESH_INTERVAL)
		{
			lastRefreshTime = millis();
			updateStatus();
			halSendStatusUpdate(status);
		}
}

void managePower() {
  static const unsigned long REFRESH_INTERVAL = 200; // ms
    static unsigned long lastRefreshTime = 0;
    if(millis() - lastRefreshTime >= REFRESH_INTERVAL)
    {
      lastRefreshTime = millis();
      if (!stepperL.isRunning() && !stepperL.isRunning() && !stepperC2.isRunning()) {
        driversDisable();
      }
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
  managePower();
  debugLoop();
}
