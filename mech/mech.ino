// Motor 1 = Ch1 = set 1 = Stepper 1 = "L" Motor
// Motor 2 = Ch2 = set 2 = Stepper 2 = "C1" Motor "Cold"
// Motor 3 = Ch3 = set 3 = Stepper 3 = "C2 Motor "Hot"
// Motor C: 200 steps 360 degrees revolution (no gear). In quater-step mode 800 steps 360 degrees revolution (no gear). 
// Measured 2964 steps 360 degrees revolution.

#include "proto/device_id.h"  // communication protocol device ID 
#include <AccelStepper.h>     / Accel Stepper library

#define PJON_ID ID_MECH       // PJON definition (communication protocol)
#include "proto/common.h"     // communication protocol library

AccelStepper stepper1 = AccelStepper(1, 3, 4);  // Custom pinout "L" - Step to D3, Dir to D4 (Default AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5)
AccelStepper stepper2 = AccelStepper(1, 5, 6);  // Custom pinout "C1" - Step to D5, Dir to D6 (Default AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5)
AccelStepper stepper3 = AccelStepper(1, 7, 8);  // Custom pinout "C2" - Step to D7, Dir to D8 (Default AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5)
PROTO_MechStatus status;

const byte ledPin = 13;     // Initialise LED for indication
const byte optInpin1 = A1;  // Signal pin Optical Interruptor Motor 1
const byte optInpin2 = A2;  // Signal pin Optical Interruptor Motor 2
const byte optInpin3 = A3;  // Signal pin Optical Interruptor Motor 3 C2

const byte fwdPwr = A5;     // Fwd signal from SWR sensor
const byte rflPwr = A6;     // Rev. signal from SWR sensor
const float adcFiltFactor = 0.99;

volatile uint8_t adcPinCurr;
volatile float fwdPwrVal = 0.0;
volatile float rflPwrVal = 0.0;


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

// ADC complete ISR
ISR (ADC_vect) {
  if (adcPinCurr == fwdPwr) {
	  fwdPwrVal = (1 - adcFiltFactor) * ADC + fwdPwrVal * adcFiltFactor;
	  adcPinCurr = rflPwr;
  }
  if (adcPinCurr == rflPwr) {
  	  rflPwrVal = (1 - adcFiltFactor) * ADC + rflPwrVal * adcFiltFactor;
  	  adcPinCurr = fwdPwr;
    }
  ADMUX =  bit(REFS0) | (adcPinCurr & 0x07);
  ADCSRA |= bit(ADSC) | bit(ADIE);
}

void setup()
{
  ADCSRA = bit(ADEN);                   // turn ADC on
  ADCSRA &= ~(bit(ADPS0) |
		  bit(ADPS1) |
		  bit(ADPS2));                  // clear prescaler bits
  ADCSRA |= bit(ADPS2);                 //  16
  adcPinCurr = fwdPwr;
  ADMUX  =  bit(REFS0) | (adcPinCurr & 0x07);
  ADCSRA |= bit(ADSC) | bit(ADIE);

  Serial.begin(115200);                 // Start Serial
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
	status.adc.fwd = round(fwdPwrVal);
	status.adc.rfl = round(rflPwrVal);
	status.flags = stepper1.isRunning() || stepper1.isRunning() << 1 || stepper3.isRunning() << 2;
	status.cnt++;
}

void sendStatusUpdates() {
	static const unsigned long REFRESH_INTERVAL = 1000; // ms
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
  stepper1.run();
  stepper2.run();
  stepper3.run();
  digitalWrite(ledPin, digitalRead(optInpin1));
  sendStatusUpdates();
}
