// Motor 1 = Ch1 = set 1 = Stepper 1 = "L" Motor
// Motor 2 = Ch2 = set 2 = Stepper 2 = "C" Motor
// Motor C: 200 steps 360 degrees revolution (no gear). In quater-step mode 800 steps 360 degrees revolution (no gear). 
// Measured 2964 steps 360 degrees revolution.
#include "proto/device_id.h"
#include <AccelStepper.h>   // Accel Stepper library
#include <CmdMessenger.h>   // CmdMessenger library

#define PJON_ID ID_MECH
#include "proto/common.h"

AccelStepper stepper1 = AccelStepper(1, 3, 4);  // Custom pinout "L" - Step to D3, Dir to D4 (Default AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5)
AccelStepper stepper2 = AccelStepper(1, 5, 6);  // Custom pinout "C" - Step to D5, Dir to D6 (Default AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5)
CmdMessenger cmdMessenger = CmdMessenger(Serial);

const byte ledPin = 13;     // Initialise LED for indication
const byte optInpin1 = A1;  // Signal pin Optical Interruptor Motor 1
const byte optInpin2 = A2;  // Signal pin Optical Interruptor Motor 2

// int byteRead;   // int for communication via TX-RX

enum
{
  // Commands
  kAcknowledge,                         // Command to acknowledge that cmd was received
  kSetCh1,                              // Command to move "L" Motor 1: syntax [1,1000;] to move Motor 1000 steps CW; syntax [1,-1000;] to move Motor 1000 steps CCW
  kSetCh2,                              // Command to move "C" Motor 2: syntax [2,1000;] to move Motor 1000 steps CW; syntax [2,-1000;] to move Motor 1000 steps CCW
  kCalibrateCh1,                        // Command to calibrate "L" Motor 1: syntax [3;]
  kCalibrateCh2,                        // Command to calibrate "C" Motor 2: syntax [4;]
  kError                                // Command to report errors
};

void attachCommandCallbacks()
{
  // Attach callback methods
  cmdMessenger.attach(OnUnknownCommand);
  cmdMessenger.attach(kCalibrateCh1, calibrate1);   // Calibrate Motor 1
  cmdMessenger.attach(kCalibrateCh2, calibrate2);   // Calibrate Motor 2
  cmdMessenger.attach(kSetCh1, set1);               // Move Motor 1 to absolute position in steps
  cmdMessenger.attach(kSetCh2, set2);               // Move Motor 2 to absolute position in steps

}

void OnUnknownCommand()
{
  cmdMessenger.sendCmd(kError, "Command without attached callback");  //Displayed in Console if command has no callback defined
}

// Callback function that responds that Arduino is ready (has booted up)
void OnArduinoReady()
{
  cmdMessenger.sendCmd(kAcknowledge, "Arduino ready");
}

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

  // Adds newline to every command
  cmdMessenger.printLfCr();

  // Attach my application's user-defined callback methods
  attachCommandCallbacks();

  // Send the status to the Console that says the both Motors finished calibration cycle
  cmdMessenger.sendCmd(kAcknowledge, "Calibration complete");

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
// Move to position process for "L" Motor 1
void set1() {
  stepper1.moveTo(cmdMessenger.readInt32Arg());
}
// Move to position process for "C" Motor 2
void set2() {
  stepper2.moveTo(cmdMessenger.readInt32Arg());
}


void loop() {
  busLoop();
  // Process incoming serial data, and perform callbacks
  cmdMessenger.feedinSerialData();
  stepper1.run();
  stepper2.run();
  digitalWrite(ledPin, digitalRead(optInpin1));

//  while (Serial.available()) {
 //   byteRead = Serial.read();
//  }
}
