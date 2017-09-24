/*
  TFT Display 128x128 16bit color ST7735 Chipset

  TFT display pinout left to right pins up
  VCC GND GND NC NC LED CLK SDI RS RST CS

  Wiring:
  PIN TFT       PIN Mega
  VCC           5V
  GND           GND
  LED           3V3                                         LED backlight control
  CLK           D20 SCK = PIN 52                            Clock signal
  SDI           D21 MOSI = PIN 51                           Data signal
  RS            D9 defined in code below  #define dc   9    Command/Data select
  RST           D8 defined in code below  #define SRT  8    Reset
  CS            D10 defined in code below #define cs   10   Chip select

  https://www.jaycar.com.au/128x128-lcd-screen-module-for-arduino/p/XC4629
  http://www.arduino.cc/en/Tutorial/TFTDisplayText

*/
#include "proto/device_id.h"
#include <TFT.h>    // Arduino LCD library
#include <SPI.h>    // SPI library
#include <Keypad.h> // Keypad library
#include <math.h>   // Math library
#define PJON_ID ID_HAL100
#include "proto/common.h"

// TFT display CS, RS and RST pins definition for Mega2560
#define cs   10
#define dc   9
#define rst  8

// create an instance of the TFT library
TFT TFTscreen = TFT(cs, dc, rst);

// char array

char enteredFreq[6];
char tuningFreq[6];
char key;
char DisplayValueC[5];
char DisplayValueL[5];
char input; //buffer for input characters for calculations
// char dataPrintout[10];

float tuningFreqCalc = 0;   // float for value C calculation
float KHz;                  // KHz constant
float PlanckTime;           // Planck constant
float pi;                   // π constant
float valueC = 0;           // calculated value C float
float valueCEffective = 0;  // calculated value C float
float valueL = 0;           // calculated value L float
float valueCorr = 0;        // calculated value C float
float valueLcorr = 0;       // calculated value L float
float Cmult;                // constant multiplier to calculate C value
float Lmult;                // constant multiplier to calculate L value
float valueRotateC1;
float valueRotateC2;
float valueRotateL;
float valueSWR;

boolean presentValue = false;

String EnteredFreqString, TuningFreqString;
String DisplayValueLstring;

int i;


// Keypad code start
const byte ROWS = 5; // Five rows
const byte COLS = 4; // Four columns
// Define the Keymap
char keys[ROWS][COLS] = {
  {'S', '1', '2', '3'}, // ROW0-pin5      Buttons association 1=1 2=2 3=3 4=4 5=5 6=6 7=7 8=8 9=9 0=0 *=Led OFF #=Led ON
  {'K', '4', '5', '6'}, // ROW1-pin6
  {'U', '7', '8', '9'}, // ROW2-pin7      TOP Buttons  C c L l
  {'D', '*', '0', '#'}, // ROW3-pin8
  {'L', 'l', 'C', 'c'}  // ROW4-pin9      Buttons in middle H U D S
};
// COL0-pin1, COL1-pin2, COL2-pin3, COL3-pin4

// Connect keypad ROW0 Pad pin ->> 5=30 <<- pin Mega, ROW1 6=32, ROW2 7=23, ROW3 8=25, ROW4 9=27
byte rowPins[ROWS] = { 30, 32, 23, 25, 27 };
// Connect keypad COL0 Pad pin ->> 1=22 <<- pin Mega, COL1 2=24, COL2 3=26, COL3 4=28
byte colPins[COLS] = { 22, 24, 26, 28 };

// Create the Keypad
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );
// end of Keypad code

void busReceiver(const TCommand *payload, const PJON_Packet_Info &packet_info) {
	switch (payload->id) {
	case cmdStatus:

		Serial.print("cnt=");
		Serial.print(payload->status.cnt);
		Serial.print(", Value fwd=");
		Serial.print(payload->status.adc.fwd);
		Serial.print(", Value rfl=");
		Serial.println(payload->status.adc.rfl);


		float rfl = payload->status.adc.rfl;
		float fwd = payload->status.adc.fwd;
		float p = sqrt(rfl / fwd);
		valueSWR = (1 + p) / (1 - p);
		Serial.print(", Value SWR=");
		Serial.println(valueSWR);

		break;
	}
}

void setup() {
  Serial.begin(115200);

  TFTscreen.begin();                                // initialise TFT screen
  TFTscreen.background(0, 20, 30);                  // clear the screen with a RGB background

  busInit(busReceiver);
  displayInitialScreen();
}

void displayInitialScreen() {                      // displays "Wait Calibrating at start. To be linked to Motors "L" and "C" calibration process
  // Displays "Set F:" at the bottom of the screen
  TFTscreen.stroke(0, 255, 0);                      // set the font color
  TFTscreen.setTextSize(1);                         // set the font size 2
  TFTscreen.text("SET", 5, 117);                    // write the text to coordinates
  TFTscreen.setTextSize(2);
  TFTscreen.text("f:", 25, 110);
  // Displays L
  TFTscreen.stroke(0, 255, 0);                      // set the font color
  TFTscreen.setTextSize(2);                         // set the font size 2
  TFTscreen.text("Ln", 5, 40);                     // write the text to coordinates  
  // Displays C
  TFTscreen.stroke(0, 255, 0);                      // set the font color             
  TFTscreen.setTextSize(2);                         // set the font size 2
  TFTscreen.text("Cn", 5, 60);                     // write the text to coordinates

  keypad.addEventListener(keypadEvent);             //add an event listener for this keypad
}

void SetFrequencyStage1() {
	// STAGE 1 tuning process
	calculateC();                                     // calculate C value at selected frequency
	calculateStepsC();                                // calculate steps needed to move C to value C
	CmoveStage1();                                    // send command to nano to move motor "C" to C step value
	calculateL();                                     // calculate L value at given frequency and C value
	calculateStepsL();                                // calculate steps needed to move C to value L
	LmoveStage1();                                    // send command to nano to move motor "L" to L step value
	EraseDisplayC(), EraseDisplayL();                 // clear previous values L and C at display
	displayC(), displayL();                           // display current values L and C at display
	mechSetPosition(round (valueRotateL), round (valueRotateC1), round (valueRotateC2));
	// End STAGE 1 tuning process
}

void SetFrequency() {                               // when # pressed - sets frequency value at the top of the screen
  // Pre-STAGE 1 (coarse tuning process) cleanups and displays
  eraseFreqTopScreen();                             // clear previously entered drequesncy at display top
  displayFreqTopScreen();                           // display desired frequesncy at display top
  eraseEnteredFrequency();                          // clear entered frequency at display bottom
  SetFrequencyStage1();
}


void incL(float diff) {
	valueRotateL = valueRotateL + diff;
	mechSetPosition(round (valueRotateL), round (valueRotateC1), round (valueRotateC2));
}

void incC(float diff) {
	valueRotateC1 = valueRotateC1 + diff;
	mechSetPosition(round (valueRotateL), round (valueRotateC1), round (valueRotateC2));
}

void incLCorr(float diff) {
    if (tuningFreqCalc != 0.0) {
      valueLcorr = valueLcorr + diff;
      SetFrequencyStage1();
    }
}

void incCCorr(float diff) {
    if (tuningFreqCalc != 0.0) {
      valueCorr = valueCorr + diff;
      SetFrequencyStage1();
    }
}


void eraseFreqTopScreen() {                      // erases frequency value from top of display
  TFTscreen.stroke(0, 20, 30);
  TFTscreen.setTextSize(4);
  TFTscreen.text(tuningFreq, 10, 10);
  TuningFreqString = "";
}

void displayFreqTopScreen() {                   // shows frequency value at top of display font 4
  TuningFreqString = EnteredFreqString;
  TuningFreqString.toCharArray(tuningFreq, 6);
  EnteredFreqString = "";
  TFTscreen.stroke(255, 255, 0);
  TFTscreen.setTextSize(4);
  TFTscreen.text(tuningFreq, 10, 10);
}

void displayTunedFreqTopScreen() {             // Display tuningFreq at the top of the screen in "green tuned mode"
  TFTscreen.stroke(0, 255, 0);
  TFTscreen.setTextSize(4);
  TFTscreen.text(tuningFreq, 10, 10);
}

void eraseEnteredFrequency () {               // erase frequency value at the botton of the screen
  TFTscreen.setTextSize(2);
  TFTscreen.stroke(0, 20, 30);
  TFTscreen.text(enteredFreq, 50, 110);
}

void displayL() {                                         // display L values
  dtostrf(valueRotateL, 5, 2, DisplayValueL);
  TFTscreen.stroke(0, 255, 0);
  TFTscreen.setTextSize(2);
  TFTscreen.text(DisplayValueL, 40, 40);
}

void EraseDisplayL() {                                   // erase display L values
  TFTscreen.stroke(0, 20, 30);
  TFTscreen.setTextSize(2);
  TFTscreen.text(DisplayValueL, 40, 40);
}

void displayC() {                                       // display C values
  dtostrf(valueRotateC1, 5, 0, DisplayValueC);
  TFTscreen.stroke(0, 255, 0);
  TFTscreen.setTextSize(2);
  TFTscreen.text(DisplayValueC, 40, 60);
}

void EraseDisplayC() {                                   // erase display C values
  TFTscreen.stroke(0, 20, 20);
  TFTscreen.setTextSize(2);
  TFTscreen.text(DisplayValueC, 40, 60);
}

void eraseFrequency() {                         // when * pressed - erases entered frequency bottom of the screen
  EnteredFreqString = "";
  TuningFreqString = "";
  TFTscreen.stroke(0, 20, 30);
  TFTscreen.setTextSize(2);
  TFTscreen.text(enteredFreq, 50, 110);
}

void tuningDisplay() {
  for (i = 0; i < 4; i++) {
    TFTscreen.stroke(255, 255, 0);
    TFTscreen.setTextSize(1);
    TFTscreen.text("TUNING", 50, 45);
    delay(1000);
    TFTscreen.stroke(0, 20, 30);
    TFTscreen.text("TUNING", 50, 45);
    delay(500);
  }
}

void calibrateMotors() {                         // when K pressed - calibrates and display "calibrating" on the screen
  for (i = 0; i < 4; i++) {
    TFTscreen.stroke(255, 255, 0);
    TFTscreen.setTextSize(1);
    TFTscreen.text("CALIBRATING", 40, 45);
    delay(1000);
    TFTscreen.stroke(0, 20, 30);
    TFTscreen.text("CALIBRATING", 40, 45);
    delay(500);
  }
}

void calculateC() {                             // Calculate value C from entered frequency: C = 299792458 / (tuningFreq * 1000)
  Serial.println(tuningFreq);
  tuningFreqCalc = atof(tuningFreq);  // convert Freq input character array to a float
  KHz = 1000;
  PlanckTime = 299792458;
  Cmult = 1.5;
  valueCEffective = PlanckTime / ( tuningFreqCalc * KHz ) * Cmult;
  valueC = valueCEffective + valueCorr;
  Serial.print("Value C pF=");
  Serial.println(valueC);
}

void calculateL () {                            // Calculate "L" value from formula L=((1/(2*π*f*C))/(2*π*f))*1000000000000 (where f in KHz and C in pF)
  pi = 3.14159;
  Lmult = 1000000000000;
  valueL = ((1 / (2 * pi * tuningFreqCalc * (valueCEffective))) / (2 * pi * tuningFreqCalc)) * Lmult + valueLcorr;
  Serial.print("Value L uH=");
  Serial.println(valueL);
}

void calculateStepsC() {                        // calculate needed steps for C
  // Process for linear capacitor with 44-408 pF - need to ajust or change for different capacitor
  float totalStepsC = 2964;
  float pFmin = 20;
  float pFmax = 200;
  float pFtotalValue = pFmax - pFmin;
  Serial.println(pFtotalValue);
  float pFperStep = pFtotalValue / ( totalStepsC / 2 );
  Serial.println(pFperStep);
  valueRotateC1 = ( valueC - pFmin ) / pFperStep;
  Serial.print("steps C to rotate=");
  Serial.println(valueRotateC1);
}

void CmoveStage1() {                            // Send commandn to move motor C
//  send syntax '2,valueRotateC;' to Nano
}

void calculateStepsL() {                        // calculate needed steps for L
 // Process for non-linear variable inductor - need to ajust or change for different inductor  
 // y = -0.0299x2 + 1.3867x + 0.5894
 float totalStepsL = 2964;                                                              // total motor steps for 1 rotation
 float LturnsRequred = ( -0.0299 * valueL * valueL ) + ( 1.3867 * valueL ) + 0.5894;    // poly math
 valueRotateL = LturnsRequred * totalStepsL;
 Serial.print("L turns requred=");
 Serial.println(LturnsRequred);
 Serial.print("steps L to rotate=");
 Serial.println(valueRotateL);
 
}

void LmoveStage1() {                            // Send commandn to move motor L
//  send syntax '1,valueRotateL;' to Nano
}

//Keypad events
void keypadEvent(KeypadEvent eKey) {
  switch (keypad.getState()) {
    case PRESSED:
      //    lcd.print(eKey);
      switch (eKey) {
        case '#':                           // submit frequesncy
          SetFrequency();
          break;
        case 'C':                           // will be used for "C" motor move 1 step CCW
          incCCorr(-1);
          break;
        case 'c':                           // will be used for "C" motor move 1 step CW
          incCCorr(1);
          break;
        case 'L':                           // used for "L" motor move 1 step CCW
          incLCorr(-0.001);
          break;
        case 'l':                           // used for "L" motor move 1 step CW
          incLCorr(0.001);
          break;
        case '*':                           // erase entered frequency
          eraseFrequency();
          break;
        case 'U':                           // calibrate both motors
          mechCalibrate(channelC1);
          break;
        case 'D':                           // calibrate both motors
          mechCalibrate(channelL);
          break;
          
      }
  }
}



void loop() {
  busLoop();
  key = keypad.getKey();

  if (key != NO_KEY && (key == '1' || key == '2' || key == '3' || key == '4' || key == '5' || key == '6' || key == '7' || key == '8' || key == '9' || key == '0'))
  {
    //                                               entering required frequency at screen bottom
    if (presentValue != true) {
      EnteredFreqString = EnteredFreqString + key;
      int numLength = EnteredFreqString.length();
      EnteredFreqString.toCharArray(enteredFreq, 6);
      TFTscreen.stroke(0, 255, 0);
      TFTscreen.setTextSize(2);
      TFTscreen.text(enteredFreq, 50, 110);
    }
  }

  //                                                  print serial to TFT - reserved for future use
  //  if (Serial.available()) {
  //    char key = Serial.read();
  //    String serialData = String(key);
  //    serialData.toCharArray(dataPrintout, 10);
  //    TFTscreen.stroke(255, 255, 0);
  //    TFTscreen.setTextSize(2);
  //    TFTscreen.text(dataPrintout, 50, 70);
  //    // wait for 1s
  //    delay(1000);
  //    // erase the text
  //    TFTscreen.stroke(0, 20, 30);
  //    TFTscreen.text(dataPrintout, 50, 70);
  //  }

}
