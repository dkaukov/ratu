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


  Motor 1 = "L" Motor
  Motor 2 = "C1" Motor "Cold"
  Motor 3 = "C2 Motor "Hot"

*/
#include "proto/device_id.h"
#include <TFT.h>    // Arduino LCD library
#include <SPI.h>    // SPI library
#include <Keypad.h> // Keypad library
#include <math.h>   // Math library
#define PJON_ID ID_HAL100
#include "proto/common.h"

//Bell icon size: 16W*16H
const unsigned char bell [] PROGMEM = {
  0x01, 0x80, 0x03, 0xc0, 0x0f, 0xf0, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x3f, 0xfc, 0x3f, 0xfc,
  0x3f, 0xfc, 0x3f, 0xfc, 0x3f, 0xfc, 0x7f, 0xfe, 0x7f, 0xfe, 0xc3, 0xc3, 0xc3, 0xc3, 0x3f, 0xfc,
};

// TFT display CS, RS and RST pins definition for Mega2560
#define cs   10
#define dc   9
#define rst  8

// create an instance of the TFT library
TFT TFTscreen = TFT(cs, dc, rst);

boolean displayRefresh = false;
boolean isBusy = false;

// char array

char enteredFreq[6];
char tuningFreq[6];
char key;
char DisplayValueC1[6];
char DisplayValueC2[6];
char DisplayValueL[6];
char DisplayValueLUH[4];
char DisplayValueSWR[6];
char input; //buffer for input characters for calculations
// char dataPrintout[10];

// float tuningFreqCalc = 0;   // float for value C calculation
// float KHz;                  // KHz constant
// float PlanckTime;           // Planck constant
// float pi;                   // π constant
// float valueC1 = 0;           // calculated value C float
// float valueCEffective = 0;  // calculated value C float
// float valueL = 0;           // calculated value L float
// float valueC1Corr = 0;      // calculated value C1 float
// float valueC2Corr = 0;      // calculated value C1 float
// float valueLcorr = 0;       // calculated value L float
// float Cmult;                // constant multiplier to calculate C value
// float Lmult;                // constant multiplier to calculate L value
float valueRotateC1 = -1;
float valueRotateC2 = -1;
float valueRotateL = -1;
float valueSWR;
float stepsToMhFloat;

boolean presentValue = false;

String EnteredFreqString, TuningFreqString;
String DisplayValueLstring;

int i;


// --- Keypad code start ---
const byte ROWS = 5; // Five rows
const byte COLS = 4; // Four columns
// Define the Keymap
char keys[ROWS][COLS] = {
  // Buttons association 1=1 2=2 3=3 4=4 5=5 6=6 7=7 8=8 9=9 0=0 *=Led OFF #=Led ON
  // TOP Buttons  C c L l
  //Buttons in middle H U D S

  // COL0-pin1, COL1-pin2, COL2-pin3, COL3-pin4
  {'S', '1', '2', '3'}, // ROW0-pin5
  {'K', '4', '5', '6'}, // ROW1-pin6
  {'U', '7', '8', '9'}, // ROW2-pin7
  {'D', '*', '0', '#'}, // ROW3-pin8
  {'c', 'C', 'h', 'H'}  // ROW4-pin9

};

// Connect keypad ROW0 Pad pin ->> 5=30 <<- pin Mega, ROW1 6=32, ROW2 7=23, ROW3 8=25, ROW4 9=27
byte rowPins[ROWS] = { 30, 32, 23, 25, 27 };
// Connect keypad COL0 Pad pin ->> 1=22 <<- pin Mega, COL1 2=24, COL2 3=26, COL3 4=28
byte colPins[COLS] = { 22, 24, 26, 28 };

// Create the Keypad
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );
// --- end of Keypad code ---

void busReceiver(char cmd, uint8_t length, const TCommand *payload) {
  switch (cmd) {
    case cmdStatus: {
        Serial.print("cnt=");
        Serial.print(payload->status.cnt);
        Serial.print(", Value fwd=");
        Serial.print(payload->status.adc.fwd);
        Serial.print(", Value rfl=");
        Serial.println(payload->status.adc.rfl);

        float rfl = payload->status.adc.rfl;
        float fwd = payload->status.adc.fwd;
        float p = (rfl / fwd);
        valueSWR = (1 + p) / (1 - p);

        valueRotateL = payload->status.pos.lPos;
        valueRotateC1 = payload->status.pos.c1Pos;
        valueRotateC2 = payload->status.pos.c2Pos;
        isBusy = payload->status.flags > 0;

        displayRefresh = true;
        break;
      }
    case cmdDebug: {
        Serial.write((char *)payload, length);
        break;
      }
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(TS_PORT_BITRATE);
  pinMode(40, OUTPUT);
  TFTscreen.begin();                                // initialise TFT screen
  TFTscreen.background(0, 20, 30);                  // clear the screen with a RGB background

  busInit(busReceiver, 2, &Serial2);
  displayInitialScreen();

}

void displayInitialScreen() {                      // displays "Wait Calibrating at start. To be linked to Motors "L" and "C" calibration process
  // Displays SWR
  TFTscreen.stroke(0, 255, 0);                      // set the font color
  TFTscreen.setTextSize(2);                         // set the font size 2
  TFTscreen.text("SWR", 5, 5);                     // write the text to coordinates
  // Displays L
  TFTscreen.stroke(0, 255, 0);                      // set the font color
  TFTscreen.setTextSize(2);                         // set the font size 2
  TFTscreen.text("L", 5, 25);                     // write the text to coordinates
  // Displays C1
  TFTscreen.stroke(0, 255, 0);                      // set the font color
  TFTscreen.setTextSize(2);                         // set the font size 2
  TFTscreen.text("C1", 5, 65);                     // write the text to coordinates
  // Displays C2
  TFTscreen.stroke(0, 255, 0);                      // set the font color
  TFTscreen.setTextSize(2);                         // set the font size 2
  TFTscreen.text("C2", 5, 85);                     // write the text to coordinates

  keypad.addEventListener(keypadEvent);             //add an event listener for this keypad
}

//void SetFrequencyStage1() {
// STAGE 1 tuning process
//	calculateC();                                     // calculate C value at selected frequency
//	calculateStepsC();                                // calculate steps needed to move C to value C
//	CmoveStage1();                                    // send command to nano to move motor "C" to C step value
//	calculateL();                                     // calculate L value at given frequency and C value
//	calculateStepsL();                                // calculate steps needed to move C to value L
//	LmoveStage1();                                    // send command to nano to move motor "L" to L step value
//	EraseDisplayC1(), EraseDisplayL();                 // clear previous values L and C at display
//	displayC1(), displayL();                           // display current values L and C at display
//	mechSetPosition(round (valueRotateL), round (valueRotateC1), round (valueRotateC2));
// End STAGE 1 tuning process
//}

// Pre-STAGE 1 (coarse tuning process) cleanups and displays - will be used later
//void SetFrequency() {                               // when # pressed - sets frequency value at the bottom of the screen
//  eraseFreqTopScreen();                             // clear previously entered drequesncy at display top
//  eraseEnteredFrequency();                          // clear entered frequency at display bottom
//  displayFreqTopScreen();                           // display desired frequesncy at display top
  //  SetFrequencyStage1();
//}


void incL(float diff) {                             // rotate motor L with number of steps
  mechSetPosition(round (diff), 0, 0);
}

void incC1(float diff) {                            // rotate motor C1 Cold with number of steps
  mechSetPosition(0, round(diff), 0);
}

void incC2(float diff) {                            // rotate motor C2 Hot with number of steps
  mechSetPosition(0, 0, round(diff));
}

// void incLCorr(float diff) {
//    if (tuningFreqCalc != 0.0) {
//      valueLcorr = valueLcorr + diff;
//      SetFrequencyStage1();
//    }
//}

//void incC1Corr(float diff) {
//    if (tuningFreqCalc != 0.0) {
//      valueC1Corr = valueC1Corr + diff;
//      SetFrequencyStage1();
//    }
//}

//void incC2Corr(float diff) {
//    if (tuningFreqCalc != 0.0) {
//      valueC2Corr = valueC2Corr + diff;
//      SetFrequencyStage1();
//    }
//}

//void eraseFreqTopScreen() {                      // erases frequency value from top of display
//  TFTscreen.stroke(0, 20, 30);
//  TFTscreen.setTextSize(2);
//  TFTscreen.text(tuningFreq, 50, 110);
//  TuningFreqString = "";
//}

//void displayFreqTopScreen() {                   // shows frequency value at top of display font 4
//  TuningFreqString = EnteredFreqString;
//  TuningFreqString.toCharArray(tuningFreq, 6);
//  EnteredFreqString = "";
//  TFTscreen.stroke(255, 255, 0);
//  TFTscreen.setTextSize(2);
//  TFTscreen.text(tuningFreq, 50, 110);
//}

//void displayTunedFreqTopScreen() {             // Display tuningFreq at the top of the screen in "green tuned mode"
//  TFTscreen.stroke(0, 255, 0);
//  TFTscreen.setTextSize(2);
//  TFTscreen.text(tuningFreq, 50, 110);
//}

//void eraseEnteredFrequency () {               // erase frequency value at the botton of the screen
//  TFTscreen.setTextSize(2);
//  TFTscreen.stroke(0, 20, 30);
//  TFTscreen.text(enteredFreq, 50, 110);
//}

void displaySWRvalue() {                                    // process to display SWR values
  EraseDisplaySWR();
  displaySWR();
}

void displaySWR() {                                         // display SWR values
  dtostrf(valueSWR, 5, 2, DisplayValueSWR);
  TFTscreen.stroke(0, 255, 0);
  TFTscreen.setTextSize(2);
  TFTscreen.text(DisplayValueSWR, 40, 5);
}

void EraseDisplaySWR() {                                   // erase display SWR values
  TFTscreen.stroke(0, 20, 30);
  TFTscreen.setTextSize(2);
  TFTscreen.text(DisplayValueSWR, 40, 5);
}

void displayLSteps() {                                         // display L steps values
  dtostrf(valueRotateL, 5, 0, DisplayValueL);
  if (isBusy) {
    TFTscreen.stroke(255, 255, 0);
  } else {
    TFTscreen.stroke(0, 255, 0);
  }
  TFTscreen.setTextSize(1);
  TFTscreen.text(DisplayValueL, 55, 42);
}

void EraseDisplayLSteps() {                                   // erase display L steps values
  TFTscreen.stroke(0, 20, 30);
  TFTscreen.setTextSize(1);
  TFTscreen.text(DisplayValueL, 55, 42);
}

void displayValueLuH() {                                         // display L values in uH
  dtostrf(stepsToMh(valueRotateL), 4, 2, DisplayValueLUH);
  if (isBusy) {
    TFTscreen.stroke(255, 255, 0);
  } else {  
    TFTscreen.stroke(0, 255, 0);
  }
  TFTscreen.setTextSize(2);
  TFTscreen.text(DisplayValueLUH, 40, 25);
}

void ErasDisplayValueLuH() {                                   // erase display L values in uH
  TFTscreen.stroke(0, 20, 30);
  TFTscreen.setTextSize(2);
  TFTscreen.text(DisplayValueLUH, 40, 25);
}

void displayL() {                                             // display L steps and uH 
  displayLSteps();  
  displayValueLuH();                                    
}

void EraseDisplayL() {                                        // erase L steps and uH
  EraseDisplayLSteps();  
  ErasDisplayValueLuH();                                    
}


void displayC1() {                                       // display C values
  dtostrf(valueRotateC1, 5, 0, DisplayValueC1);
  if (isBusy) {
    TFTscreen.stroke(255, 255, 0);
  } else {
    TFTscreen.stroke(0, 255, 0);
  }
  TFTscreen.setTextSize(2);
  TFTscreen.text(DisplayValueC1, 40, 65);
}

void EraseDisplayC1() {                                   // erase display C values
  TFTscreen.stroke(0, 20, 20);
  TFTscreen.setTextSize(2);
  TFTscreen.text(DisplayValueC1, 40, 65);
}

void displayC2() {                                       // display C values
  dtostrf(valueRotateC2, 5, 0, DisplayValueC2);
  if (isBusy) {
    TFTscreen.stroke(255, 255, 0);
  } else {
    TFTscreen.stroke(0, 255, 0);
  }
  TFTscreen.setTextSize(2);
  TFTscreen.text(DisplayValueC2, 40, 85);
}

void EraseDisplayC2() {                                   // erase display C values
  TFTscreen.stroke(0, 20, 20);
  TFTscreen.setTextSize(2);
  TFTscreen.text(DisplayValueC2, 40, 85);
}

double stepsToMh(float x) {
    return  3.2408606623203362e-001 * pow(x, 0)
            +  4.8175734249036963e-005 * pow(x, 1)
            + -8.2394558534492893e-009 * pow(x, 2)
            +  4.1381892772191581e-012 * pow(x, 3)
            + -5.0628320718901775e-016 * pow(x, 4)
            +  3.2564586259750260e-020 * pow(x, 5)
            + -1.1101886921087241e-024 * pow(x, 6)
            +  1.3237902692887521e-029 * pow(x, 7)
            +  3.9488291689952899e-034 * pow(x, 8)
            + -1.8793946779017113e-038 * pow(x, 9)
            +  3.2823426065013548e-043 * pow(x, 10)
            + -2.7820892583909193e-048 * pow(x, 11)
            +  9.4929814793314678e-054 * pow(x, 12);
  }
  


void eraseFrequency() {                         // when * pressed - erases entered frequency bottom of the screen
  EnteredFreqString = "";
  TuningFreqString = "";
  TFTscreen.stroke(0, 20, 30);
  TFTscreen.setTextSize(2);
  TFTscreen.text(enteredFreq, 50, 110);
}

//void tuningDisplay() {
//  for (i = 0; i < 4; i++) {
//    TFTscreen.stroke(255, 255, 0);
//    TFTscreen.setTextSize(1);
//    TFTscreen.text("TUNING", 50, 45);
//    delay(1000);
//    TFTscreen.stroke(0, 20, 30);
//    TFTscreen.text("TUNING", 50, 45);
//    delay(500);
//  }
//}

//void calculateC() {                             // Calculate value C from entered frequency: C = 299792458 / (tuningFreq * 1000)
//  Serial.println(tuningFreq);
//  tuningFreqCalc = atof(tuningFreq);  // convert Freq input character array to a float
//  KHz = 1000;
//  PlanckTime = 299792458;
//  Cmult = 1.5;
//  valueCEffective = PlanckTime / ( tuningFreqCalc * KHz ) * Cmult;
//  valueC1 = valueCEffective + valueC1Corr;
//  Serial.print("Value C1 pF=");
//  Serial.println(valueC1);
//}

//void calculateL () {                            // Calculate "L" value from formula L=((1/(2*π*f*C))/(2*π*f))*1000000000000 (where f in KHz and C in pF)
//  pi = 3.14159;
//  Lmult = 1000000000000;
//  valueL = ((1 / (2 * pi * tuningFreqCalc * (valueCEffective))) / (2 * pi * tuningFreqCalc)) * Lmult + valueLcorr;
//  Serial.print("Value L uH=");
//  Serial.println(valueL);
//}

//void calculateStepsC() {                        // calculate needed steps for C
// Process for linear capacitor with 44-408 pF - need to ajust or change for different capacitor
//  float totalStepsC = 2964;
//  float pFmin = 20;
//  float pFmax = 200;
// float pFtotalValue = pFmax - pFmin;
//  Serial.println(pFtotalValue);
//  float pFperStep = pFtotalValue / ( totalStepsC / 2 );
//  Serial.println(pFperStep);
//  valueRotateC1 = ( valueC1 - pFmin ) / pFperStep;
//  Serial.print("steps C to rotate=");
//  Serial.println(valueRotateC1);
//}

//void CmoveStage1() {                            // Send commandn to move motor C
//  send syntax '2,valueRotateC;' to Nano
//}

//void calculateStepsL() {                        // calculate needed steps for L
// Process for non-linear variable inductor - need to ajust or change for different inductor
// y = -0.0299x2 + 1.3867x + 0.5894
// float totalStepsL = 2964;                                                              // total motor steps for 1 rotation
// float LturnsRequred = ( -0.0299 * valueL * valueL ) + ( 1.3867 * valueL ) + 0.5894;    // poly math
// valueRotateL = LturnsRequred * totalStepsL;
// Serial.print("L turns requred=");
// Serial.println(LturnsRequred);
// Serial.print("steps L to rotate=");
// Serial.println(valueRotateL);
//}

//void LmoveStage1() {                            // Send commandn to move motor L
//  send syntax '1,valueRotateL;' to Nano
//}


void sendStatusRequest() {
  static const unsigned long REFRESH_INTERVAL = 100; // ms
  static unsigned long lastRefreshTime = 0;
  if (millis() - lastRefreshTime >= REFRESH_INTERVAL)
  {
    lastRefreshTime = millis();
    mechSendStatusRequest();
  }
}


//Keypad events
void keypadEvent(KeypadEvent eKey) {
  switch (keypad.getState()) {
    case PRESSED:
      //    lcd.print(eKey);
      switch (eKey) {
//        case '#':                           // submit frequesncy
//          SetFrequency();
//         break;
        case 'c':                           // "C1 Cold" motor move 5 step CCW
          incC1(-1);
          break;
        case 'C':                           // "C1 Cold" motor move 5 step CW
          incC1(1);
          break;
        case 'h':                           // "C2 Hot" motor move 5 step CCW
          incC2(-1);
          break;
        case 'H':                           // "C2 Hot" motor move 5 step CW
          incC2(1);
          break;
        case 'D':                           // "L" motor move 100 step CCW
          incL(-10.0);
          break;
        case 'U':                           // "L" motor move 100 step CW
          incL(+10.0);
          break;
        case '*':                           // erase entered frequency
          eraseFrequency();
          break;
        case 'S':                           // erase entered frequency
          mechAutoTune();
          break;
        case 'K':                           // erase entered frequency
          mechFineTune();
          break;

          //        case 'U':                           // calibrate both motors
          //          mechCalibrate(channelC1);
          //          break;
          //        case 'D':                           // calibrate both motors
          //          mechCalibrate(channelL);
          //          break;

      }
  }
}



void loop() {
  busLoop();
  //sendStatusRequest();

  key = keypad.getKey();

  if (key != NO_KEY && (key == '1' || key == '2' || key == '3' || key == '4' || key == '5' || key == '6' || key == '7' || key == '8' || key == '9' || key == '0'))
  {
    //                                               entering required frequency at screen bottom
    if (presentValue != true) {
      /*
        EnteredFreqString = EnteredFreqString + key;
        int numLength = EnteredFreqString.length();
        EnteredFreqString.toCharArray(enteredFreq, 6);
        TFTscreen.stroke(0, 255, 0);
        TFTscreen.setTextSize(2);
        TFTscreen.text(enteredFreq, 50, 110);
      */
    }
  }
  if (displayRefresh) {

    displaySWRvalue();

    EraseDisplayL();
    displayL();

    EraseDisplayC1();
    displayC1();

    EraseDisplayC2();
    displayC2();

    if (!isBusy) {
      TFTscreen.stroke(255, 255, 0);                  // set the font color
      digitalWrite(40, LOW);
    } else {
      TFTscreen.stroke(0, 20, 20);
      digitalWrite(40, HIGH);
    }
    TFTscreen.setTextSize(1);
    TFTscreen.text("Ready", 50, 115);                // write the text to coordinates

    displayRefresh = false;
  }


}
