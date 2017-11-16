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

char key;
char DisplayValueC1[6];
char DisplayValueC1pF[6];
char DisplayValueC2[6];
char DisplayValueC2pF[6];
char DisplayValueL[6];
char DisplayValueL1uH[6];
char DisplayValueSWR[6];
char input; //buffer for input characters for calculations

float valueRotateC1 = -1;
float valueRotateC2 = -1;
float valueRotateL = -1;
float valueSWR;
float stepsToMhFloat;

boolean presentValue = false;

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
  TFTscreen.text("C1", 5, 55);                     // write the text to coordinates
  // Displays C2
  TFTscreen.stroke(0, 255, 0);                      // set the font color
  TFTscreen.setTextSize(2);                         // set the font size 2
  TFTscreen.text("C2", 5, 85);                     // write the text to coordinates

  keypad.addEventListener(keypadEvent);             //add an event listener for this keypad
}


void incL(float diff) {                             // rotate motor L with number of steps
  mechSetPosition(round (diff), 0, 0);
}

void incC1(float diff) {                            // rotate motor C1 Cold with number of steps
  mechSetPosition(0, round(diff), 0);
}

void incC2(float diff) {                            // rotate motor C2 Hot with number of steps
  mechSetPosition(0, 0, round(diff));
}

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


double stepsToUH(float x) {
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

void displayLSteps() {                                         // display L steps values
  dtostrf(valueRotateL, 6, 0, DisplayValueL);
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
  dtostrf(stepsToUH(valueRotateL), 4, 2, DisplayValueL1uH);
  if (isBusy) {
    TFTscreen.stroke(255, 255, 0);
  } else {
    TFTscreen.stroke(0, 255, 0);
  }
  TFTscreen.setTextSize(2);
  TFTscreen.text(DisplayValueL1uH, 40, 25);
}

void ErasDisplayValueLuH() {                                   // erase display L values in uH
  TFTscreen.stroke(0, 20, 30);
  TFTscreen.setTextSize(2);
  TFTscreen.text(DisplayValueL1uH, 40, 25);
}

void displayL() {                                             // display L steps and uH
  displayLSteps();
  displayValueLuH();
}

void EraseDisplayL() {                                        // erase L steps and uH
  EraseDisplayLSteps();
  ErasDisplayValueLuH();
}

double stepsC1topF(double x) {
  return  3.5215908593183684e+002 * pow(x, 0)
          + -1.6245967367326516e-001 * pow(x, 1)
          + -8.9757122524750620e-004 * pow(x, 2)
          +  5.3372752892602197e-006 * pow(x, 3)
          + -1.7696527557016530e-008 * pow(x, 4)
          +  3.3363571963309934e-011 * pow(x, 5)
          + -3.6438105893235793e-014 * pow(x, 6)
          +  2.2733846666230038e-017 * pow(x, 7)
          + -7.5016703085658784e-021 * pow(x, 8)
          +  1.0154127876671653e-024 * pow(x, 9);
}

void displayC1steps() {                                       // display C1 steps values
  dtostrf(valueRotateC1, 5, 0, DisplayValueC1);
  if (isBusy) {
    TFTscreen.stroke(255, 255, 0);
  } else {
    TFTscreen.stroke(0, 255, 0);
  }
  TFTscreen.setTextSize(1);
  TFTscreen.text(DisplayValueC1, 55, 72);
}

void EraseDisplayC1steps() {                                   // erase display C1 steps values
  TFTscreen.stroke(0, 20, 20);
  TFTscreen.setTextSize(1);
  TFTscreen.text(DisplayValueC1, 55, 72);
}

void displayValueC1pF() {                                         // display C1 values in pF
  dtostrf(stepsC1topF(valueRotateC1), 4, 1, DisplayValueC1pF);
  if (isBusy) {
    TFTscreen.stroke(255, 255, 0);
  } else {
    TFTscreen.stroke(0, 255, 0);
  }
  TFTscreen.setTextSize(2);
  TFTscreen.text(DisplayValueC1pF, 40, 55);
}

void ErasDisplayValueC1pF() {                                   // erase display C1 values in pF
  TFTscreen.stroke(0, 20, 30);
  TFTscreen.setTextSize(2);
  TFTscreen.text(DisplayValueC1pF, 40, 55);
}

void displayC1() {                                             // display C1 values
  displayC1steps();
  displayValueC1pF();
}

void EraseDisplayC1() {                                        // erase C1 values
  EraseDisplayC1steps();
  ErasDisplayValueC1pF();
}

double stepsC2topF(double x) {
  return  3.5215908593183684e+002 * pow(x, 0)
          + -1.6245967367326516e-001 * pow(x, 1)
          + -8.9757122524750620e-004 * pow(x, 2)
          +  5.3372752892602197e-006 * pow(x, 3)
          + -1.7696527557016530e-008 * pow(x, 4)
          +  3.3363571963309934e-011 * pow(x, 5)
          + -3.6438105893235793e-014 * pow(x, 6)
          +  2.2733846666230038e-017 * pow(x, 7)
          + -7.5016703085658784e-021 * pow(x, 8)
          +  1.0154127876671653e-024 * pow(x, 9);
}

void displayC2steps() {                                       // display C2 steps values
  dtostrf(valueRotateC2, 5, 0, DisplayValueC2);
  if (isBusy) {
    TFTscreen.stroke(255, 255, 0);
  } else {
    TFTscreen.stroke(0, 255, 0);
  }
  TFTscreen.setTextSize(1);
  TFTscreen.text(DisplayValueC2, 55, 102);
}

void EraseDisplayC2steps() {                                   // erase display C2 steps values
  TFTscreen.stroke(0, 20, 20);
  TFTscreen.setTextSize(1);
  TFTscreen.text(DisplayValueC2, 55, 102);
}

void displayValueC2pF() {                                         // display C2 values in pF
  dtostrf(stepsC2topF(valueRotateC2), 4, 1, DisplayValueC2pF);
  if (isBusy) {
    TFTscreen.stroke(255, 255, 0);
  } else {
    TFTscreen.stroke(0, 255, 0);
  }
  TFTscreen.setTextSize(2);
  TFTscreen.text(DisplayValueC2pF, 40, 85);
}

void ErasDisplayValueC2pF() {                                   // erase display C1 values in pF
  TFTscreen.stroke(0, 20, 30);
  TFTscreen.setTextSize(2);
  TFTscreen.text(DisplayValueC2pF, 40, 85);
}

void displayC2() {                                             // display C1 values
  displayC2steps();
  displayValueC2pF();
}

void EraseDisplayC2() {                                        // erase C1 values
  EraseDisplayC2steps();
  ErasDisplayValueC2pF();
}

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
        ///        case '*':                           // erase entered frequency
        //          eraseFrequency();
        //          break;
        case 'S':                           // erase entered frequency
          mechAutoTune();
          break;
        case 'K':                           // erase entered frequency
          mechFineTune();
          break;
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
