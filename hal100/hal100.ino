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
#include <SPI.h>          // SPI library
#include <TFT_ST7735.h>   // Graphics and font library for ST7735 driver chip
#include <Keypad.h>       // Keypad library
#include <math.h>         // Math library
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
TFT_ST7735 tft = TFT_ST7735();

#define DISPLAY_REFRESH_L   0x01
#define DISPLAY_REFRESH_C1  0x02
#define DISPLAY_REFRESH_C2  0x04
#define DISPLAY_REFRESH_SWR 0x08
#define DISPLAY_REFRESH_BSY 0x10

uint8_t displayRefreshFlags = 0;
boolean isBusy = false;

// char array

char key;
char DisplayValueC1[10];
char DisplayValueC1pF[10];
char DisplayValueC2[10];
char DisplayValueC2pF[10];
char DisplayValueL[10];
char DisplayValueL1uH[10];
char DisplayValueSWR[10];
char input; //buffer for input characters for calculations

float valueRotateC1 = -1;
float valueRotateC2 = -1;
float valueRotateL = -1;
float valueSWR;
float stepsToMhFloat;

boolean presentValue = false;

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
        float rfl = payload->status.adc.rfl;
        float fwd = payload->status.adc.fwd;
        float p = (rfl / fwd);
        float swr = (1 + p) / (1 - p);
        if (swr != valueSWR) {
          valueSWR = (1 + p) / (1 - p);
          displayRefreshFlags |= DISPLAY_REFRESH_SWR;
        }
        if (valueRotateL != payload->status.pos.lPos) {
          valueRotateL = payload->status.pos.lPos;
          displayRefreshFlags |= DISPLAY_REFRESH_L;
        }
        if (valueRotateC1 != payload->status.pos.c1Pos) {
          valueRotateC1 = payload->status.pos.c1Pos;
          displayRefreshFlags |= DISPLAY_REFRESH_C1;
        }
        if (valueRotateC2 != payload->status.pos.c2Pos) {
          valueRotateC2 = payload->status.pos.c2Pos;
          displayRefreshFlags |= DISPLAY_REFRESH_C2;
        }
        boolean bsy = payload->status.flags & (1 << 3);
        if (isBusy != bsy) {
          isBusy = bsy;
          displayRefreshFlags |= DISPLAY_REFRESH_BSY;
        }
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
  tft.init();
  tft.setRotation(1);                               // initialise TFT screen
  busInit(busReceiver, 2, &Serial2);
  tft.fillScreen(TFT_BLACK);
  displayInitialScreen();

}

void displayInitialScreen() {                      // displays "Wait Calibrating at start. To be linked to Motors "L" and "C" calibration process
  // Displays SWR
  tft.setTextColor(TFT_GREEN);
  tft.setTextSize(1);                         // set the font size 2
  tft.drawString("SWR", 10, 10, 2);             // write the text to coordinates
  // Displays L
  tft.setTextColor(TFT_GREEN);
  tft.setTextSize(1);
  tft.drawString("L", 7, 37, 2);
  // Displays uH
  tft.setTextColor(TFT_GREEN);
  tft.setTextSize(1);
  tft.drawString("uH", 117, 49, 2);
  // Displays C1
  tft.setTextColor(TFT_GREEN);
  tft.setTextSize(1);
  tft.drawString("C1", 7, 70, 2);
  // Displays pF for C1
  tft.setTextColor(TFT_GREEN);
  tft.setTextSize(1);
  tft.drawString("pF", 117, 81, 2);
  // Displays C2
  tft.setTextColor(TFT_GREEN);
  tft.setTextSize(1);
  tft.drawString("C2", 7, 101, 2);
  // Displays pF for C2
  tft.setTextColor(TFT_GREEN);
  tft.setTextSize(1);
  tft.drawString("pF", 117, 112, 2);
  

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
  tft.setTextColor(TFT_GREEN);
  tft.setTextSize(1);
  tft.drawString(DisplayValueSWR, 50, 5, 4);
}

void EraseDisplaySWR() {                                   // erase display SWR values
  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(2);
  tft.drawString(DisplayValueSWR, 40, 5, 1);
}

double stepsToUH(double x) {
   return  3.2984940709870936e-001 * pow(x,0)
        +  2.5621377247607574e-005 * pow(x,1)
        +  7.3269703268229895e-009 * pow(x,2)
        +  1.7411736893707688e-013 * pow(x,3)
        + -1.9811135781400833e-017 * pow(x,4)
        +  6.3890319515881330e-022 * pow(x,5)
        + -9.2196784942118622e-027 * pow(x,6)
        +  4.9627745914218940e-032 * pow(x,7)
        +  7.0123651511795207e-039 * pow(x,8);
}

void displayValueLuH() {                                         // display L values in uH
  dtostrf(stepsToUH(valueRotateL), 4, 3, DisplayValueL1uH);
  if (isBusy) {
    tft.setTextColor(TFT_YELLOW);
  } else {
    tft.setTextColor(TFT_GREEN);
  }
  tft.setTextSize(1);
  tft.drawString(DisplayValueL1uH, 50, 42, 4);
}

void ErasDisplayValueLuH() {                                   // erase display L values in uH
  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(1);
  tft.drawString(DisplayValueL1uH, 50, 42, 4);
}

void displayLSteps() {                                         // display L steps values
  dtostrf(valueRotateL, 6, 0, DisplayValueL);
  if (isBusy) {
    tft.setTextColor(TFT_YELLOW);

  } else {
    tft.setTextColor(TFT_GREEN);
  }
  tft.setTextSize(1);
  tft.drawString(DisplayValueL, 0, 51, 2);
}

void EraseDisplayLSteps() {                                   // erase display L steps values
  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(1);
  tft.drawString(DisplayValueL, 0, 51, 2);
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

void displayValueC1pF() {                                         // display C1 values in pF
  dtostrf(stepsC1topF(valueRotateC1), 4, 1, DisplayValueC1pF);
  if (isBusy) {
    tft.setTextColor(TFT_YELLOW);
  } else {
    tft.setTextColor(TFT_GREEN);
  }
  tft.setTextSize(1);
  tft.drawString(DisplayValueC1pF, 50, 75, 4);
}

void ErasDisplayValueC1pF() {                                   // erase display C1 values in pF
  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(1);
  tft.drawString(DisplayValueC1pF, 50, 75, 4);
}

void displayC1steps() {                                       // display C1 steps values
  dtostrf(valueRotateC1, 5, 0, DisplayValueC1);
  if (isBusy) {
    tft.setTextColor(TFT_YELLOW);
  } else {
    tft.setTextColor(TFT_GREEN);
  }
  tft.setTextSize(1);
  tft.drawString(DisplayValueC1, 0, 83, 2);
}

void EraseDisplayC1steps() {                                   // erase display C1 steps values
  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(1);
  tft.drawString(DisplayValueC1, 0, 83, 2);
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

void displayValueC2pF() {                                         // display C2 values in pF
  dtostrf(stepsC2topF(valueRotateC2), 4, 1, DisplayValueC2pF);
  if (isBusy) {
    tft.setTextColor(TFT_YELLOW);
  } else {
    tft.setTextColor(TFT_GREEN);
  }
  tft.setTextSize(1);
  tft.drawString(DisplayValueC2pF, 50, 107, 4);
}

void ErasDisplayValueC2pF() {                                   // erase display C1 values in pF
  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(1);
  tft.drawString(DisplayValueC2pF, 50, 107, 4);
}

void displayC2steps() {                                       // display C2 steps values
  dtostrf(valueRotateC2, 5, 0, DisplayValueC2);
  if (isBusy) {
    tft.setTextColor(TFT_YELLOW);
  } else {
    tft.setTextColor(TFT_GREEN);
  }
  tft.setTextSize(1);
  tft.drawString(DisplayValueC2, 0, 115, 2);
}

void EraseDisplayC2steps() {                                   // erase display C2 steps values
  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(1);
  tft.drawString(DisplayValueC2, 0, 115, 2);
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
        TFTscreen.setTextColor(TFT_GREEN);
        TFTscreen.setTextSize(2);
        TFTscreen.drawString(enteredFreq, 50, 110);
      */
    }
  }
  if (displayRefreshFlags != 0) {

    if ((displayRefreshFlags & DISPLAY_REFRESH_SWR) != 0) {
      displaySWRvalue();
      displayRefreshFlags &= ~DISPLAY_REFRESH_SWR;
    }
    if ((displayRefreshFlags & DISPLAY_REFRESH_L) != 0) {
      EraseDisplayL();
      displayL();
      displayRefreshFlags &= ~DISPLAY_REFRESH_L;
    }
    if ((displayRefreshFlags & DISPLAY_REFRESH_C1) != 0) {
      EraseDisplayC1();
      displayC1();
      displayRefreshFlags &= ~DISPLAY_REFRESH_C1;
    }
    if ((displayRefreshFlags & DISPLAY_REFRESH_C2) != 0) {
      EraseDisplayC2();
      displayC2();
      displayRefreshFlags &= ~DISPLAY_REFRESH_C2;
    }
    if ((displayRefreshFlags & DISPLAY_REFRESH_BSY) != 0) {
      if (!isBusy) {
        tft.setTextColor(TFT_YELLOW);
        digitalWrite(40, LOW);
      } else {
        tft.setTextColor(TFT_BLACK);
        digitalWrite(40, HIGH);
      }
//      tft.setTextSize(1);
//      tft.drawString("Ready", 50, 115, 1);                // write the text to coordinates
//      displayRefreshFlags &= ~DISPLAY_REFRESH_BSY;
    }
  }
}
