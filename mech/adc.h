/*
 * adc.h
 *
 *  Created on: 24Sep.,2017
 *      Author: ziss
 */

#ifndef ADC_H_
#define ADC_H_

const byte fwdPwrPin    = A5;     // Fwd signal from SWR sensor
const byte rflPwrPin    = A6;     // Rev. signal from SWR sensor

uint16_t fwdPwrVal = 0;
uint16_t rflPwrVal = 0;
const uint16_t diodeDropVal = 161*2;
uint16_t adcCnt = 0;

inline void adcStartconversion(uint8_t pin) {
  ADMUX  =  bit(REFS0) | ((pin - A0) & 0x07);
  sbi(ADCSRA, ADSC);
}

inline uint8_t adcMuxPin() {
  return (ADMUX & 0x07) + A0;
}

inline void adcInit() {
  pinMode(fwdPwrPin, INPUT);
  pinMode(rflPwrPin, INPUT);
  ADCSRA = bit(ADEN);
  ADCSRA |= bit(ADPS0) |  bit(ADPS1) | bit(ADPS2);
  adcStartconversion(fwdPwrPin);
}

inline uint8_t isAdcConversionFinished(){
  return !bit_is_set(ADCSRA, ADSC);
}

int16_t adcGetForwardVoltage(){
  static int16_t lpf;
  uint8_t low, high;
  low  = ADCL;  high = ADCH;
  int16_t val = (((high << 8) | low) << 3);
  lpf += (val -  lpf) >> 2;
  return lpf;
}

int16_t adcGetReflectedVoltage(){
  static int16_t lpf;
  uint8_t low, high;
  low  = ADCL;  high = ADCH;
  int16_t val = (((high << 8) | low) << 3);
  lpf += (val -  lpf) >> 2;
  return lpf;
}

inline void adcLoop() {
  if(isAdcConversionFinished()) {
    if (adcMuxPin() == fwdPwrPin) {
      fwdPwrVal = adcGetForwardVoltage();
      adcStartconversion(rflPwrPin);
    } else {
      rflPwrVal = adcGetReflectedVoltage();
      adcStartconversion(fwdPwrPin);
    }
    adcCnt++;
  }
}

#endif /* ADC_H_ */
