/*
 * RS485Serial.h
 *
 *  Created on: 14Nov.,2017
 *      Author: ziss
 */

#ifndef PROTO_RS485SERIAL_H_
#define PROTO_RS485SERIAL_H_

class RS485Serial : public ThroughSerial {
public:

  bool can_start() {
    // Only check we need that buffers are empty
    return !PJON_SERIAL_AVAILABLE(serial);
  };

  uint16_t receive_string(uint8_t *string, uint16_t max_length) {
    // quickly return if buffers are empty.
    if(max_length == PJON_PACKET_MAX_LENGTH) {
      if (!PJON_SERIAL_AVAILABLE(serial)) {
        return TS_FAIL;
      }
    }
    return ThroughSerial::receive_string(string, max_length);
  }

};


#endif /* PROTO_RS485SERIAL_H_ */
