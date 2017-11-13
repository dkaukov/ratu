/*
 * common.h
 *
 *  Created on: 6Aug.,2017
 *      Author: ziss
 *
 *
 *
 */

#ifndef PROTO_COMMON_H_
#define PROTO_COMMON_H_

enum {
  channelC1, channelC2, channelL
};

enum {
  cmdCalibrate, cmdSetPos, cmdStatus, cmdAutoTune, cmdFineTune, cmdDebug, cmdStatusReq
};

typedef struct PROTO_AbsPosParams {
  uint32_t lPos;
  uint32_t c1Pos;
  uint32_t c2Pos;
} PROTO_AbsPosParams;

typedef struct PROTO_DeltaPosParams {
  int16_t lPos;
  int16_t c1Pos;
  int16_t c2Pos;
} PROTO_DeltaPosParams;

typedef struct PROTO_AdcValues {
  uint16_t fwd;
  uint16_t rfl;
} PROTO_AdcValues;

typedef struct PROTO_StringValue {
  uint8_t len;
  char str[32];
} PROTO_StringValue;


typedef struct PROTO_CalParams {
  uint8_t channel;
} PROTO_CalParams;

typedef struct PROTO_MechStatus {
  uint8_t flags;
  uint16_t cnt;
  PROTO_AdcValues adc;
  PROTO_AbsPosParams pos;
} PROTO_MechStatus;

typedef struct TCommand {
  uint8_t id;
  union {
    PROTO_CalParams cal;
    PROTO_DeltaPosParams pos;
    PROTO_MechStatus status;
    PROTO_StringValue str;
  };
} TCommand;

#define PJON_INCLUDE_TS

#define PJON_MAX_PACKETS        4
#define PJON_PACKET_MAX_LENGTH  sizeof(TCommand) + 13
#define PJON_INCLUDE_ASYNC_ACK  false

#define TS_BITRATE_SCALER     3
#define TS_BYTE_TIME_OUT      1200 / TS_BITRATE_SCALER
#define TS_COLLISION_DELAY    2400 / TS_BITRATE_SCALER
#define TS_RESPONSE_TIME_OUT  20000
#define TS_TIME_IN            1200 / TS_BITRATE_SCALER
#define TS_BACK_OFF_DEGREE    4
#define TS_MAX_ATTEMPTS       3
#define TS_PORT_BITRATE       9600 * TS_BITRATE_SCALER

//#define PJON_INCLUDE_SWBB
#define SWBB_MODE               1
#define SWBB_RESPONSE_TIMEOUT   15000
#define SWBB_BACK_OFF_DEGREE    4
#define SWBB_MAX_ATTEMPTS       20

#include <PJON.h>
#include "RS485Serial.h"

//PJON<SoftwareBitBang> bus(PJON_ID);
PJON<RS485Serial> bus(PJON_ID);

typedef void (*PROTO_Receiver)(const TCommand *payload,
    const PJON_Packet_Info &packet_info);

PROTO_Receiver __rcv = NULL;

void __busReceiver(uint8_t *payload, uint16_t length, const PJON_Packet_Info &packet_info) {
  if (length <= sizeof(TCommand) && __rcv != NULL) {
    __rcv((TCommand *) payload, packet_info);
  }
}

void __busErrorHandler(uint8_t code, uint8_t data) {
  /*
  if (code == PJON_CONNECTION_LOST) {
    Serial.print("Connection with device ID ");
    Serial.print(bus.packets[data].content[0], DEC);
    Serial.println(" is lost.");
  }
  if (code == PJON_PACKETS_BUFFER_FULL) {
    Serial.print("Packet buffer is full, has now a length of ");
    Serial.println(data, DEC);
    Serial.println("Possible wrong bus configuration!");
    Serial.println("higher PJON_MAX_PACKETS if necessary.");
  }
  if (code == PJON_CONTENT_TOO_LONG) {
    Serial.print("Content is too long, length: ");
    Serial.println(data);
  }
  if (code == PJON_ID_ACQUISITION_FAIL) {
    Serial.print("ID Acqusition fail: ");
    Serial.println(data);
  }
  if (code == PJON_DEVICES_BUFFER_FULL) {
    Serial.print("Device buffer full: ");
    Serial.println(data);
  }
  */
}

inline void busInit(PROTO_Receiver rcv, uint8_t enablePin, Stream *serial_port) {
  bus.strategy.set_serial(serial_port);
  bus.strategy.set_enable_RS485_pin(enablePin);
  bus.set_receiver(__busReceiver);
  bus.set_synchronous_acknowledge(true);
  bus.set_asynchronous_acknowledge(false);
  bus.set_crc_32(false);
  bus.set_shared_network(false);
  bus.set_error(__busErrorHandler);
  __rcv = rcv;
  bus.begin();
}

inline void busLoop() {
  bus.receive();
  bus.update();
}

inline uint16_t mechSetPosition(int16_t l, int16_t c1, int16_t c2) {
  TCommand cmd = { .id = cmdSetPos };
  cmd.pos.lPos = l;
  cmd.pos.c1Pos = c1;
  cmd.pos.c2Pos = c2;
  return bus.send(ID_MECH, (char *) &cmd, sizeof(cmd));
}

inline uint16_t mechCalibrate(uint8_t ch) {
  TCommand cmd = { .id = cmdCalibrate };
  cmd.cal.channel = ch;
  return bus.send(ID_MECH, (char *) &cmd, sizeof(cmd.cal) + sizeof(cmd.id));
}

inline uint16_t halSendStatusUpdate(PROTO_MechStatus &status) {
  TCommand cmd = { .id = cmdStatus };
  cmd.status = status;
  return bus.send(ID_HAL100, (char *) &cmd, sizeof(cmd.status) + sizeof(cmd.id));
}

inline uint16_t halSendStatusUpdateReply(PROTO_MechStatus &status) {
  TCommand cmd = { .id = cmdStatus };
  cmd.status = status;
  return bus.reply((char *) &cmd, sizeof(cmd.status) + sizeof(cmd.id));
}


inline uint16_t mechAutoTune() {
  TCommand cmd = { .id = cmdAutoTune };
  return bus.send(ID_MECH, (char *) &cmd, sizeof(cmd.id));
}

inline uint16_t mechFineTune() {
  TCommand cmd = { .id = cmdFineTune };
  return bus.send(ID_MECH, (char *) &cmd, sizeof(cmd.id));
}

inline uint16_t mechSendStatusRequest() {
  TCommand cmd = { .id = cmdStatusReq };
  return bus.send(ID_MECH, (char *) &cmd, sizeof(cmd.id));
}


#endif /* PROTO_COMMON_H_ */
