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
  cmdCalibrate = 0x10, cmdSetPos, cmdStatus, cmdAutoTune, cmdFineTune, cmdDebug, cmdStatusReq
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


#define TS_PORT_BITRATE 28800

#include "ICSC.h"

ICSC icsc(PJON_ID);

typedef void (*PROTO_Receiver)(const TCommand *payload);

PROTO_Receiver __rcv = NULL;

void __icscReceiver(uint8_t sender, char cmd, uint8_t length, uint8_t *payload) {
  if (length <= sizeof(TCommand) && __rcv != NULL) {
    __rcv((TCommand *) payload);
  }
}

inline void busInit(PROTO_Receiver rcv, uint8_t enablePin, Stream *serial_port) {
  __rcv = rcv;
  icsc.setStream(serial_port);
  icsc.registerCommand(ICSC_CATCH_ALL, __icscReceiver);
  icsc.setDePin(enablePin);
  icsc.begin();
}

inline void busLoop() {
  icsc.process();
}

inline boolean mechSetPosition(int16_t l, int16_t c1, int16_t c2) {
  TCommand cmd = { .id = cmdSetPos };
  cmd.pos.lPos = l;
  cmd.pos.c1Pos = c1;
  cmd.pos.c2Pos = c2;
  return icsc.send(ID_MECH, cmdSetPos, sizeof(cmd.pos) + sizeof(cmd.id), (char *) &cmd);
}

inline boolean mechCalibrate(uint8_t ch) {
  TCommand cmd = { .id = cmdCalibrate };
  cmd.cal.channel = ch;
  return icsc.send(ID_MECH, cmdCalibrate, sizeof(cmd.cal) + sizeof(cmd.id), (char *) &cmd);
}

inline boolean halSendStatusUpdate(PROTO_MechStatus &status) {
  TCommand cmd = { .id = cmdStatus };
  cmd.status = status;
  return icsc.broadcast(cmdStatus, sizeof(cmd.status) + sizeof(cmd.id), (char *) &cmd);
}

inline boolean halSendStatusUpdateReply(PROTO_MechStatus &status) {
  TCommand cmd = { .id = cmdStatus };
  cmd.status = status;
  return icsc.send(ID_HAL100, cmdStatus, sizeof(cmd.status) + sizeof(cmd.id), (char *) &cmd);
}


inline boolean mechAutoTune() {
  TCommand cmd = { .id = cmdAutoTune };
  return icsc.send(ID_MECH, cmdAutoTune, sizeof(cmd.id), (char *) &cmd);
}

inline boolean mechFineTune() {
  TCommand cmd = { .id = cmdFineTune };
  return icsc.send(ID_MECH, cmdFineTune, sizeof(cmd.id), (char *) &cmd);
}

inline boolean mechSendStatusRequest() {
  TCommand cmd = { .id = cmdStatusReq };
  return icsc.send(ID_MECH, cmdStatusReq, sizeof(cmd.id), (char *) &cmd);
}


#endif /* PROTO_COMMON_H_ */
