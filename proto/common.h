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


#define PJON_INCLUDE_SWBB
#define PJON_INCLUDE_ASYNC_ACK true
#define SWBB_MODE 2
#define SWBB_RESPONSE_TIMEOUT 1500
#define SWBB_BACK_OFF_DEGREE     4
#define SWBB_MAX_ATTEMPTS       20

#include <PJON.h>

PJON<SoftwareBitBang> bus(PJON_ID);

enum {
	channelC, channelL
};

enum {
	cmdCalibrate,
	cmdSetPos
};

typedef struct PROTO_PosParams {
	long lPos;
	long cPos;
} PROTO_PosParams;

typedef struct PROTO_CalParams {
	uint8_t channel;
} PROTO_CalParams;

typedef struct TCommand {
	uint8_t id;
	union {
		PROTO_CalParams cal;
		PROTO_PosParams pos;
	};
} TCommand;

typedef void (*PROTO_Receiver)(const TCommand *payload,
		const PJON_Packet_Info &packet_info);

PROTO_Receiver __rcv = NULL;

void __busReceiver(uint8_t *payload, uint16_t length,	const PJON_Packet_Info &packet_info) {
	if (length == sizeof(TCommand) && __rcv != NULL) {
		__rcv((TCommand *) payload, packet_info);
	}
}

void __busErrorHandler(uint8_t code, uint8_t data) {
  if(code == PJON_CONNECTION_LOST) {
    Serial.print("Connection with device ID ");
    Serial.print(bus.packets[data].content[0], DEC);
    Serial.println(" is lost.");
  }
  if(code == PJON_PACKETS_BUFFER_FULL) {
    Serial.print("Packet buffer is full, has now a length of ");
    Serial.println(data, DEC);
    Serial.println("Possible wrong bus configuration!");
    Serial.println("higher PJON_MAX_PACKETS if necessary.");
  }
  if(code == PJON_CONTENT_TOO_LONG) {
    Serial.print("Content is too long, length: ");
    Serial.println(data);
  }
};

inline void busInit(PROTO_Receiver rcv) {
	bus.strategy.set_pin(2);
	bus.set_receiver(__busReceiver);
	bus.set_asynchronous_acknowledge(true);
	bus.set_error(__busErrorHandler);
	__rcv = rcv;
}

inline void busLoop() {
	bus.receive();
	bus.update();
}


#endif /* PROTO_COMMON_H_ */
