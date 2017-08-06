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

inline void busInit(PROTO_Receiver rcv) {
	bus.strategy.set_pin(2);
	bus.set_receiver(__busReceiver);
	__rcv = rcv;
}

inline void busLoop() {
	bus.receive();
	bus.update();
}


#endif /* PROTO_COMMON_H_ */
