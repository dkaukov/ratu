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


#define PJON_MAX_PACKETS 		4
#define PJON_PACKET_MAX_LENGTH 	30

#define PJON_INCLUDE_SWBB
#define PJON_INCLUDE_ASYNC_ACK 	false
#define SWBB_MODE 				1
#define SWBB_RESPONSE_TIMEOUT 	15000
#define SWBB_BACK_OFF_DEGREE	4
#define SWBB_MAX_ATTEMPTS		20

#include <PJON.h>

PJON<SoftwareBitBang> bus(PJON_ID);

enum {
	channelC1, channelC2, channelL
};

enum {
	cmdCalibrate,
	cmdSetPos,
	cmdStatus
};

typedef struct PROTO_PosParams {
	long lPos;
	long c1Pos;
	long c2Pos;
} PROTO_PosParams;

typedef struct PROTO_AdcValues {
	uint16_t fwd;
	uint16_t rfl;
} PROTO_AdcValues;

typedef struct PROTO_CalParams {
	uint8_t channel;
} PROTO_CalParams;

typedef struct PROTO_MechStatus {
	uint8_t flags;
	uint16_t cnt;
	PROTO_AdcValues adc;
} PROTO_MechStatus;


typedef struct TCommand {
	uint8_t id;
	union {
		PROTO_CalParams cal;
		PROTO_PosParams pos;
		PROTO_MechStatus status;
	};
} TCommand;



typedef void (*PROTO_Receiver)(const TCommand *payload,
		const PJON_Packet_Info &packet_info);

PROTO_Receiver __rcv = NULL;

void __busReceiver(uint8_t *payload, uint16_t length, const PJON_Packet_Info &packet_info) {
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
  if(code == PJON_ID_ACQUISITION_FAIL) {
    Serial.print("ID Acqusition fail: ");
    Serial.println(data);
  }
  if(code == PJON_DEVICES_BUFFER_FULL) {
      Serial.print("Device buffer full: ");
      Serial.println(data);
  }
};

inline void busInit(PROTO_Receiver rcv) {
	bus.strategy.set_pin(2);
	bus.set_receiver(__busReceiver);
	bus.set_asynchronous_acknowledge(false);
	bus.set_error(__busErrorHandler);
	__rcv = rcv;
}

inline void busLoop() {
	bus.receive();
	bus.update();
}

inline uint16_t mechSetPosition(long l, long c1, long c2) {
	TCommand cmd = {
		.id = cmdSetPos
	};
	cmd.pos.lPos = l;
	cmd.pos.c1Pos = c1;
	cmd.pos.c2Pos = c2;
	return bus.send_packet_blocking(ID_MECH, (char *)&cmd, sizeof(cmd));
}

inline uint16_t mechCalibrate(uint8_t ch) {
	TCommand cmd = {
		.id = cmdCalibrate
	};
	cmd.cal.channel = ch;
	return bus.send(ID_MECH, (char *)&cmd, sizeof(cmd));
}

inline uint16_t halSendStatusUpdate(PROTO_MechStatus &status) {
	TCommand cmd = {
		.id = cmdStatus
	};
	cmd.status = status;
	return bus.send(ID_HAL100, (char *)&cmd, sizeof(cmd));
}


#endif /* PROTO_COMMON_H_ */
