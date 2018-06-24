#ifndef INPUT_SYMBIAN_H
#define INPUT_SYMBIAN_H

#include <stdio.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <unistd.h>
#include "../../mjpg_streamer.h"

typedef enum {
	PACKET_HEADER = 0,
	PACKET_DATA = 1
} smart_type_t;

typedef enum {
	DISCONNECTED = 0,
	CONNECTED = 1,
	ERROR = 2
} comm_state_t;

typedef struct {
	smart_type_t type;
	size_t len;
} smart_packet_t;

typedef struct {
	uint8_t *buf;
	size_t len; // The total buffer length
	size_t size; // The total frame size
} frame_t;

#define INIT_FRAME \
	{ \
		.buf = NULL, \
		.len = 0, \
		.size = 0 \
	}

#endif
