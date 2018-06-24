/*******************************************************************************
#                                                                              #
#      MJPG-streamer allows to stream JPG frames from an input-plugin          #
#      to several output plugins                                               #
#                                                                              #
#      Copyright (C) 2007 Tom Stöveken                                         #
#                                                                              #
# This program is free software; you can redistribute it and/or modify         #
# it under the terms of the GNU General Public License as published by         #
# the Free Software Foundation; version 2 of the License.                      #
#                                                                              #
# This program is distributed in the hope that it will be useful,              #
# but WITHOUT ANY WARRANTY; without even the implied warranty of               #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                #
# GNU General Public License for more details.                                 #
#                                                                              #
# You should have received a copy of the GNU General Public License            #
# along with this program; if not, write to the Free Software                  #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA    #
#                                                                              #
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <getopt.h>
#include <pthread.h>
#include <syslog.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include <error.h>
#include <errno.h>

#include "../../mjpg_streamer.h"
#include "../../utils.h"
#include "input_symbian.h"

#define INPUT_PLUGIN_NAME "Symbian (smart camera) input plugin"

/* private functions and variables to this plugin */
static pthread_t   worker;
static globals     *pglobal;

void *worker_thread(void *);
void worker_cleanup(void *);
void help(void);

/* global variables for this plugin */
static int plugin_number;
static uint16_t port;
int bind_sock = -1;
int client_sock = -1;
uint64_t dropped = 0;

static frame_t frames[2] = { INIT_FRAME, INIT_FRAME };
static frame_t header = INIT_FRAME;
int frame_idx = 0;

static int start_server(int port)
{
	struct sockaddr_in sin;

	// Initialize the addr
	sin.sin_family = AF_INET;
	sin.sin_addr.s_addr = INADDR_ANY;
	sin.sin_port = htons(port);

	bind_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if(bind_sock == -1)
	{
		fprintf(stderr, 
			"Could not create inet socket: %d\n(%s)", errno,
			strerror(errno));
		return -1;
	}

	// Bind the socket to the address returned
	if(bind(bind_sock, (struct sockaddr*)&sin, sizeof(sin)) < 0)
	{
		fprintf(stderr, "Could not bind inet socket: %d\n(%s)", errno,
				strerror(errno));
		close(bind_sock);
		return -1;
	}

	if(listen(bind_sock, 1) < 0)
	{
		fprintf(stderr, "Could not listen on inet socket: %d\n(%s)", errno,
				strerror(errno));
		close(bind_sock);
		return -1;
	}

	DBG("Server started on port: %d\n", port);

	return 0;
}

/*** plugin interface functions ***/
int input_init(input_parameter *param, int id)
{
    int i;
    plugin_number = id;

    param->argv[0] = INPUT_PLUGIN_NAME;

    /* show all parameters for DBG purposes */
    for(i = 0; i < param->argc; i++) {
        DBG("argv[%d]=%s\n", i, param->argv[i]);
    }

    reset_getopt();
    while(1) {
    	char *endptr;
        int option_index = 0, c = 0;
        static struct option long_options[] = {
            {"h", no_argument, 0, 0},
            {"help", no_argument, 0, 0},
            {"p", required_argument, 0, 0},
            {"port", required_argument, 0, 0},
            {0, 0, 0, 0}
        };

        c = getopt_long_only(param->argc, param->argv, "", long_options, &option_index);

        /* no more options to parse */
        if(c == -1) break;

        /* unrecognized option */
        if(c == '?') {
            help();
            return 1;
        }

        switch(option_index) {
            /* h, help */
        case 0:
        case 1:
            DBG("case 0,1\n");
            help();
            return 1;
            break;

            /* d, delay */
        case 2:
        case 3:
            DBG("case 2,3\n");
	    errno = 0;
            port = strtol(optarg, &endptr, 0);
	    if(errno || *endptr) {
		IPRINT("ERROR: Bad TCP port, valid ranges [0,0xFFFF]\n");
		return 1;
	    }
            break;
        default:
            DBG("default case\n");
            help();
            return 1;
        }
    }

    pglobal = param->global;

    IPRINT("Port to watch.....: %d\n", port);

    param->global->in[id].name = malloc((strlen(INPUT_PLUGIN_NAME) + 1) * sizeof(char));
    sprintf(param->global->in[id].name, INPUT_PLUGIN_NAME);

    return 0;
}

int input_stop(int id)
{
    DBG("will cancel input thread\n");
    pthread_cancel(worker);
    return 0;
}

int input_run(int id)
{
	pglobal->in[id].buf = NULL;

	if(start_server(port) < 0)
		return EXIT_FAILURE;

	if(pthread_create(&worker, 0, worker_thread, NULL) != 0) {
		worker_cleanup(NULL);
		exit(EXIT_FAILURE);
	}

	pthread_detach(worker);
	return 0;
}

/*** private functions for this plugin below ***/
void help(void)
{
    fprintf(stderr, " ---------------------------------------------------------------\n" \
    " Help for input plugin..: "INPUT_PLUGIN_NAME"\n" \
    " ---------------------------------------------------------------\n" \
    " The following parameters can be passed to this plugin:\n\n" \
    " [-p | --port ]........: Listen on this port for a smart camera connection\n" \
    " ---------------------------------------------------------------\n");
}

static int resize(frame_t *buf, size_t len)
{
	if(buf->len < len) {
		buf->len += (len / getpagesize() + 1) * getpagesize();
		if(!(buf->buf = realloc(buf->buf, buf->len))) {
			fprintf(stderr, "Out of memory");
			return -1;
		}
	}
	buf->size = len;
	return 0;
}

static void try_swap()
{
	if(!pthread_mutex_trylock(&pglobal->in[plugin_number].db)) {
		struct timeval timestamp;
		frame_t *frame = frames + frame_idx;

		// Hand it over
		pglobal->in[plugin_number].buf = frame->buf;
		pglobal->in[plugin_number].size = frame->size;
		gettimeofday(&timestamp, NULL);
		pglobal->in[plugin_number].timestamp = timestamp;
		DBG("Handed an image over: data = %dB\n", 
				frame->size);
		pthread_cond_broadcast(&pglobal->in[plugin_number].db_update);
		pthread_mutex_unlock(&pglobal->in[plugin_number].db);

		frame_idx = !frame_idx;
	} else {
		DBG("Frame dropped\n");
		dropped++;
	}
}

smart_packet_t* read_hdr()
{
	int ret = 0;
	unsigned char hdr[4] = {0};
	static smart_packet_t pkt;

	ret = recv(client_sock, (char*) hdr, 4, 0);
	if((ret == 0) || (ret == -1))
		return NULL;

	pkt.type = (smart_type_t) (hdr[0]);
	pkt.len = ((unsigned int)hdr[1] << 16) | 
		  ((unsigned int)hdr[2] << 8) | 
		  ((unsigned int)hdr[3]);
	DBG("Got %s: %d\n", pkt.type == PACKET_HEADER ? "header" : "data", 
			pkt.len);

	return &pkt;
}

#define min(a, b) ( (a) < (b) ? (a) : (b) )
static comm_state_t read_pkt(frame_t *frame, smart_packet_t *pkt, 
		size_t start, size_t len)
{
	int ret;
	size_t rx = min(frame->size, start);
	size_t blen = min(pkt->len, len);
	frame->size = rx + blen;
	pkt->len -= blen;

	DBG("Reading to offset %d %dB\n", rx, blen);

	if(resize(frame, frame->size))
		return -1;

	while(rx < frame->size) {
		ret = recv(client_sock,
				((char*) frame->buf) + rx, 
				frame->size - rx, 0);
		// Connection closed or socket error
		if(ret <= 0)
			return ERROR;
		rx += ret;
	}
	return CONNECTED;
}

static int append(frame_t *to, frame_t *from, size_t start, size_t len)
{
	size_t rx = to->size;
	if(resize(to, to->size + len))
		return -1;
	memcpy(to->buf + rx, from->buf + start, len);
	return 0;
}

static comm_state_t handle_data(smart_packet_t *hdr)
{
	size_t hdr_size;
	comm_state_t state;

	frame_t *frame = frames + frame_idx;

	// Get the jpg header size
	if((state = read_pkt(frame, hdr, 0, 6)) != CONNECTED)
		return state;
	hdr_size = (frame->buf[4] << 8) | frame->buf[5];

	// Read the jpg header
	if((state = read_pkt(frame, hdr, 6, hdr_size - 2)) != CONNECTED)
		return state;

	// Append the initial header
	if(append(frame, &header, 2, header.size - 2))
		return -1;

	// Read the rest of the frame
	if((state = read_pkt(frame, hdr, SIZE_MAX, SIZE_MAX)) != CONNECTED)
		return state;

	try_swap();

	return state;
}

static comm_state_t handle_packet()
{
	comm_state_t state;
	smart_packet_t *hdr;

	if(!(hdr = read_hdr()))
		return ERROR;

	if(hdr->type == PACKET_DATA) {
		state = handle_data(hdr);
	} else {
		state = read_pkt(&header, hdr, 0, SIZE_MAX);
		header.size -= 2;
	}

	return state;
}

static comm_state_t smart_connect()
{
	struct sockaddr_in addr;
	socklen_t addr_len = sizeof(addr);

	DBG("Waiting for connections\n");
	while((client_sock = accept(bind_sock, 
					(struct sockaddr*) &addr,
					&addr_len)) == -1)
	{
		if(errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
		    continue;
		fprintf(stderr, 
			"Could not accept inet connection on socket: %d\n(%s)",
			errno, strerror(errno));
		close(client_sock);
		return ERROR;
	}

	char* asc_addr = inet_ntoa(addr.sin_addr);
#ifdef DEBUG
	if(asc_addr != NULL)
		DBG("accepted inet connection from %s\n", asc_addr);
	else
		DBG("accepted inet connection, but inet_ntoa() failed ...\n");
#endif

	return CONNECTED;
}

static comm_state_t smart_disconnect()
{
	if(client_sock >= 0 && close(client_sock))
		fprintf(stderr, "Closing socket: %s\n", strerror(errno));
	client_sock = -1;
	return DISCONNECTED;
}

static void smart_close()
{
	comm_state_t unused = smart_disconnect();
	if(bind_sock >= 0 && close(bind_sock))
		fprintf(stderr, "Closing socket: %s\n", strerror(errno));
	bind_sock = -1;
}

/* the single writer thread */
void *worker_thread(void *arg)
{
	static smart_packet_t *packet = NULL;
	comm_state_t state = DISCONNECTED;
	dropped = 0;

	/* set cleanup handler to cleanup allocated resources */
	pthread_cleanup_push(worker_cleanup, NULL);

	while(!pglobal->stop) {
		switch(state) {
		case DISCONNECTED:
			state = smart_connect();
			break;
		case CONNECTED:
			state = handle_packet();
			break;
		case ERROR:
			state = smart_disconnect();
			break;
		default:
			DBG("Unexpected connection state: %d (%s)\n", 
					state, strerror(errno));
			goto thread_quit;
		}
	}

thread_quit:
	DBG("leaving input thread, calling cleanup function now\n");
	/* call cleanup handler, signal with the parameter */
	pthread_cleanup_pop(1);

	return NULL;
}

void worker_cleanup(void *arg)
{
	static unsigned char first_run = 1;

	if(!first_run) {
		DBG("already cleaned up resources\n");
		return;
	}

	first_run = 0;
	DBG("cleaning up resources allocated by input thread\n");
	IPRINT("Frames dropped: %lld\n", dropped);

	for (int i = 0; i < 2; i++) {
		free(frames[i].buf);
		frames[i].buf = NULL;
		frames[i].len = 0;
		frames[i].size = 0;
	}

	smart_close();
}
