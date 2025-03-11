#ifndef TCP_Server_H_
#define TCP_Server_H_

#include <cstdlib>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#include <stdbool.h>

#include <common/mavlink.h>

#include "generic_port.h"

class TCP_Server : public Generic_Port
{

public:
	TCP_Server();
	TCP_Server(int tcp_port_);
	virtual ~TCP_Server();

	int read_message(mavlink_message_t &message);
	int write_message(const mavlink_message_t &message);

	bool is_running()
	{
		return is_open;
	}
	void start();
	void stop();

private:
	mavlink_status_t lastStatus;
	pthread_mutex_t lock;

	void initialize_defaults();

	const static int BUFF_LEN = 2041;
	char buff[BUFF_LEN];
	int buff_ptr;
	int buff_len;
	bool debug;
	int port;
	int sock;
	bool is_open;

	int _read_port(uint8_t &cp);
	int _write_port(char *buf, unsigned len);
};

#endif // TCP_Server_H_
