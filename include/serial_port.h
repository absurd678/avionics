#ifndef SERIAL_PORT_H_
#define SERIAL_PORT_H_

#include <cstdlib>
#include <stdio.h>	 // Standard input/output definitions
#include <unistd.h>	 // UNIX standard function definitions
#include <fcntl.h>	 // File control definitions
#include <termios.h> // POSIX terminal control definitions
#include <pthread.h> // This uses POSIX Threads
#include <signal.h>

#include "generic_port.h"

#ifndef B460800
#define B460800 460800
#endif

#ifndef B921600
#define B921600 921600
#endif

class Serial_Port : public Generic_Port
{

public:
	Serial_Port();
	Serial_Port(const char *uart_name_, int baudrate_);
	virtual ~Serial_Port();

	int read_message(mavlink_message_t &message);
	int write_message(const mavlink_message_t &message);

	bool is_running()
	{
		return is_open;
	}
	void start();
	void stop();

private:
	int fd;
	mavlink_status_t lastStatus;
	pthread_mutex_t lock;

	void initialize_defaults();

	bool debug;
	const char *uart_name;
	int baudrate;
	bool is_open;

	int _open_port(const char *port);
	bool _setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control);
	int _read_port(uint8_t &cp);
	int _write_port(char *buf, unsigned len);
};

#endif // SERIAL_PORT_H_
