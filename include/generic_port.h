#ifndef GENERIC_PORT_H_
#define GENERIC_PORT_H_

#include <common/mavlink.h>

class Generic_Port
{
public:
	Generic_Port() {};
	virtual ~Generic_Port() {};
	virtual int read_message(mavlink_message_t &message) = 0;
	virtual int write_message(const mavlink_message_t &message) = 0;
	virtual bool is_running() = 0;
	virtual void start() = 0;
	virtual void stop() = 0;
};

#endif // GENERIC_PORT_H_
