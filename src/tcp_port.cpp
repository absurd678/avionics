#include "tcp_port.h"

TCP_Port::
	TCP_Port(int tcp_port_)
{
	initialize_defaults();
	port = tcp_port_;
	is_open = false;
}

TCP_Port::
	TCP_Port()
{
	initialize_defaults();
}

TCP_Port::
	~TCP_Port()
{
	// destroy mutex
	pthread_mutex_destroy(&lock);
}

void TCP_Port::
	initialize_defaults()
{
	// Initialize attributes
	port = 5600;
	is_open = false;
	debug = false;
	sock = -1;

	// Start mutex
	int result = pthread_mutex_init(&lock, NULL);
	if (result != 0)
	{
		printf("\n mutex init failed\n");
		throw 1;
	}
}

int TCP_Port::
	read_message(mavlink_message_t &message)
{
	uint8_t cp;
	mavlink_status_t status;
	uint8_t msgReceived = false;

	// this function locks the port during read
	int result = _read_port(cp);

	if (result > 0)
	{
		// the parsing
		msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);

		// check for dropped packets
		if ((lastStatus.packet_rx_drop_count != status.packet_rx_drop_count) && debug)
		{
			printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
			unsigned char v = cp;
			fprintf(stderr, "%02x ", v);
		}
		lastStatus = status;
	}

	// Couldn't read from port
	else
	{
		fprintf(stderr, "ERROR: Could not read, res = %d, errno = %d : %m\n", result, errno);
	}

	if (msgReceived && debug)
	{
		// Report info
		printf("Received message from TCP with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);

		fprintf(stderr, "Received TCP data: ");
		unsigned int i;
		uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

		// check message is write length
		unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);

		// message length error
		if (messageLength > MAVLINK_MAX_PACKET_LEN)
		{
			fprintf(stderr, "\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
		}

		// print out the buffer
		else
		{
			for (i = 0; i < messageLength; i++)
			{
				unsigned char v = buffer[i];
				fprintf(stderr, "%02x ", v);
			}
			fprintf(stderr, "\n");
		}
	}

	// Done!
	return msgReceived;
}

int TCP_Port::
	write_message(const mavlink_message_t &message)
{
	char buf[300];

	// Translate message to buffer
	unsigned len = mavlink_msg_to_send_buffer((uint8_t *)buf, &message);

	// Write buffer to TCP port, locks port while writing
	int bytesWritten = _write_port(buf, len);
	if (bytesWritten < 0)
	{
		fprintf(stderr, "ERROR: Could not write, res = %d, errno = %d : %m\n", bytesWritten, errno);
	}

	return bytesWritten;
}

void TCP_Port::
	start()
{

	/* Create socket */
    int sockfd, connfd, len; 
    struct sockaddr_in servaddr, cli; 
  
    // socket create and verification 
    sockfd = socket(AF_INET, SOCK_STREAM, 0); 
    if (sockfd == -1) { 
        printf("socket creation failed...\n"); 
        exit(0); 
    } 
    else
        printf("Socket successfully created..\n"); 
    bzero(&servaddr, sizeof(servaddr)); 
  
    // assign IP, PORT 
    servaddr.sin_family = AF_INET; 
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY); 
    servaddr.sin_port = htons(port); 
  
    // Binding newly created socket to given IP and verification 
    if ((bind(sockfd, (sockaddr*)&servaddr, sizeof(servaddr))) != 0) { 
        printf("socket bind failed...\n"); 
        exit(0); 
    } 
    else
        printf("Socket successfully binded..\n"); 
  
    // Now server is ready to listen and verification 
    if ((listen(sockfd, 5)) != 0) { 
        printf("Listen failed...\n"); 
        exit(0); 
    } 
    else
        printf("Server listening..\n"); 
    len = sizeof(cli); 
  
    // Accept the data packet from client and verification 
    socklen_t l = len;
    connfd = accept(sockfd, (sockaddr*)&cli, &l); 
    if (connfd < 0) { 
        printf("server accept failed...\n"); 
        exit(0); 
    } 
    else
        printf("server accept the client...\n"); 

	return;
}

void TCP_Port::
	stop()
{
	printf("CLOSE PORT\n");

	int result = close(sock);
	sock = -1;

	if (result)
	{
		fprintf(stderr, "WARNING: Error on port close (%i)\n", result);
	}

	is_open = false;

	printf("\n");
}

int TCP_Port::
	_read_port(uint8_t &cp)
{

	socklen_t len;

	// Lock
	pthread_mutex_lock(&lock);

	int result = -1;
	if (buff_ptr < buff_len)
	{
		cp = buff[buff_ptr];
		buff_ptr++;
		result = 1;
	}
	else
	{
		struct sockaddr_in addr;
		len = sizeof(struct sockaddr_in);
		result = recvfrom(sock, &buff, BUFF_LEN, 0, (struct sockaddr *)&addr, &len);
		if (tx_port < 0)
		{
			if (strcmp(inet_ntoa(addr.sin_addr), target_ip) == 0)
			{
				tx_port = ntohs(addr.sin_port);
				printf("Got first packet, sending to %s:%i\n", target_ip, rx_port);
			}
			else
			{
				printf("ERROR: Got packet from %s:%i but listening on %s\n", inet_ntoa(addr.sin_addr), ntohs(addr.sin_port), target_ip);
			}
		}
		if (result > 0)
		{
			buff_len = result;
			buff_ptr = 0;
			cp = buff[buff_ptr];
			buff_ptr++;
			// printf("recvfrom: %i %i\n", result, cp);
		}
	}

	// Unlock
	pthread_mutex_unlock(&lock);

	return result;
}

int TCP_Port::
	_write_port(char *buf, unsigned len)
{

	// Lock
	pthread_mutex_lock(&lock);

	// Write packet via TCP link
	int bytesWritten = 0;
	if (tx_port > 0)
	{
		struct sockaddr_in addr;
		memset(&addr, 0, sizeof(addr));
		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = inet_addr(target_ip);
		addr.sin_port = htons(tx_port);
		bytesWritten = sendto(sock, buf, len, 0, (struct sockaddr *)&addr, sizeof(struct sockaddr_in));
		printf("sendto: %i\n", bytesWritten);
	}
	else
	{
		printf("ERROR: Sending before first packet received!\n");
		bytesWritten = -1;
	}

	// Unlock
	pthread_mutex_unlock(&lock);

	return bytesWritten;
}
