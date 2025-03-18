#include "tcp_server.h"

/**
 * @brief Конструктор класса TCP_Server с указанием порта.
 * 
 * @param tcp_port_ Порт TCP для сервера.
 */
TCP_Server::TCP_Server(int tcp_port_) {
  initialize_defaults();
  port = tcp_port_;
  is_open = false;
}

/**
 * @brief Конструктор по умолчанию класса TCP_Server.
 */
TCP_Server::TCP_Server() { initialize_defaults(); }

/**
 * @brief Деструктор класса TCP_Server.
 * 
 * Уничтожает мьютекс.
 */
TCP_Server::~TCP_Server() {
  // destroy mutex
  pthread_mutex_destroy(&lock);
}

/**
 * @brief Инициализирует значения по умолчанию для атрибутов.
 */
void TCP_Server::initialize_defaults() {
  // Initialize attributes
  port = 5600;
  is_open = false;
  debug = false;
  sockfd = -1;

  // Start mutex
  int result = pthread_mutex_init(&lock, NULL);
  if (result != 0) {
    printf("\n mutex init failed\n");
    throw 1;
  }
}

/**
 * @brief Читает сообщение из TCP соединения.
 * 
 * @param message Ссылка на объект сообщения Mavlink, в который будет записано прочитанное сообщение.
 * @return int Возвращает true, если сообщение было успешно прочитано.
 */
int TCP_Server::read_message(mavlink_message_t &message) {
  uint8_t cp;
  mavlink_status_t status;
  uint8_t msgReceived = false;

  // this function locks the port during read
  int result = _read_port(cp);

  if (result > 0) {
    // the parsing
    msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);

    // check for dropped packets
    if ((lastStatus.packet_rx_drop_count != status.packet_rx_drop_count) &&
        debug) {
      printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
      unsigned char v = cp;
      fprintf(stderr, "%02x ", v);
    }
    lastStatus = status;
  }

  // Couldn't read from port
  else {
    fprintf(stderr, "ERROR: Could not read, res = %d, errno = %d : %m\n",
            result, errno);
  }

  if (msgReceived && debug) {
    // Report info
    printf("Received message from TCP with ID #%d (sys:%d|comp:%d):\n",
           message.msgid, message.sysid, message.compid);

    fprintf(stderr, "Received TCP data: ");
    unsigned int i;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    // check message is write length
    unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);

    // message length error
    if (messageLength > MAVLINK_MAX_PACKET_LEN) {
      fprintf(stderr,
              "\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
    }

    // print out the buffer
    else {
      for (i = 0; i < messageLength; i++) {
        unsigned char v = buffer[i];
        fprintf(stderr, "%02x ", v);
      }
      fprintf(stderr, "\n");
    }
  }

  // Done!
  return msgReceived;
}

/**
 * @brief Записывает сообщение в TCP соединение.
 * 
 * @param message Константная ссылка на объект сообщения Mavlink, которое будет отправлено.
 * @return int Количество байт, записанных в соединение.
 */
int TCP_Server::write_message(const mavlink_message_t &message) {
  char buf[300];

  // Translate message to buffer
  unsigned len = mavlink_msg_to_send_buffer((uint8_t *)buf, &message);

  // Write buffer to TCP port, locks port while writing
  int bytesWritten = _write_port(buf, len);
  if (bytesWritten < 0) {
    fprintf(stderr, "ERROR: Could not write, res = %d, errno = %d : %m\n",
            bytesWritten, errno);
  }

  return bytesWritten;
}

/**
 * @brief Запускает TCP сервер и ожидает подключения клиента.
 */
void TCP_Server::start() {

  /* Create socket */
  int  len;
  struct sockaddr_in servaddr, cli;

  // socket create and verification
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd == -1) {
    printf("socket creation failed...\n");
    exit(0);
  } else
    printf("Socket successfully created..\n");
  bzero(&servaddr, sizeof(servaddr));

  // assign IP, PORT
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  servaddr.sin_port = htons(port);

  // Binding newly created socket to given IP and verification
  if ((bind(sockfd, (sockaddr *)&servaddr, sizeof(servaddr))) != 0) {
    printf("socket bind failed...\n");
    exit(0);
  } else
    printf("Socket successfully binded..\n");

  // Now server is ready to listen and verification
  if ((listen(sockfd, 5)) != 0) {
    printf("Listen failed...\n");
    exit(0);
  } else
    printf("Server listening..\n");
  len = sizeof(cli);

  // Accept the data packet from client and verification
  socklen_t l = len;
  connfd = accept(sockfd, (sockaddr *)&cli, &l);
  if (connfd < 0) {
    printf("server accept failed...\n");
    exit(0);
  } else
    printf("server accept the client...\n");

  return;
}

/**
 * @brief Останавливает TCP сервер и закрывает соединение.
 */
void TCP_Server::stop() {
  printf("CLOSE PORT\n");

  int result = close(sockfd);
  sockfd = -1;

  if (result) {
    fprintf(stderr, "WARNING: Error on port close (%i)\n", result);
  }

  is_open = false;

  printf("\n");
}

/**
 * @brief Читает байт из TCP соединения.
 * 
 * @param cp Ссылка на переменную, в которую будет записан прочитанный байт.
 * @return int Результат операции чтения.
 */
int TCP_Server::_read_port(uint8_t &cp) {

  socklen_t len;

  // Lock
  pthread_mutex_lock(&lock);

  int result = -1;
  if (buff_ptr < buff_len) {
    cp = buff[buff_ptr];
    buff_ptr++;
    result = 1;
  } else {
    // struct sockaddr_in addr;
    len = sizeof(struct sockaddr_in);
    result = read(connfd, buff, sizeof(buff));

    if (result > 0) {
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

/**
 * @brief Записывает данные в TCP соединение.
 * 
 * @param buf Буфер с данными для записи.
 * @param len Длина данных для записи.
 * @return int Количество записанных байт.
 */
int TCP_Server::_write_port(char *buf, unsigned len) {

  // Lock
  pthread_mutex_lock(&lock);

  // Write packet via TCP link
  int bytesWritten = 0;

  bytesWritten = write(connfd, buff, sizeof(buff));
  printf("sendto: %i\n", bytesWritten);

  // Unlock
  pthread_mutex_unlock(&lock);

  return bytesWritten;
}
