#include "serial_port.h"

/**
 * @brief Конструктор класса Serial_Port.
 * 
 * @param uart_name_ Имя UART устройства.
 * @param baudrate_ Скорость передачи данных (бод).
 */
Serial_Port::Serial_Port(const char *uart_name_, int baudrate_)
{
    initialize_defaults();
    uart_name = uart_name_;
    baudrate = baudrate_;
}

/**
 * @brief Конструктор по умолчанию класса Serial_Port.
 */
Serial_Port::Serial_Port()
{
    initialize_defaults();
}

/**
 * @brief Деструктор класса Serial_Port.
 * 
 * Уничтожает мьютекс.
 */
Serial_Port::~Serial_Port()
{
    // destroy mutex
    pthread_mutex_destroy(&lock);
}

/**
 * @brief Инициализирует значения по умолчанию для атрибутов.
 */
void Serial_Port::initialize_defaults()
{
    // Initialize attributes
    debug = false;
    fd = -1;
    is_open = false;

    uart_name = (char *)"/dev/ttyUSB0";
    baudrate = 57600;

    // Start mutex
    int result = pthread_mutex_init(&lock, NULL);
    if (result != 0)
    {
        printf("\n mutex init failed\n");
        throw 1;
    }
}

// ------------------------------------------------------------------------------
//   Read from Serial
// ------------------------------------------------------------------------------

/**
 * @brief Читает сообщение из последовательного порта.
 * 
 * @param message Ссылка на объект сообщения Mavlink, в который будет записано прочитанное сообщение.
 * @return int Возвращает true, если сообщение было успешно прочитано.
 */
int Serial_Port::read_message(mavlink_message_t &message)
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
        fprintf(stderr, "ERROR: Could not read from fd %d\n", fd);
    }

    if (msgReceived && debug)
    {
        // Report info
        printf("Received message from serial with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);

        fprintf(stderr, "Received serial data: ");
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

/**
 * @brief Записывает сообщение в последовательный порт.
 * 
 * @param message Константная ссылка на объект сообщения Mavlink, которое будет отправлено.
 * @return int Количество байт, записанных в порт.
 */
int Serial_Port::write_message(const mavlink_message_t &message)
{
    char buf[300];

    // Translate message to buffer
    unsigned len = mavlink_msg_to_send_buffer((uint8_t *)buf, &message);

    // Write buffer to serial port, locks port while writing
    int bytesWritten = _write_port(buf, len);

    return bytesWritten;
}

/**
 * @brief Открывает последовательный порт и настраивает его.
 */
void Serial_Port::start()
{
    printf("OPEN PORT\n");

    fd = _open_port(uart_name);

    // Check success
    if (fd == -1)
    {
        printf("failure, could not open port.\n");
        throw EXIT_FAILURE;
    }

    bool success = _setup_port(baudrate, 8, 1, false, false);

    if (!success)
    {
        printf("failure, could not configure port.\n");
        throw EXIT_FAILURE;
    }
    if (fd <= 0)
    {
        printf("Connection attempt to port %s with %d baud, 8N1 failed, exiting.\n", uart_name, baudrate);
        throw EXIT_FAILURE;
    }

    printf("Connected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", uart_name, baudrate);
    lastStatus.packet_rx_drop_count = 0;

    is_open = true;

    printf("\n");

    return;
}

/**
 * @brief Закрывает последовательный порт.
 */
void Serial_Port::stop()
{
    printf("CLOSE PORT\n");

    int result = close(fd);

    if (result)
    {
        fprintf(stderr, "WARNING: Error on port close (%i)\n", result);
    }

    is_open = false;

    printf("\n");
}

/**
 * @brief Открывает указанный порт.
 * 
 * @param port Имя порта для открытия.
 * @return int Дескриптор файла порта.
 */
int Serial_Port::_open_port(const char *port)
{
    fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);

    // Check for Errors
    if (fd == -1)
    {
        /* Could not open the port. */
        return (-1);
    }

    // Finalize
    else
    {
        fcntl(fd, F_SETFL, 0);
    }

    // Done!
    return fd;
}

/**
 * @brief Настраивает параметры порта.
 * 
 * @param baud Скорость передачи данных (бод).
 * @param data_bits Количество бит данных.
 * @param stop_bits Количество стоп-битов.
 * @param parity Использование четности.
 * @param hardware_control Аппаратное управление потоком.
 * @return true если настройка успешна.
 * @return false если произошла ошибка при настройке.
 */
bool Serial_Port::_setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
{
    // Check file descriptor
    if (!isatty(fd))
    {
        fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n", fd);
        return false;
    }

    // Read file descritor configuration
    struct termios config;
    if (tcgetattr(fd, &config) < 0)
    {
        fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
        return false;
    }

    // Input flags - Turn off input processing
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                        INLCR | PARMRK | INPCK | ISTRIP | IXON);

    // Output flags - Turn off output processing
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
                        ONOCR | OFILL | OPOST);

#ifdef OLCUC
    config.c_oflag &= ~OLCUC;
#endif

#ifdef ONOEOT
    config.c_oflag &= ~ONOEOT;
#endif

    // No line processing:
    // echo off, echo newline off, canonical mode off,
    // extended input processing off, signal chars off
    config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

    // Turn off character processing
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    config.c_cflag &= ~(CSIZE | PARENB);
    config.c_cflag |= CS8;

    // One input byte is enough to return from read()
    // Inter-character timer off
    config.c_cc[VMIN] = 1;
    config.c_cc[VTIME] = 10; // was 0

    // Get the current options for the port
    ////struct termios options;
    ////tcgetattr(fd, &options);

    // Apply baudrate
    switch (baud)
    {
    case 1200:
        if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
        {
            fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
            return false;
        }
        break;
    case 1800:
        cfsetispeed(&config, B1800);
        cfsetospeed(&config, B1800);
        break;
    case 9600:
        cfsetispeed(&config, B9600);
        cfsetospeed(&config, B9600);
        break;
    case 19200:
        cfsetispeed(&config, B19200);
        cfsetospeed(&config, B19200);
        break;
    case 38400:
        if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
        {
            fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
            return false;
        }
        break;
    case 57600:
        if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
        {
            fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
            return false;
        }
        break;
    case 115200:
        if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
        {
            fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
            return false;
        }
        break;

    // These two non-standard (by the 70'ties ) rates are fully supported on
    // current Debian and Mac OS versions (tested since 2010).
    case 460800:
        if (cfsetispeed(&config, B460800) < 0 || cfsetospeed(&config, B460800) < 0)
        {
            fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
            return false;
        }
        break;
    case 921600:
        if (cfsetispeed(&config, B921600) < 0 || cfsetospeed(&config, B921600) < 0)
        {
            fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
            return false;
        }
        break;
    default:
        fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
        return false;

        break;
    }

    // Finally, apply the configuration
    if (tcsetattr(fd, TCSAFLUSH, &config) < 0)
    {
        fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
        return false;
    }

    // Done!
    return true;
}

/**
 * @brief Читает байт из последовательного порта.
 * 
 * @param cp Ссылка на переменную, в которую будет записан прочитанный байт.
 * @return int Результат операции чтения.
 */
int Serial_Port::_read_port(uint8_t &cp)
{
    // Lock
    pthread_mutex_lock(&lock);

    int result = read(fd, &cp, 1);

    // Unlock
    pthread_mutex_unlock(&lock);

    return result;
}

/**
 * @brief Записывает данные в последовательный порт.
 * 
 * @param buf Буфер с данными для записи.
 * @param len Длина данных для записи.
 * @return int Количество записанных байт.
 */
int Serial_Port::_write_port(char *buf, unsigned len)
{
    // Lock
    pthread_mutex_lock(&lock);

    // Write packet via serial link
    const int bytesWritten = static_cast<int>(write(fd, buf, len));

    // Wait until all data has been written
    tcdrain(fd);

    // Unlock
    pthread_mutex_unlock(&lock);

    return bytesWritten;
}
