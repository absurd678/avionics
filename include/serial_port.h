#ifndef SERIAL_PORT_H_
#define SERIAL_PORT_H_

#include <cstdlib>
#include <stdio.h>   // Standard input/output definitions
#include <unistd.h>  // UNIX standard function definitions
#include <fcntl.h>   // File control definitions
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

/**
 * @brief Класс для работы с последовательным портом.
 * 
 * Этот класс предоставляет методы для чтения и записи сообщений через последовательный порт,
 * а также для управления состоянием порта.
 */
class Serial_Port : public Generic_Port
{

public:
    /**
     * @brief Конструктор по умолчанию.
     */
    Serial_Port();

    /**
     * @brief Конструктор с указанием имени UART и скорости передачи данных.
     * 
     * @param uart_name_ Имя UART устройства.
     * @param baudrate_ Скорость передачи данных (бод).
     */
    Serial_Port(const char *uart_name_, int baudrate_);

    /**
     * @brief Деструктор.
     */
    virtual ~Serial_Port();

    /**
     * @brief Читает сообщение из последовательного порта.
     * 
     * @param message Ссылка на объект сообщения Mavlink, в который будет записано прочитанное сообщение.
     * @return int Возвращает true, если сообщение было успешно прочитано.
     */
    int read_message(mavlink_message_t &message);

    /**
     * @brief Записывает сообщение в последовательный порт.
     * 
     * @param message Константная ссылка на объект сообщения Mavlink, которое будет отправлено.
     * @return int Количество байт, записанных в порт.
     */
    int write_message(const mavlink_message_t &message);

    /**
     * @brief Проверяет, запущен ли порт.
     * 
     * @return true если порт запущен.
     * @return false если порт не запущен.
     */
    bool is_running()
    {
        return is_open;
    }

    /**
     * @brief Открывает последовательный порт и настраивает его.
     */
    void start();

    /**
     * @brief Закрывает последовательный порт.
     */
    void stop();
private:
    int fd; ///< Дескриптор файла порта.
    mavlink_status_t lastStatus; ///< Статус последнего сообщения Mavlink.
    pthread_mutex_t lock; ///< Мьютекс для синхронизации доступа к порту.

    /**
     * @brief Инициализирует значения по умолчанию для атрибутов.
     */
    void initialize_defaults();

    bool debug; ///< Флаг для включения режима отладки.
    const char *uart_name; ///< Имя UART устройства.
    int baudrate; ///< Скорость передачи данных (бод).
    bool is_open; ///< Флаг, указывающий, открыт ли порт.

    /**
     * @brief Открывает указанный порт.
     * 
     * @param port Имя порта для открытия.
     * @return int Дескриптор файла порта.
     */
    int _open_port(const char *port);

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
    bool _setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control);

    /**
     * @brief Читает байт из последовательного порта.
     * 
     * @param cp Ссылка на переменную, в которую будет записан прочитанный байт.
     * @return int Результат операции чтения.
     */
    int _read_port(uint8_t &cp);

    /**
     * @brief Записывает данные в последовательный порт.
     * 
     * @param buf Буфер с данными для записи.
     * @param len Длина данных для записи.
     * @return int Количество записанных байт.
     */
    int _write_port(char *buf, unsigned len);
};

#endif // SERIAL_PORT_H_
