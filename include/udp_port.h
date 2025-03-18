#ifndef UDP_PORT_H_
#define UDP_PORT_H_

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

/**
 * @brief Класс для работы с UDP портом.
 * 
 * Этот класс предоставляет методы для чтения и записи сообщений через UDP соединение,
 * а также для управления состоянием порта.
 */
class UDP_Port : public Generic_Port
{

public:
    /**
     * @brief Конструктор по умолчанию.
     */
    UDP_Port();

    /**
     * @brief Конструктор с указанием IP-адреса и порта UDP.
     * 
     * @param target_ip_ Целевой IP-адрес.
     * @param udp_port_ Порт UDP для соединения.
     */
    UDP_Port(const char *target_ip_, int udp_port_);

    /**
     * @brief Деструктор.
     */
    virtual ~UDP_Port();

    /**
     * @brief Читает сообщение из UDP соединения.
     * 
     * @param message Ссылка на объект сообщения Mavlink, в который будет записано прочитанное сообщение.
     * @return int Возвращает true, если сообщение было успешно прочитано.
     */
    int read_message(mavlink_message_t &message);

    /**
     * @brief Записывает сообщение в UDP соединение.
     * 
     * @param message Константная ссылка на объект сообщения Mavlink, которое будет отправлено.
     * @return int Количество байт, записанных в соединение.
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
     * @brief Открывает UDP порт и настраивает его.
     */
    void start();

    /**
     * @brief Закрывает UDP порт.
     */
    void stop();
private:
    mavlink_status_t lastStatus; ///< Статус последнего сообщения Mavlink.
    pthread_mutex_t lock; ///< Мьютекс для синхронизации доступа к порту.

    /**
     * @brief Инициализирует значения по умолчанию для атрибутов.
     */
    void initialize_defaults();

    const static int BUFF_LEN = 2041; ///< Длина буфера для чтения данных.
    char buff[BUFF_LEN]; ///< Буфер для чтения данных.
    int buff_ptr; ///< Указатель на текущую позицию в буфере.
    int buff_len; ///< Длина данных в буфере.
    bool debug; ///< Флаг для включения режима отладки.
    const char *target_ip; ///< Целевой IP-адрес.
    int rx_port; ///< Порт для приема данных.
    int tx_port; ///< Порт для передачи данных.
    int sock; ///< Дескриптор сокета.
    bool is_open; ///< Флаг, указывающий, открыт ли порт.

    /**
     * @brief Читает байт из UDP соединения.
     * 
     * @param cp Ссылка на переменную, в которую будет записан прочитанный байт.
     * @return int Результат операции чтения.
     */
    int _read_port(uint8_t &cp);

    /**
     * @brief Записывает данные в UDP соединение.
     * 
     * @param buf Буфер с данными для записи.
     * @param len Длина данных для записи.
     * @return int Количество записанных байт.
     */
    int _write_port(char *buf, unsigned len);
};

#endif // UDP_PORT_H_
