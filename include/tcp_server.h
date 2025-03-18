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

/**
 * @brief Класс для работы с TCP сервером.
 * 
 * Этот класс предоставляет методы для чтения и записи сообщений через TCP соединение,
 * а также для управления состоянием сервера.
 */
class TCP_Server : public Generic_Port
{

public:
    /**
     * @brief Конструктор по умолчанию.
     */
    TCP_Server();

    /**
     * @brief Конструктор с указанием порта TCP.
     * 
     * @param tcp_port_ Порт TCP для сервера.
     */
    TCP_Server(int tcp_port_);

    /**
     * @brief Деструктор.
     */
    virtual ~TCP_Server();

    /**
     * @brief Читает сообщение из TCP соединения.
     * 
     * @param message Ссылка на объект сообщения Mavlink, в который будет записано прочитанное сообщение.
     * @return int Возвращает true, если сообщение было успешно прочитано.
     */
    int read_message(mavlink_message_t &message);

    /**
     * @brief Записывает сообщение в TCP соединение.
     * 
     * @param message Константная ссылка на объект сообщения Mavlink, которое будет отправлено.
     * @return int Количество байт, записанных в соединение.
     */
    int write_message(const mavlink_message_t &message);

    /**
     * @brief Проверяет, запущен ли сервер.
     * 
     * @return true если сервер запущен.
     * @return false если сервер не запущен.
     */
    bool is_running()
    {
        return is_open;
    }

    /**
     * @brief Запускает TCP сервер и ожидает подключения клиента.
     */
    void start();

    /**
     * @brief Останавливает TCP сервер и закрывает соединение.
     */
    void stop();
private:
    mavlink_status_t lastStatus; ///< Статус последнего сообщения Mavlink.
    pthread_mutex_t lock; ///< Мьютекс для синхронизации доступа к соединению.

    /**
     * @brief Инициализирует значения по умолчанию для атрибутов.
     */
    void initialize_defaults();

    const static int BUFF_LEN = 2041; ///< Длина буфера для чтения данных.
    char buff[BUFF_LEN]; ///< Буфер для чтения данных.
    int buff_ptr; ///< Указатель на текущую позицию в буфере.
    int buff_len; ///< Длина данных в буфере.
    bool debug; ///< Флаг для включения режима отладки.
    int port; ///< Порт TCP для сервера.
    int sockfd, connfd; ///< Дескрипторы сокетов для сервера и соединения.
    bool is_open; ///< Флаг, указывающий, открыт ли сервер.

    /**
     * @brief Читает байт из TCP соединения.
     * 
     * @param cp Ссылка на переменную, в которую будет записан прочитанный байт.
     * @return int Результат операции чтения.
     */
    int _read_port(uint8_t &cp);

    /**
     * @brief Записывает данные в TCP соединение.
     * 
     * @param buf Буфер с данными для записи.
     * @param len Длина данных для записи.
     * @return int Количество записанных байт.
     */
    int _write_port(char *buf, unsigned len);
};

#endif // TCP_Server_H_
