#ifndef GENERIC_PORT_H_
#define GENERIC_PORT_H_

#include <common/mavlink.h>

/**
 * @brief Абстрактный класс для представления общего интерфейса порта.
 * 
 * Этот класс определяет интерфейс для чтения и записи сообщений,
 * а также для управления состоянием порта.
 */
class Generic_Port
{
public:
    /**
     * @brief Конструктор по умолчанию.
     */
    Generic_Port() {};

    /**
     * @brief Виртуальный деструктор.
     */
    virtual ~Generic_Port() {};

    /**
     * @brief Читает сообщение.
     * 
     * @param message Ссылка на объект сообщения Mavlink, в который будет записано прочитанное сообщение.
     * @return int Код состояния операции чтения.
     */
    virtual int read_message(mavlink_message_t &message) = 0;

    /**
     * @brief Записывает сообщение.
     * 
     * @param message Константная ссылка на объект сообщения Mavlink, которое будет отправлено.
     * @return int Код состояния операции записи.
     */
    virtual int write_message(const mavlink_message_t &message) = 0;

    /**
     * @brief Проверяет, запущен ли порт.
     * 
     * @return true если порт запущен.
     * @return false если порт не запущен.
     */
    virtual bool is_running() = 0;

    /**
     * @brief Запускает порт.
     */
    virtual void start() = 0;

    /**
     * @brief Останавливает порт.
     */
    virtual void stop() = 0;
};

#endif // GENERIC_PORT_H_
