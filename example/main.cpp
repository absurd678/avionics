#include <serial_port.h>
#include <udp_port.h>
#include <tcp_server.h>
// #include <mav_timesync/mav_timesync.h>
#include <common/mavlink.h>
#include "cxxopts.hpp"
#include "iostream"
#include <chrono>

int main(int argc, char **argv)
{
    cxxopts::Options options("mav_timesync", "mavlink time syncronisation");
    options.add_options()("d,device", "serial device", cxxopts::value<std::string>()->default_value("none"))(
        "b,baudrate", "serial baudrate", cxxopts::value<int>()->default_value("0"))(
        "a, address", "udp address", cxxopts::value<std::string>()->default_value("none"))(
        "p,port", "udp port", cxxopts::value<int>()->default_value("14550"))("t,tcp", "tcp_port", cxxopts::value<int>()->default_value("-1"))(
        "hz", "timesync hz", cxxopts::value<int>()->default_value("10"))(
        "h,help", "Print usage");
    auto result = options.parse(argc, argv);

    if (result.count("help"))
    {
        std::cout << options.help() << std::endl;
        exit(0);
    }

    std::string serial_device = result["device"].as<std::string>();
    int serial_baudrate = result["baudrate"].as<int>();
    std::string udp_address = result["address"].as<std::string>();
    int udp_port = result["port"].as<int>();
    int timesync_hz = result["hz"].as<int>();
    int tcp_port = result["tcp"].as<int>();

    Generic_Port *port;

    if (serial_device == "none" && udp_address == "none" && tcp_port == -1 || serial_device != "none" && udp_address != "none" && tcp_port != -1)
    {
        std::cout << options.help() << std::endl;
        exit(0);
    }
    else if (serial_device != "none")
    {
        port = new Serial_Port(serial_device.c_str(), serial_baudrate);
    }
    else if (udp_address != "none")
    {
        port = new UDP_Port(udp_address.c_str(), udp_port);
    }
    else if (tcp_port != -1)
    {
        port = new TCP_Server(tcp_port);
    }

    port->start();
    uint64_t timesync_last = 0;
    bool success;
    // mav_timesync timesync;
    while (true)
    {
        mavlink_message_t message;
        success = port->read_message(message);
        if (success)
        {
            uint64_t _now = std::chrono::duration_cast<std::chrono::microseconds>(
                                std::chrono::system_clock::now().time_since_epoch())
                                .count();
            switch (message.msgid)
            {
            case MAVLINK_MSG_ID_TIMESYNC:
            {
                mavlink_timesync_t timesync_msg;
                mavlink_msg_timesync_decode(&message, &timesync_msg);
                // if (timesync.f_time_sync(timesync_msg))
                // {

                //     mavlink_message_t msg;
                //     mavlink_msg_timesync_encode(255, MAV_COMP_ID_ONBOARD_COMPUTER, &msg,
                //                                 &timesync_msg);
                //     int len = port->write_message(msg);
                //     std::cout << "SEND_TIMESYNC: " << timesync_msg.tc1 << '\t'
                //               << timesync_msg.ts1 << std::endl;
                //     // std::cout << "dt: " << timesync_msg.tc1 - timesync_msg.ts1 << " ns"
                //     //   << std::endl;
                //     // check the write
                //     if (len <= 0)
                //         fprintf(stderr, "WARNING: could not send TIMESYNC \n");
                // }

                mavlink_system_time_t system_time;
                system_time.time_unix_usec =
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count();
                system_time.time_boot_ms = 0;
                mavlink_message_t msg1;
                mavlink_msg_system_time_encode(255, MAV_COMP_ID_ONBOARD_COMPUTER, &msg1,
                                               &system_time);
                std::cout << "SEND SYST_TIME: " << system_time.time_unix_usec
                          << std::endl;
                int len = port->write_message(msg1);
                std::time_t now = std::time(nullptr);
                std::cout << "Current time: " << std::asctime(std::localtime(&now))
                          << std::endl;
                if (len <= 0)
                    fprintf(stderr, "WARNING: could not send SYSTEM_TIME \n");

                break;
            }
            case MAVLINK_MSG_ID_SYSTEM_TIME:
            {
                mavlink_system_time_t system_time_msg;
                mavlink_msg_system_time_decode(&message, &system_time_msg);

                std::cout << "ARDUPILOT SYST_TIME: " << system_time_msg.time_unix_usec
                          << std::endl;

                ;
                break;
            }
            }

            if (_now - timesync_last > ((float)1. / timesync_hz) * 1e6)
            {

                mavlink_system_time_t system_time;
                system_time.time_unix_usec =
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count();
                system_time.time_boot_ms = 0;
                mavlink_message_t msg1;
                mavlink_msg_system_time_encode(255, MAV_COMP_ID_ONBOARD_COMPUTER, &msg1,
                                               &system_time);
                std::cout << "SEND SYST_TIME: " << system_time.time_unix_usec
                          <<'\t'<< _now - timesync_last <<'\t'<< ((float)1. / timesync_hz) * 1e6 << std::endl;
                int len = port->write_message(msg1);
                timesync_last = _now;
            }
        }
    }

    return 0;
}
