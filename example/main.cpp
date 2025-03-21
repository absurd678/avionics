#include <serial_port.h>
#include <udp_port.h>
#include <tcp_server.h>
// #include <mav_timesync/mav_timesync.h>
#include <common/mavlink.h>
#include "cxxopts.hpp"
#include "iostream"
#include <chrono>


void turn_right(mavlink_message_t message, Generic_Port *port){
    // turn right
    mavlink_msg_set_position_target_local_ned_pack(
        255,                                // Sender system ID
        MAV_COMP_ID_ONBOARD_COMPUTER,        // Sender component ID
        &message,                                // MAVLink message to pack into
        0,                                   // Timestamp (not used)
        0,                                   // Target system ID
        0,                                   // Target component ID
        MAV_FRAME_BODY_OFFSET_NED,                 // Coordinate frame
        2503,                                   // Type mask (velocity control)
        0, 0, 0,                             // x, y, z position (not used)
        0, 0, 0,                             // vx, vy, vz velocity (1 m/s North)
        0, 0, 0,                             // afx, afy, afz acceleration (not used)
        1.5708,                                   // Yaw angle 45d
        0                                    // Yaw rate (not used)
    );
    int len = port->write_message(message_set_pos);
    if (len <= 0) {
        std::cerr << "WARNING (R): could not send mavlink_msg_set_position_target_local_ned_pack" << std::endl;
    } else {
        std::cout << "Requested mavlink_msg_set_position_target_local_ned_pack message" << std::endl;
    }
}

void go_forward_10_m(mavlink_message_t message, Generic_Port *port){
    // go forward
    mavlink_msg_set_position_target_local_ned_pack(
        255,                                // Sender system ID
        MAV_COMP_ID_ONBOARD_COMPUTER,        // Sender component ID
        &message,                                // MAVLink message to pack into
        0,                                   // Timestamp (not used)
        0,                                   // Target system ID
        0,                                   // Target component ID
        MAV_FRAME_BODY_OFFSET_NED,                 // Coordinate frame
        3576,                                   // Type mask (velocity control)
        100, 0, 0,                             // x, y, z position (not used)
        0, 0, 0,                             // vx, vy, vz velocity (1 m/s North)
        0, 0, 0,                             // afx, afy, afz acceleration (not used)
        0,                                   // Yaw angle (not used)
        0                                    // Yaw rate (not used)
    );
    int len = port->write_message(message_set_pos);
    if (len <= 0) {
        std::cerr << "WARNING (F): could not send mavlink_msg_set_position_target_local_ned_pack" << std::endl;
    } else {
        std::cout << "Requested mavlink_msg_set_position_target_local_ned_pack message" << std::endl;
    }
}

int main(int argc, char **argv)
{
    cxxopts::Options options("mav_timesync", "mavlink time syncronisation");
    options.add_options()("d,device", "serial device", cxxopts::value<std::string>()->default_value("none"))(
        "b,baudrate", "serial baudrate", cxxopts::value<int>()->default_value("0"))(
        "a, address", "udp address", cxxopts::value<std::string>()->default_value("none"))(
        "p,port", "udp port", cxxopts::value<int>()->default_value("14550"))("t,tcp", "tcp_port", cxxopts::value<int>()->default_value("8800"))(
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
    bool success;   // response result

    bool allow_right = 1;  // send command
    bool is_turning_right = 0;  // info flag
    bool is_going_forward = 0;  // info flag
    bool allow_forward = 0;  // send command

    bool need_mission_state = true;
    mavlink_mission_current_t mission_current;
    // mav_timesync timesync;


    

    while (true)
    {
        mavlink_message_t message;
        success = port->read_message(message);
        
        
        //MAV_CMD_REQUEST_MESSAGE
        if (success)
        {  

            if (need_mission_state){

                mavlink_command_long_t cmd = {};
                cmd.target_system = 1; // Target system ID
                cmd.target_component = 1; // Target component ID
                cmd.command = MAV_CMD_REQUEST_MESSAGE;
                cmd.param1 = MAVLINK_MSG_ID_MISSION_CURRENT; // Request MISSION_CURRENT message
                cmd.param2 = 0; // Unused
                cmd.param3 = 0; // Unused
                cmd.param4 = 0; // Unused
                cmd.param5 = 0; // Unused
                cmd.param6 = 0; // Unused
                cmd.param7 = 0; // Unused
            
                mavlink_message_t msg;
                mavlink_msg_command_long_encode(255, MAV_COMP_ID_ONBOARD_COMPUTER, &msg, &cmd);
            
                int len = port->write_message(msg);
                if (len <= 0) {
                    std::cerr << "WARNING: could not send MAV_CMD_REQUEST_MESSAGE" << std::endl;
                } else {
                    std::cout << "Requested MISSION_CURRENT message" << std::endl;
                }
            
                need_mission_state = false;
            }
            else {
                mavlink_mission_current_t mission_current;
                mavlink_msg_mission_current_decode(&message, &mission_current);
                need_mission_state = true;
                
                switch (mission_current.mission_state)
                {
                    case MISSION_STATE_NO_MISSION:
                        mavlink_message_t message_set_pos;
                        if (allow_right){
                            turn_right(message_set_pos);
                            allow_right = false;
                            is_turning_right = true;
                            
                        }
                        else if (allow_forward) {
                            go_forward_10_m(message_set_pos);
                            allow_forward = false;
                            is_going_forward = true;
                            
                        }
                        break;
                    
                    case MISSION_STATE_COMPLETE:
                        
                        if (is_turning_right) {
                            std::cout<<"COMPLETE RIGHT"<<std::endl;
                        
                            allow_forward = true;
                            is_turning_right = false;
                        }
                        else if (is_going_forward) {
                            std::cout<<"COMPLETE FORWARD"<<std::endl;
                        
                            allow_right = true;
                            is_going_forward = false;
                        }

                        break;
                    
                }
            }
        }
    }

    return 0;
}
