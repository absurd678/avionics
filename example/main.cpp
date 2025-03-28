#include <serial_port.h>
#include <udp_port.h>
#include <tcp_server.h>
// #include <mav_timesync/mav_timesync.h>
#include <common/mavlink.h>
#include "cxxopts.hpp"
#include "iostream"
#include <chrono>
#include <cmath>  // For M_PI and fmod


const float TURN_RIGHT_DEGREES = 1.5708;    // 90d
const float GO_FORWARD_METERS = 100;  
const float PRECISION = 0.1;  
const float PRECISION_XYZ = 5;


// Constrains an angle (in radians) to [-π, π]
inline double wrapToPi(double angle) {
    angle = fmod(angle + M_PI, 2.0 * M_PI);  // Shift to [0, 2π] range
    if (angle < 0)
        angle += 2.0 * M_PI;                 // Ensure positive
    return angle - M_PI;                     // Shift back to [-π, π]
}

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
        TURN_RIGHT_DEGREES,                                   // Yaw angle 90d
        0                                    // Yaw rate (not used)
    );
    int len = port->write_message(message);
    if (len <= 0) {
        std::cerr << "WARNING (R): could not send mavlink_msg_set_position_target_local_ned_pack" << std::endl;
    } else {
        std::cout << "Requested mavlink_msg_set_position_target_local_ned_pack message" << std::endl;
    }
}

void go_forward_100_m(mavlink_message_t message, Generic_Port *port){
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
        GO_FORWARD_METERS, 0, 0,                             // x, y, z position (not used)
        0, 0, 0,                             // vx, vy, vz velocity (1 m/s North)
        0, 0, 0,                             // afx, afy, afz acceleration (not used)
        0,                                   // Yaw angle (not used)
        0                                    // Yaw rate (not used)
    );
    int len = port->write_message(message);
    if (len <= 0) {
        std::cerr << "WARNING (F): could not send mavlink_msg_set_position_target_local_ned_pack" << std::endl;
    } else {
        std::cout << "Requested mavlink_msg_set_position_target_local_ned_pack message" << std::endl;
    }
}

void request_local_position_ned(Generic_Port *port){
    mavlink_command_long_t cmd = {};
    cmd.target_system = 1; // Target system ID
    cmd.target_component = 1; // Target component ID
    cmd.command = MAV_CMD_REQUEST_MESSAGE;
    cmd.param1 = MAVLINK_MSG_ID_LOCAL_POSITION_NED; // Request current position message
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
}

void request_attitude(Generic_Port *port){
    mavlink_command_long_t cmd = {};
    cmd.target_system = 1;                   // Target system ID
    cmd.target_component = 1;                // Target component ID
    cmd.command = MAV_CMD_REQUEST_MESSAGE;   // Command to request a message
    cmd.param1 = MAVLINK_MSG_ID_ATTITUDE;    // Request ATTITUDE message
    // Set unused params to 0
    cmd.param2 = cmd.param3 = cmd.param4 = cmd.param5 = cmd.param6 = cmd.param7 = 0;

    mavlink_message_t msg;
    mavlink_msg_command_long_encode(255, MAV_COMP_ID_ONBOARD_COMPUTER, &msg, &cmd);
    port->write_message(msg);  // Send the request
}

bool is_turned_right(mavlink_attitude_t expected_rpy, mavlink_attitude_t actual_rpy){
    if (abs(expected_rpy.yaw-actual_rpy.yaw)<PRECISION){
        return true;
    }
    return false;
}

bool is_went_forward(mavlink_local_position_ned_t expected_xyz, mavlink_local_position_ned_t actual_xyz){
    if (abs(expected_xyz.x-actual_xyz.x)<PRECISION_XYZ && abs(expected_xyz.y-actual_xyz.y)<PRECISION_XYZ){
        return true;
    }
    return false;
}

mavlink_local_position_ned_t modify_xyz(mavlink_local_position_ned_t actual_xyz,
    mavlink_attitude_t actual_rpy){

    mavlink_local_position_ned_t expected_xyz = actual_xyz;
    expected_xyz.x += GO_FORWARD_METERS*cos(actual_rpy.yaw);
    expected_xyz.y += GO_FORWARD_METERS*sin(actual_rpy.yaw);
    return expected_xyz;
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
    bool success;   // response result

    mavlink_local_position_ned_t expected_xyz;
    mavlink_local_position_ned_t actual_xyz;
    mavlink_attitude_t actual_rpy;
    mavlink_attitude_t expected_rpy;

    // flags for commands
    bool allow_right = true;
    bool is_turning_right = false;
    bool allow_forward = false;
    bool is_going_forward = false;

    // messages
    mavlink_mission_current_t mission_current;
    // time
    std::chrono::time_point<std::chrono::system_clock> last_req_mess_sent, time_now;
    int freq = 1;
    std::chrono::duration<double> elapsed_seconds;

    
    last_req_mess_sent = std::chrono::system_clock::now(); // not sent yet, wait 1 second
    while (true)
    {
        mavlink_message_t message;
        success = port->read_message(message);
        time_now = std::chrono::system_clock::now();
        
        
        if (success)
        {  
            
            //TODO: add mode guided setup, arming the throttle


            elapsed_seconds = time_now - last_req_mess_sent;
            if (elapsed_seconds.count() >= freq){ // check the mission state every <freq> seconds
                if (allow_right || is_turning_right) {
                    request_attitude(port); // ask roll pitch yaw
                }
                else if (allow_forward || is_going_forward){
                    request_local_position_ned(port); // ask x y z
                }
                last_req_mess_sent = std::chrono::system_clock::now();
            }
            

            if (message.msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED){
                mavlink_msg_local_position_ned_decode(&message, &actual_xyz);
                

                mavlink_message_t message_set_pos;
                if (allow_forward) {
                    go_forward_100_m(message_set_pos, port);
                    allow_forward = false;
                    is_going_forward = true;
                    expected_xyz = modify_xyz(actual_xyz, actual_rpy);
                }

                else if (is_going_forward && is_went_forward(expected_xyz, actual_xyz)) {
                    std::cout<<"COMPLETE FORWARD"<<std::endl;

                    allow_right = true;
                    is_going_forward = false;
                }

            } else if (message.msgid == MAVLINK_MSG_ID_ATTITUDE){
                mavlink_msg_attitude_decode(&message, &actual_rpy);
                mavlink_message_t message_set_pos;
                if (allow_right){
                    turn_right(message_set_pos, port);
                    allow_right = false;
                    is_turning_right = true;
                    expected_rpy = actual_rpy;
                    expected_rpy.yaw += TURN_RIGHT_DEGREES; 
                    expected_rpy.yaw = wrapToPi(expected_rpy.yaw);  // CONSTRAIN to [-pi, pi]
                }
                else if (is_turning_right && is_turned_right(expected_rpy, actual_rpy)) {
                    std::cout<<"COMPLETE RIGHT"<<std::endl;

                    allow_forward = true;
                    is_turning_right = false;
                }
            }        
        }   
    }

    return 0;
}
