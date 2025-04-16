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
const float GO_FORWARD_METERS = 500;  
const float PRECISION = 0.1;  
const float PRECISION_XYZ = 10;
const int WAYPOINT_AMOUNT = 5;

mavlink_local_position_ned_t mission_square[WAYPOINT_AMOUNT];

// TODO: игнорит команды лететь в точку. Пишет, что она приходит, но не летит
void go_to(mavlink_message_t message, Generic_Port *port, mavlink_local_position_ned_t where){
    // Определяем параметры команды
    uint8_t target_system      = 1;                      // ID системы автопилота (обычно 1)
    uint8_t target_component   = MAV_COMP_ID_AUTOPILOT1;   // Компонент автопилота (обычно 1)
    uint16_t command           = MAV_CMD_DO_REPOSITION;     // Команда MAV_CMD_DO_REPOSITION 
    uint8_t frame              = MAV_FRAME_LOCAL_NED;      // Локальная система координат
    int16_t current            = 0;                      // Флаг «текущей» команды (0 – не текущая)
    uint8_t autocontinue       = 1;                      // Автопереход к следующей команде
    float   param1             = 5.0f;                   // Hold time (задержка) – 0 сек
    float   param2             = MAV_DO_REPOSITION_FLAGS_CHANGE_MODE;                   // Hold time (задержка) – 0 сек
    float   param3             = 5.0f;                   // Hold time (задержка) – 0 сек
    float   param4             = 0.0f;                   // 

    // При преобразовании координат: x и y умножаются на 100 для перехода из метров в сантиметры
    int32_t x = static_cast<int32_t>(where.x * 100);
    int32_t y = static_cast<int32_t>(where.y * 100);
    float   z = where.z; // Высота (z) передаётся в метрах
    
    /*
      Формируем MAVLink сообщение командой COMMAND_INT.
      Функция mavlink_msg_command_int_pack имеет следующий прототип:
        mavlink_msg_command_int_pack(uint8_t system_id, uint8_t component_id,
                                     mavlink_message_t* msg,
                                     uint8_t target_system, uint8_t target_component,
                                     uint16_t command, uint8_t frame,
                                     int16_t current, uint8_t autocontinue,
                                     float param1, int32_t x, int32_t y, float z);
    */
    mavlink_msg_command_int_pack(
        255,                          // system_id отправителя (например, наземная станция)
        MAV_COMP_ID_ONBOARD_COMPUTER,       // component_id отправителя (может быть изменён, если нужно)
        &message,
        0,
        0,
        command,
        frame,
        current,
        autocontinue,
        param1,
        param2,
        param3,
        param4,
        x,
        y,
        z
    );
    
    // Отправка сообщения через указанный порт
    int len = port->write_message(message);
    if (len <= 0) {
        std::cerr << "WARNING (F): could not send mavlink_msg_command_int_pack message" << std::endl;
    } else {
        std::cout << "Requested mavlink_msg_command_int_pack message" << std::endl;
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

bool waypoint_achieved(mavlink_local_position_ned_t expected_xyz, mavlink_local_position_ned_t actual_xyz){
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

void make_waypoints(mavlink_local_position_ned_t current, mavlink_local_position_ned_t mission[]){
    // точка 0
    mission[0].x = current.x + 1000;
    mission[0].y = current.y+ 1000;

    // точка 1
    mission[1].x = mission[0].x + GO_FORWARD_METERS;
    mission[1].y = mission[0].y;
    // точка 2
    mission[2].x = mission[0].x + GO_FORWARD_METERS;
    mission[2].y = mission[0].y + GO_FORWARD_METERS;
    // точка 3
    mission[3].x = mission[0].x;
    mission[3].y = mission[0].y + GO_FORWARD_METERS;
    // точка 4
    mission[3].x = mission[0].x;
    mission[3].y = mission[0].y;
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

    // flags for commands
    bool is_running_waypoint = false; // дрон на пути к точке
    int waypoint_number = -1;   // на какой точке сейчас
    bool is_first_coord = true;     // впервые ли получены координаты

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
            
            elapsed_seconds = time_now - last_req_mess_sent;
            if (elapsed_seconds.count() >= freq){ // check the mission state every <freq> seconds
                request_local_position_ned(port); // ask x y z
                last_req_mess_sent = std::chrono::system_clock::now();
            }
            

            if (message.msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED){
                mavlink_msg_local_position_ned_decode(&message, &actual_xyz);
                if(is_first_coord){
                    make_waypoints(actual_xyz, mission_square);
                    is_first_coord = false;

                    continue;
                }

                mavlink_message_t message_set_pos;
                if (waypoint_number>5 || waypoint_number<0) {
                    waypoint_number = 0;    
                }

                if(!is_running_waypoint){
                    go_to(message, port, mission_square[waypoint_number]);
                    is_running_waypoint = true;
                }
                if (waypoint_achieved(mission_square[waypoint_number], actual_xyz)){ 
                    waypoint_number++;
                    is_running_waypoint = false;
                }

            }  
        }   
    }

    return 0;
}
