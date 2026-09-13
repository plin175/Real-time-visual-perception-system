// serialPort.cpp

#include "/home/ubuntu/Desktop/serialPort/includes/serialPort.hpp"
#include "/home/ubuntu/Desktop/serialPort/includes/crc.hpp"

#include <iterator>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <serial/serial.h>
#include <libserial/SerialStream.h>

#include <cstddef>
#include <cstdint>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>

using namespace serialPort;
using namespace std;

extern std::vector<DataPacket> receivedPackets;


namespace serialPort{

// 构造函数中的初始化代码
SerialDriver::SerialDriver(const rclcpp::NodeOptions & options)
    : Node("serial_driver", options) {
        try{
            this->declare_parameter("port_name", "/dev/ttyUSB0");
            this->declare_parameter("baud_rate", 115200);
            this->get_parameter("port_name", port_name_);
            this->get_parameter("baud_rate", baud_rate_);

            init_port();
            open_port();

            read_thread = std::thread(&SerialDriver::receiveData, this);
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "节点初始化失败: %s", e.what());
            throw;
        }
    }

// 析构函数中停止线程
SerialDriver::~SerialDriver() {
    running_ = false;
    if (read_thread.joinable()) {
        read_thread.join();
    }
    close_port();
}

void SerialDriver:: process_external_input(uint16_t robot_id, float x, float y){
    last_robot_id_ = robot_id;
    last_x_ = x;
    last_y_ = y;

    send_robot_position_data(robot_id, x, y);
}

/*
串口基本操作：串口参数、打开、关闭、重开
*/
void SerialDriver::init_port(){
    try{
        serial_port_.setPort(port_name_);
        serial_port_.setBaudrate(baud_rate_);
        // 串口超时
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serial_port_.setTimeout(timeout);
    }
    catch (const serial::IOException& e) {
        RCLCPP_ERROR(this->get_logger(), "设置串口参数时发生IOException: %s", e.what());
        throw;   // 重新抛出异常
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "设置串口参数时发生异常: %s", e.what());
        throw;
    }
}

void SerialDriver::open_port() {
    const int max_retries = 5;
    int retries = 0;
    while (rclcpp::ok() && retries < max_retries) {
        try {
            if (!serial_port_.isOpen()) {
                serial_port_.open();
                if (serial_port_.isOpen()) {
                    RCLCPP_INFO(get_logger(), "成功打开串口");
                    return;
                }
            }
        } catch (const serial::IOException& e) {
            RCLCPP_WARN(get_logger(), "打开串口失败，重试中... (%d/%d)", retries+1, max_retries);
            retries++;
            std::this_thread::sleep_for(1s);
        }
    }
    RCLCPP_ERROR(get_logger(), "无法打开串口");
}

void SerialDriver::close_port(){
    if(serial_port_.isOpen()){
        serial_port_.close();
        RCLCPP_INFO(this->get_logger(), "串口已关闭");
    }
}  

void SerialDriver::reopen_port(){
    close_port();
    init_port();
    open_port();
}
    



/*
从裁判系统接收数据：接收、处理
*/   
// 接受数据
void SerialDriver::receiveData() {
    running_ = true;
    // while (rclcpp::ok()) {
    while (running_) {
        if (!serial_port_.isOpen()) {
            RCLCPP_ERROR(get_logger(), "串口未打开");
            reopen_port();
            std::this_thread::sleep_for(100ms);
            continue;
        }

        std::vector<uint8_t> buffer;
        size_t bytes_available = serial_port_.available();
        if (bytes_available == 0) {
            continue;
        }

        buffer.resize(bytes_available);
        size_t bytes_read = serial_port_.read(buffer.data(), bytes_available);
        if (bytes_read != bytes_available) {
            RCLCPP_ERROR(get_logger(), "读取数据不完整");
            continue;
        }

        size_t start = 0;
        while (start < bytes_read) {
            if (buffer[start] != 0xA5) {
                start++;
                continue;
            }

            if (bytes_read - start < sizeof(frame_header)) {
                break; // 数据不足，等待下次接收
            }

            frame_header header;
            memcpy(&header, buffer.data() + start, sizeof(frame_header));
            if (!Verify_CRC8_Check_Sum(reinterpret_cast<uint8_t*>(&header), 5)) {
                RCLCPP_ERROR(get_logger(), "帧头CRC校验失败");
                start++;
                continue;
            }

            uint16_t data_length = header.data_length;
            if (bytes_read - start < sizeof(frame_header) + data_length + 2) { // +2 for CRC16
                break; // 数据不足
            }

            uint16_t cmdId;
            memcpy(&cmdId, buffer.data() + start + sizeof(frame_header), sizeof(cmdId));
            uint8_t* data_start = buffer.data() + start + sizeof(frame_header) + sizeof(cmdId);
            uint16_t received_crc16;
            memcpy(&received_crc16, data_start + data_length, sizeof(received_crc16));

            // 校验数据部分CRC16
            if (!Verify_CRC16_Check_Sum(buffer.data() + start + sizeof(frame_header), data_length + sizeof(cmdId) + sizeof(received_crc16))) {
                RCLCPP_ERROR(get_logger(), "数据CRC校验失败");
                start += sizeof(frame_header) + data_length + sizeof(received_crc16);
                continue;
            }

            handle_receiveData(data_start, data_length);
            start += sizeof(frame_header) + data_length + sizeof(received_crc16);
        }
    }
}

void SerialDriver::handle_receiveData(uint8_t* buffer, size_t buffer_size) {
    if (buffer_size < 7) {
        RCLCPP_ERROR(this->get_logger(), "接收的不正确");
        return;
    }

    // 解析帧头
    frame_header header;
    header.SOF = buffer[0];
    header.data_length = buffer[1] | (buffer[2] << 8);
    header.seq = buffer[3];
    header.CRC8 = buffer[4];

    // SOF
    if (header.SOF != 0xA5) {
        RCLCPP_ERROR(this->get_logger(), "SOF无效");
        return;
    }

    // Check if the data length matches the actual received data
    if (buffer_size != 7 + header.data_length + 2) { // 2 bytes的RC16
        RCLCPP_ERROR(this->get_logger(), "数据长度获取不正确");
        return;
    }

    uint16_t robot_id = buffer[5] | (buffer[6] << 8);
    uint16_t x = buffer[7] | (buffer[8] << 8);
    uint16_t y = buffer[9] | (buffer[10] << 8);
    uint16_t receivedCRC16 = buffer[11] | (buffer[12] << 8);

    uint16_t calculatedCRC16 = Get_CRC16_Check_Sum(buffer + 5, header.data_length + 2, CRC_INIT);
    if (receivedCRC16 != calculatedCRC16) {
        RCLCPP_ERROR(this->get_logger(), "CRC16检查失败");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "得到的机器人坐标: ID=%d, x=%d, y=%d", robot_id, x, y);

    handle_cmdId(static_cast<CMD_ID>(robot_id), data, header.data_length);
}







// void SerialDriver::handle_receiveData(uint8_t* buffer, size_t buffer_size){
//     if (buffer_size < 7) {
//         RCLCPP_ERROR(this->get_logger(), "接收的不正确");
//         return;
//     }

//     // 解析帧头
//     frame_header header;
//     header.SOF = buffer[0];
//     header.data_length = buffer[1] | (buffer[2] << 8);
//     header.seq = buffer[3];
//     header.CRC8 = buffer[4];

//     // SOF
//     if (header.SOF != 0xA5) {
//         RCLCPP_ERROR(this->get_logger(), "SOF无效");
//         return;
//     }

//     // 检查数据长度是否与实际接收到的数据长度匹配
//     if (buffer_size != 7 + header.data_length + 2) { // 2 bytes for CRC16
//         RCLCPP_ERROR(this->get_logger(), "数据长度获取不正确");
//         return;
//     }

//     uint16_t cmdId = buffer[5] | (buffer[6] << 8);
//     uint8_t* data = buffer + 7; // 跳过帧头和命令ID
//     // size_t data_size = header.data_length;
//     uint16_t receivedCRC16 = buffer[7 + header.data_length] | (buffer[8 + header.data_length] << 8);

//     uint16_t calculatedCRC16 = Get_CRC16_Check_Sum(buffer + 5, header.data_length + 2, CRC_INIT);
//     if (receivedCRC16 != calculatedCRC16) {
//         RCLCPP_ERROR(this->get_logger(), "CRC16检查失败");
//         return;
//     }

//     handle_cmdId(static_cast<CMD_ID>(cmdId), data, header.data_length);


// }

int SerialDriver::handle_cmdId(CMD_ID cmdId, uint8_t* data, uint16_t len){
    switch (cmdId) {
        // 比赛机器人状态
        case CMD_ID::GAME_STATUS:{ 
			if(len == (JUDGE_GAME_STATE_DATA_LENGTH -9)){
				// game_status_t* GameStatus = reinterpret_cast<game_status_t*>(data);
                return JUDGE_GAME_STATE_DATA_LENGTH;
			}
		}break;
        // 机器人血量数据
		case CMD_ID::GAME_ROBOT_HP:{
			if(len == (JUDGE_GAME_ROBOT_HP_LENGTH -9)){
				// game_robot_HP_t* GameRobotHP = reinterpret_cast<game_robot_HP_t*>(data);
                return JUDGE_GAME_ROBOT_HP_LENGTH;
			}
		}break;
        // 比赛机器人状态
		case CMD_ID::ROBOT_STATUS:{
			if(len == (JUDGE_ROBOT_STAUS_LENGTH -9)){
				// robot_status_t* RobotStatus = reinterpret_cast<robot_status_t*>(data);
                return JUDGE_ROBOT_STAUS_LENGTH;
			}
		}break;
        // 敌方机器人被雷达标记进度
		case CMD_ID::DETECT_PROCESS:{
			if(len == (JUDGE_MARK_LENGTH -9)){
				// radar_mark_data_t* RadarMarkData = reinterpret_cast<radar_mark_data_t*>(data);
                return JUDGE_MARK_LENGTH;
			}
		}break;
        // 雷达自主决策信息同步
		case CMD_ID::RADAR_INFO:
		{
			if(len == (JUDGE_LIDAR_DATA_LENGTH_AUTOCHOOSE -9))
			{
				// radar_info_t* RadarInfo = reinterpret_cast<radar_info_t*>(data);
                return JUDGE_LIDAR_DATA_LENGTH_AUTOCHOOSE;
			}
		}break;
        // 机器人交互数据
		case CMD_ID::INTERACTION_DATA:{
			if(len == (JUDGE_ROBOT_DATA_SENDOTHER_LENGTH -9)){
				// robot_interaction_data_t* RobotInteractionData = reinterpret_cast<robot_interaction_data_t*>(data);
                return JUDGE_ROBOT_DATA_SENDOTHER_LENGTH;
			}
		}break;
        // 选手端小地图交互数据
		case CMD_ID::MAP_COMMAND:{
			if(len == (JUDGE_MAP_DATA_LENGTH -9)){
				// map_command_t* MapCommand = reinterpret_cast<map_command_t*>(data);
                return JUDGE_MAP_DATA_LENGTH;
			}
		}break;
        // 选手端小地图接收雷达数据
		case CMD_ID::ROBOT_MAP:{
			if(len == (JUDGE_LIDARMAP_DATA_LENGTH -9)){
				// map_robot_data_t* MapRobotData = reinterpret_cast<map_robot_data_t*>(data);
                return JUDGE_LIDARMAP_DATA_LENGTH;
			}
		}break;
        case CMD_ID::SEND_CUSTOM_INFO:{
            if(len == (JUDGE_SEND_CUSTOM_INFO_LENGTH -9)){
                return JUDGE_SEND_CUSTOM_INFO_LENGTH;
            }
        }break;
        default:
            RCLCPP_ERROR(this->get_logger(), "未知的命令码: %d", cmdId);
            return 0;
    }
}

/*
发送命令：整合发送、分别发送
*/
// 处理要发送的数据
void SerialDriver::handle_send_data(CMD_ID cmdId, uint8_t* data, uint16_t data_length) {
    if (data_length > 112) {
        RCLCPP_INFO(this->get_logger(), "Data too long: %d", data_length);
        return;
    }

    send_DataPacket packet;

    // Construct frame header
    packet.header.SOF = 0xA5;
    packet.header.data_length = data_length;
    packet.header.seq = seq++;
    Append_CRC8_Check_Sum(reinterpret_cast<uint8_t*>(&packet.header), sizeof(send_frame_header));

    packet.cmdId = static_cast<uint16_t>(cmdId);

    // Copy data
    packet.data = new uint8_t[data_length];
    memcpy(packet.data, data, data_length);

    // Construct complete packet
    std::vector<uint8_t> full_pack(sizeof(packet.header) + data_length + sizeof(packet.CRC16));
    memcpy(full_pack.data(), &packet.header, sizeof(packet.header));
    memcpy(full_pack.data() + sizeof(packet.header), packet.data, data_length);

    // Append CRC16 checksum
    Append_CRC16_Check_Sum(full_pack.data(), full_pack.size());

    // Lock mutex before writing
    std::lock_guard<std::mutex> lock(write_lock);

    // Write to serial port
    // write(static_cast<CMD_ID>(cmdId), full_pack.data(), full_pack.size());

    delete[] packet.data;
}

// 发送机器人坐标给裁判系统
void SerialDriver::send_robot_position_data(uint16_t id, float x, float y) {
    // 转换坐标
    uint16_t x_converted = static_cast<uint16_t>(x * 100);
    uint16_t y_converted = static_cast<uint16_t>(y * 100);

    send_robot_position_t position_data = {id, x_converted, y_converted};

    handle_send_data(static_cast<CMD_ID>(id), reinterpret_cast<uint8_t*>(&position_data), sizeof(position_data));
    // write(static_cast<CMD_ID>(id), reinterpret_cast<uint8_t*>(&position_data), sizeof(position_data));
    // 发送数据
    std::lock_guard<std::mutex> lock(write_lock);
    try {
        serial_port_.write(full_packet);
    } catch (const serial::IOException& e) {
        RCLCPP_ERROR(get_logger(), "串口写入失败: %s", e.what());
    }
}

// 发送给哨兵的数据
void SerialDriver::send_sentry_data(uint16_t id, float x, float y) {
    uint16_t x_converted = static_cast<uint16_t>(x);
    uint16_t y_converted = static_cast<uint16_t>(y);

    send_robot_position_t position_data = {id, x_converted, y_converted};

    handle_send_data(static_cast<CMD_ID>(id), reinterpret_cast<uint8_t*>(&position_data), sizeof(position_data));
    write(static_cast<CMD_ID>(id), reinterpret_cast<uint8_t*>(&position_data), sizeof(position_data));
}          

// 发送雷达命令
void SerialDriver::send_radar_cmd(const radar_mark_data_t& mark_data) {
    bool triggered = false;
    CMD_ID cmdId = RADAR_INFO;
    for(size_t i=0; i<sizeof(radar_mark_data_RM); ++i){
        uint8_t progress = *reinterpret_cast<const uint8_t*>(&mark_data) + i;
        if(progress >= 100){
            RCLCPP_INFO(this->get_logger(), "触发易伤");
            triggered = true;
            // 发送易伤命令
            break;
        }
    }

    if (triggered && (radar_info_RM.radar_info & 0x03) && !(radar_info_RM.radar_info & 0x04)){
        radar_cmd_RM.radar_cmd = 1;
        radar_info_RM.radar_info |= 0x04;
        RCLCPP_INFO(this->get_logger(), "触发双倍易伤");
    }
    handle_send_data(cmdId, reinterpret_cast<uint8_t*>(&radar_info_RM), sizeof(radar_info_RM));
    write(cmdId, reinterpret_cast<uint8_t*>(&radar_info_RM), sizeof(radar_info_RM));
}
}




int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<serialPort::SerialDriver>(rclcpp::NodeOptions());
    
    // 从命令行获取输入
    std::cout << "Enter robot ID (1-6): ";
    uint16_t robot_id;
    std::cin >> robot_id;
    std::cout << "Enter x coordinate: ";
    float x;
    std::cin >> x;
    std::cout << "Enter y coordinate: ";
    float y;
    std::cin >> y;

    // 处理外部输入
    node->process_external_input(robot_id, x, y);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}