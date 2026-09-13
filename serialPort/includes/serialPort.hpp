// serialPort.hpp

// 串口通信
#ifndef SERIALPORT_H
#define SERIALPORT_H

#pragma once

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <serial/serial.h>
#include <std_msgs/msg/float32.hpp>

#include <queue>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <iostream>
#include <stdexcept>
#include <cstdint>

#include "/home/ubuntu/Desktop/serialPort/includes/interaction.hpp"
#include "/home/ubuntu/Desktop/serialPort/includes/crc.hpp"

#define JUDGE_GAME_STATE_DATA_LENGTH         20   //比赛状态信息
#define JUDGE_GAME_ROBOT_HP_LENGTH           41   //比赛机器人血量数据
#define JUDGE_ROBOT_STAUS_LENGTH             22   //机器人状态数据
#define JUDGE_GROUND_ROBOT_POSITION_LENGTH   45   //己方机器人坐标位置
#define JUDGE_MARK_LENGTH					 15   //雷达标记进度
#define JUDGE_LIDAR_DATA_LENGTH_AUTOCHOOSE	 10   //雷达自主决策信息同步
#define JUDGE_ROBOT_DATA_SENDOTHER_LENGTH	 136  //机器人交互数据
#define JUDGE_MAP_DATA_LENGTH			     24   //选手端小地图交互数据
#define JUDGE_LIDARMAP_DATA_LENGTH			 19   //选手端小地图接收雷达数据
#define JUDGE_SEND_CUSTOM_INFO_LENGTH        43   //选手端小地图接收机器人数据

using namespace std;

namespace serialPort{
    using SerialPair = std::pair<uint16_t, std::vector<uint8_t>>;

    class SerialDriver : public rclcpp::Node{
        public:
            explicit SerialDriver(const rclcpp::NodeOptions & options);
            ~SerialDriver() override;

            void process_external_input(uint16_t robot_id, float x, float y);

        private:
            // 基本定义
            serial::Serial serial_port_;
            std::string port_name_;
            int baud_rate_;

            uint8_t seq = 0;
            std::mutex write_lock;
            int fd;

            std::mutex port_mutex_;
            std::atomic<bool> running_{false};

            /*
            成员变量：
                发布：
                    雷达标记数据 pub_radar_mark_data
                    雷达信息 pub_radar_info
                    队伍颜色 pub_team_color
                    剩余时间 pub_remain_time
                    机器人HP pub_robot_hp
                    地图命令 pub_map_keyboard
                    UWB数据 pub_uwb_data
                后台读取串行数据 read_thread
                订阅：
                    雷达命令 sub_radar_cmd
                    比赛结果 sub_game_result
                    机器人地图数据 sub_robot_position_data
                队伍颜色 team_color
            */
            uint16_t last_robot_id_;
            float last_x_;
            float last_y_;

            std::thread read_thread;

            void init_port();                                    // 初始化串口
            void open_port();                                    // 打开串口
            void close_port();                                   // 关闭串口
            void reopen_port();                                  // 重新打开串口       
            
            void receiveData();                                  // 接收数据
            void handle_receiveData(uint8_t* buffer, size_t buffer_size);         // 处理接收数据
            int handle_cmdId(CMD_ID cmdId, uint8_t* data, uint16_t len);

            void handle_send_data(CMD_ID cmdId, uint8_t* data, uint16_t data_length);
            void send_robot_position_data(uint16_t id, float x, float y);        // 发送机器人数据
            void send_sentry_data(uint16_t id, float x, float y);                             // 发送给哨兵的数据
            void send_radar_cmd(const radar_mark_data_t& mark_data);                               // 发送雷达命令

            
            // // 串口成员变量
            // serial::Serial serial_port_;
            // std::string port_name_;
            // int baud_rate_;

            // // 在参数客户端设置detect_colr
            // rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;  // 异步
            // rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;
            
            // // 在服务客户端重设跟踪器
            // rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;

            // // Aimimg point receiving from serial port for visualization
            // visualization_msgs::msg::Marker aiming_point_;

            // // Broadcast tf from odom to gimbal_link
            // double timestamp_offset_ = 0;
            // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
            // rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;

            // // For debug usage
            // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;
            // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

            // std::thread receive_thread_;
    };
}  // namespace serialPort

#pragma pack()

#endif // SERIALPORT_H