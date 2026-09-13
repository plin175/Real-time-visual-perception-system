// interaction.hpp

// 串口交互
#ifndef INTERACTION_H
#define INTERACTION_H

#pragma once

#include <vector>
#include <cstdint>
#include <cstring>
#include <atomic>

#pragma pack(1)

struct FrameHeader {
    uint8_t SOF = 0xA5;
    uint16_t data_length;
    uint8_t seq;
    uint8_t CRC8;
};

struct DataPacket {
    FrameHeader header;
    uint16_t cmdId;
    uint8_t* data;
    uint16_t CRC16;
};

using namespace std;

enum CMD_ID{ 
    GAME_STATUS = 0x0001,          // 比赛状态数据              服务器→全体机器人
    GAME_ROBOT_HP = 0x0003,        // 机器人血量数据            服务器→全体机器人
    ROBOT_STATUS = 0x0201,         // 机器人性能体系数据         主控模块→对应机器人    
    DETECT_PROCESS = 0x020C,       // 雷达标记进度数据           服务器→己方雷达机器人
    RADAR_INFO = 0x020E,           // 雷达自主决策信息同步       服务器→己方雷达机器人
    ROBOT_MAP = 0x0305,            // 选手端小地图接收雷达数据    雷达→服务器→己方所有选手端
    MAP_COMMAND = 0x0303,          // 选手端小地图交互数据       选手端点击→服务器→发送方选择的己方机器人
    INTERACTION_DATA = 0x0301,     // 机器人交互数据
    SEND_CUSTOM_INFO = 0x0308,     // 选手端小地图接收机器人数据  己方机器人→己方选手端
};

enum RADAR_ID{
    RADAR_RED = 9,                 // 雷达红方ID
    RADAR_BLUE = 109,              // 雷达蓝方ID
};

// 机器人间通信CMD
enum INTERACTION_CMD {
    RADAR_CMD = 0x0121,
    SENTRY_DATA = 0x0200,
    MAP_KEYBOARD = 0x0202,
    UWB_DATA = 0x0203,
};

// 蓝方 红方
constexpr uint8_t SENTRY_ID[] = {107, 7};                         // 哨兵  ID
constexpr uint8_t HERO_ID[] = {101, 1};                           // 英雄  ID
constexpr uint8_t ENGINEER_ID[] = {102, 2};                       // 工程  ID
constexpr uint8_t STANDARD_3_ID[] = {103, 3};                     // 步兵3 ID
constexpr uint8_t STANDARD_4_ID[] = {104, 4};                     // 步兵4 ID
constexpr uint8_t STANDARD_5_ID[] = {105, 5};                     // 步兵5 ID
 
constexpr uint8_t RED_ROBOT[] = {7, 1, 2, 3, 4, 5};               // 红色方所有机器人ID
constexpr uint8_t BLUE_ROBOT[] = {107, 101, 102, 103, 104, 105};  // 蓝色方所有机器人ID

constexpr uint16_t AERIAL_CLIENT[2] = {0x016A, 0x0106};           // 空中机器人选手端ID
constexpr uint16_t RADAR_ID[2] = {109, 9};                        // 雷达  ID

// 帧头定义格式
struct frame_header{
    uint8_t SOF = 0xA5;
    uint16_t data_length;
    uint8_t seq;
    uint8_t CRC8;
};

// 定义完整的数据包结构体
struct DataPacket {
    frame_header header;
    uint8_t* data;
    uint16_t cmdId;
    uint16_t CRC16;
};

// 帧头定义格式----send
struct send_frame_header{
    uint8_t SOF = 0xA5;
    uint16_t data_length;
    uint8_t seq;
    uint8_t CRC8;
};

// 定义完整的数据包结构体----send
struct send_DataPacket {
    send_frame_header header;
    uint8_t* data;
    uint16_t cmdId;
    uint16_t CRC16;
};

// 比赛类型、当前比赛阶段
struct game_status_t{
    uint8_t game_type : 4;    // 1：RoboMaster 机甲大师超级对抗赛; 2：RoboMaster 机甲大师高校单项赛; 3：ICRA RoboMaster 高校人工智能挑战赛; 4：RoboMaster 机甲大师高校联盟赛 3V3 对抗; 5：RoboMaster 机甲大师高校联盟赛步兵对抗
    uint8_t game_progress : 4;     // 0：未开始比赛; 1：准备阶段; 2：十五秒裁判系统自检阶段; 3：五秒倒计时; 4：比赛中; 5：比赛结算中
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
};

// 机器人血量
struct game_robot_HP_t{
    uint16_t red_1_robot_HP;   // 红 1 英雄
    uint16_t red_2_robot_HP;   // 红 2 工程
    uint16_t red_3_robot_HP;   // 红 3 步兵
    uint16_t red_4_robot_HP;   // 红 4 步兵
    uint16_t red_5_robot_HP;   // 红 5 步兵
    uint16_t red_7_robot_HP;   // 红 7 哨兵
    uint16_t red_outpost_HP;   // 红方前哨站
    uint16_t red_base_HP;      // 红方基地
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
};

// 机器人基础数值
struct robot_status_t{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
};

struct send_robot_position_t
{
    uint16_t robot_id;
    uint16_t robot_x;
    uint16_t robot_y;
};

// 地图上机器人坐标（红方补给站附近的交点为坐标原点，沿场地长边向蓝方为 X 轴正方向，沿场地短边向红方停机坪为 Y 轴正方向）
struct map_robot_position_t
{
    uint16_t hero_x;            // 英雄x坐标
    uint16_t hero_y;            // 英雄y坐标
    uint16_t engineer_x;        // 工程x坐标
    uint16_t engineer_y;        // 工程y坐标
    uint16_t standard_3_x;      // 3号步兵x坐标
    uint16_t standard_3_y;      // 3号步兵y坐标
    uint16_t standard_4_x;      // 4号步兵x坐标
    uint16_t standard_4_y;      // 4号步兵y坐标
    uint16_t standard_5_x;      // 5号步兵x坐标
    uint16_t standard_5_y;      // 5号步兵y坐标
    uint16_t sentry_x;          // 哨兵x坐标
    uint16_t sentry_y;          // 哨兵y坐标    
};

// 己方机器人坐标交互
struct ground_robot_position_t
{
    float hero_x;            // 英雄x坐标
    float hero_y;            // 英雄y坐标
    float engineer_x;        // 工程x坐标
    float engineer_y;        // 工程y坐标
    float standard_3_x;      // 3号步兵x坐标
    float standard_3_y;      // 3号步兵y坐标
    float standard_4_x;      // 4号步兵x坐标
    float standard_4_y;      // 4号步兵y坐标
    float standard_5_x;      // 5号步兵x坐标
    float standard_5_y;      // 5号步兵y坐标
};

// 选手端下发数据(小地图交互)
struct map_command_t{
    float target_position_x;
    float target_position_y;
    uint8_t cmd_keyboard;          // 云台手按下的键盘按键通用键值
    uint8_t target_robot_id;
    uint16_t cmd_source;           // 信息来源ID
};

// 选手端收数据(小地图交互)
struct map_robot_data_t{
    uint16_t hero_position_x;                  // 英雄x坐标
    uint16_t hero_position_y;                  // 英雄y坐标
    uint16_t engineer_position_x;              // 工程x坐标
    uint16_t engineer_position_y;              // 工程y坐标
    uint16_t infantry_3_position_x;            // 3号步兵x坐标
    uint16_t infantry_3_position_y;            // 3号步兵y坐标
    uint16_t infantry_4_position_x;            // 4号步兵x坐标
    uint16_t infantry_4_position_y;            // 4号步兵y坐标
    uint16_t infantry_5_position_x;            // 5号步兵x坐标
    uint16_t infantry_5_position_y;            // 5号步兵y坐标
    uint16_t sentry_position_x;                // 哨兵x坐标
    uint16_t sentry_position_y;                // 哨兵y坐标
};

// 给己方任意选手端发消息，显示在特定位置
struct custom_info_t{
    uint16_t sender_id;                       // 发送者
    uint16_t receiver_id;                     // 接收者
    uint8_t user_data[30];                    // 发送数据
};

// 被标记进度: 0-120
struct radar_mark_data_t
{
    uint8_t mark_hero_progress;         // 英雄
    uint8_t mark_engineer_progress;     // 工程
    uint8_t mark_standard_3_progress;   // 3号步兵
    uint8_t mark_standard_4_progress;   // 4号步兵
    uint8_t mark_standard_5_progress;   // 5号步兵
    uint8_t mark_sentry_progress;       // 哨兵
};

// 雷达是否触发双倍易伤
struct radar_info_t
{
    uint8_t radar_info;
};

// 收发数据
struct robot_interaction_data_t
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
};

// 雷达自主决策指令
struct radar_cmd_t
{
    uint8_t radar_cmd;
};

radar_info_t radar_info_RM;
radar_cmd_t radar_cmd_RM;
radar_mark_data_t radar_mark_data_RM;

#pragma pack()

#endif