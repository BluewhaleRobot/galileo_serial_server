/******************************************************************************
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Bluewhale Robot
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Author: Xiefusheng, Randoms
 *******************************************************************************/

#ifndef __GALILEO_SERIAL_SERVER_H__
#define __GALILEO_SERIAL_SERVER_H__

#include "galileo_serial_server/AsyncSerial.h"

#include "ros/ros.h"
#include <boost/thread.hpp>
#include <boost/assign/list_of.hpp>
#include "galileo_serial_server/GalileoNativeCmds.h"
#include "galileo_serial_server/GalileoStatus.h"

namespace galileo_serial_server
{
typedef struct
{
  int nav_status;// 导航服务状态，0表示没开启closed，1表示开启opened。
  int visual_status;// 视觉系统状态，-1标系视觉系统处于关闭状态，0表示没初始化uninit，1表示正在追踪tracking,2表示丢失lost,1和2都表示视觉系统已经初始化完成。
  int map_status; // 建图服务状态，0表示未开始建图，1表示正在建图
  int gc_status; // 内存回收标志，0表示未进行内存回收，1表示正在进行内存回收
  int gba_status; // 闭环优化标志，0表示未进行闭环优化，1表示正在进行闭环优化
  int charge_status; // 充电状态，0 free 未充电状态, 1 charging 充电中, 2 charged 已充满，但仍在小电流充电, 3 finding
                     // 寻找充电桩, 4 docking 停靠充电桩, 5 error 错误
  int loop_status; // 是否处于自动巡检状态，1为处于，0为不处于。
  float power;// 电源电压【946】v。
  int target_numID;// 当前目标点编号,默认值为-1表示无效值，当正在执行无ID的任务是值为-2，比如通过Http API 创建的导航任务。
  int target_status;// 当前目标点状态，0表示已经到达或者取消free，1表示正在前往目标点过程中working,2表示当前目标点的移动任务被暂停paused,3表示目标点出现错误error,默认值为-1表示无效值。
  float target_distance;// 机器人距离当前目标点的距离，单位为米，-1表示无效值，该值的绝对值小于0.01时表示已经到达。
  int angle_goal_status;// 目标角度达到情况，0表示未完成，1表示完成，2表示error,默认值为-1表示无效值。
  float control_speed_x;// 导航系统计算给出的前进速度控制分量,单位为m/s。
  float control_speed_theta;// 导航系统计算给出的角速度控制分量,单位为rad/s。
  float current_speed_x;// 当前机器人实际前进速度分量,单位为m/s。
  float current_speed_theta;// 当前机器人实际角速度分量,单位为rad/s。
  unsigned int time_stamp;// 时间戳,单位为1/30毫秒，用于统计丢包率。对于ROS API时间戳在状态的header里面
  float current_pose_x; // 当前机器人在map坐标系下的X坐标,此坐标可以直接用于设置动态插入点坐标
  float current_pose_y; // 当前机器人在map坐标系下的Y坐标
  float current_angle; // 当前机器人在map坐标系下的z轴转角(yaw)
  int busy_status; //当busy为true时系统将仍然后接收新指令，但是不会立即处理。当系统退出busy状态后再处理消息
} DOWNLOAD_STATUS;

class StatusPublisher
{
  public:
    StatusPublisher(std::string galileoCmds_topic, std::string galileoStatus_topic, CallbackAsyncSerial* cmd_serial);
    void Refresh();
    void UpdateCmds(const char* data, unsigned int len);
    void UpdateStatus(const galileo_serial_server::GalileoStatus& current_receive_status);
    void run();
    DOWNLOAD_STATUS car_status;

  private:
    ros::NodeHandle mNH_;
    ros::Publisher mgalileoCmdsPub_;
    bool mbUpdated_;
    std::string galileoCmds_topic_;
    std::string galileoStatus_topic_;
    CallbackAsyncSerial* cmd_serial_;
    boost::mutex mStausMutex_;
    boost::mutex mCmdsMutex_;
    char* cmd_str_;
};

}  // namespace galileo_serial_server

#endif
