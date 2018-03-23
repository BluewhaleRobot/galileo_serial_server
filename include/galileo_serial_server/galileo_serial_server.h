#ifndef galileoSERIALSERVER_H
#define galileoSERIALSERVER_H

#include "galileo_serial_server/AsyncSerial.h"

#include "ros/ros.h"
#include <boost/thread.hpp>
#include <boost/assign/list_of.hpp>
#include "galileo_serial_server/GalileoNativeCmds.h"
#include "galileo_serial_server/GalileoStatus.h"

namespace galileo_serial_server
{
typedef struct {
  int nav_status;//导航服务状态,0 表示没开启 closed,1 表示开启 opened 。
  int visual_status;//视觉系统状态,0 表示没初始化 uninit,1 表示正在追踪 tracking,2表示丢失 lost,1 和 2 都表示视觉系统已经初始化完成。
  float power;//电源电压【9 46】v 。
  int target_numID;//当前目标点编号,默认值为-1 表示无效值。
  int target_status;//当前目标点状态,0 表示已经到达或者取消 free,1 表示正在前往目标点过程中 working,2 表示当前目标点的移动任务被暂停 paused,3 表示目标点出现错误 error, 默认值为-1 表示无效值。
  float target_distance;//机器人距离当前目标点的距离,单位为米,-1 表示无效值,该值的绝对值小于 0.01 时表示已经到达。
  int angle_goal_status;//目标角度达到情况,0 表示未完成,1 表示完成,2 表示 error,默认值为-1 表示无效值。
  float control_speed_x;//导航系统计算给出的前进速度控制分量,单位为 m/s 。
  float control_speed_theta;//导航系统计算给出的角速度控制分量,单位为 rad/s 。
  float current_speed_x;//当前机器人实际前进速度分量,单位为 m/s 。
  float current_speed_theta;//当前机器人实际角速度分量,单位为 rad/s 。
  unsigned int time_stamp;//时间戳,单位为 1/30 毫秒,用于统计丢包率。
}DOWNLOAD_STATUS;

class StatusPublisher
{
public:
    StatusPublisher(std::string galileoCmds_topic,std::string galileoStatus_topic,CallbackAsyncSerial* cmd_serial);
    void Refresh();
    void UpdateCmds(const char *data, unsigned int len);
    void UpdateStatus(const galileo_serial_server::GalileoStatus & current_receive_status);
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
    char * cmd_str_;
};

} //galileo_serial_server


#endif // galileoSERIALSERVER_H
