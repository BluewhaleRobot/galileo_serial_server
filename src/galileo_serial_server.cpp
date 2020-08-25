#include "galileo_serial_server/galileo_serial_server.h"
#define DISABLE 0
#define ENABLE 1

namespace galileo_serial_server
{
StatusPublisher::StatusPublisher(std::string galileoCmds_topic, std::string galileoStatus_topic,
                                 CallbackAsyncSerial* cmd_serial)
  : galileoCmds_topic_(galileoCmds_topic), galileoStatus_topic_(galileoStatus_topic), cmd_serial_(cmd_serial)
{
    mbUpdated_ = false;
    mgalileoCmdsPub_ = mNH_.advertise<galileo_serial_server::GalileoNativeCmds>(galileoCmds_topic_, 1, true);

    car_status.nav_status = 0;// 导航服务状态，0表示没开启closed，1表示开启opened。
    car_status.visual_status = 0;// 视觉系统状态，-1标系视觉系统处于关闭状态，0表示没初始化uninit，1表示正在追踪tracking,2表示丢失lost,1和2都表示视觉系统已经初始化完成。
    car_status.map_status = 0; // 建图服务状态，0表示未开始建图，1表示正在建图
    car_status.gc_status = 0; // 内存回收标志，0表示未进行内存回收，1表示正在进行内存回收
    car_status.gba_status = 0; // 闭环优化标志，0表示未进行闭环优化，1表示正在进行闭环优化
    car_status.charge_status = 0; // 充电状态，0 free 未充电状态, 1 charging 充电中, 2 charged 已充满，但仍在小电流充电, 3 finding
                     // 寻找充电桩, 4 docking 停靠充电桩, 5 error 错误
    car_status.loop_status = 0; // 是否处于自动巡检状态，1为处于，0为不处于。
    car_status.power = 0.f;// 电源电压【946】v。
    car_status.target_numID = -1;// 当前目标点编号,默认值为-1表示无效值，当正在执行无ID的任务是值为-2，比如通过Http API 创建的导航任务。
    car_status.target_status = -1;// 当前目标点状态，0表示已经到达或者取消free，1表示正在前往目标点过程中working,2表示当前目标点的移动任务被暂停paused,3表示目标点出现错误error,默认值为-1表示无效值。
    car_status.target_distance = -1.f;// 机器人距离当前目标点的距离，单位为米，-1表示无效值，该值的绝对值小于0.01时表示已经到达。
    car_status.angle_goal_status = -1;// 目标角度达到情况，0表示未完成，1表示完成，2表示error,默认值为-1表示无效值。
    car_status.control_speed_x = 0.f;// 导航系统计算给出的前进速度控制分量,单位为m/s。
    car_status.control_speed_theta = 0.f;// 导航系统计算给出的角速度控制分量,单位为rad/s。
    car_status.current_speed_x = 0.f;// 当前机器人实际前进速度分量,单位为m/s。
    car_status.current_speed_theta = 0.f;// 当前机器人实际角速度分量,单位为rad/s。
    car_status.time_stamp = 0;// 时间戳,单位为1/30毫秒，用于统计丢包率。对于ROS API时间戳在状态的header里面
    car_status.current_pose_x = 0.f; // 当前机器人在map坐标系下的X坐标,此坐标可以直接用于设置动态插入点坐标
    car_status.current_pose_y = 0.f; // 当前机器人在map坐标系下的Y坐标
    car_status.current_angle = 0.f; // 当前机器人在map坐标系下的z轴转角(yaw)
    car_status.busy_status = 0; //当busy为true时系统将仍然后接收新指令，但是不会立即处理。当系统退出busy状态后再处理消息

    cmd_str_ = new char[89];
    cmd_str_[0] = (char)0xcd;
    cmd_str_[1] = (char)0xeb;
    cmd_str_[2] = (char)0xd7;
    cmd_str_[3] = (char)0x55;
    cmd_str_[88] = (char)0x00;
}

void StatusPublisher::Refresh()
{
    boost::mutex::scoped_lock lock(mStausMutex_);
    car_status.time_stamp += 1;
    int* receive_byte = (int*)&car_status;
    memcpy(&cmd_str_[4], &receive_byte[0], 84);
    if (NULL != cmd_serial_)
    {
        cmd_serial_->write(cmd_str_, 53);
    }
}

void StatusPublisher::UpdateCmds(const char* data, unsigned int len)
{
    boost::mutex::scoped_lock lock(mCmdsMutex_);
    int i = 0, j = 0;
    static unsigned char last_str[2] = { 0x00, 0x00 };
    static unsigned char new_packed_ctr = DISABLE;  // ENABLE表示新包开始，DISABLE 表示上一个包还未处理完；
    static int new_packed_ok_len = 0;               //包的理论长度
    static int new_packed_len = 0;                  //包的实际长度
    static unsigned char cmd_string_buf[512];
    unsigned char current_str = 0x00;
    const int cmd_string_max_size = 512;

    for (i = 0; i < len; i++)
    {
        current_str = data[i];
        // unsigned int temp=(unsigned int)current_str;
        // std::cout<<temp<<std::endl;
        //判断是否有新包头
        if (last_str[0] == 205 && last_str[1] == 235 && current_str == 215)  //包头 205 235 215
        {
            // std::cout<<"runup1 "<<std::endl;
            new_packed_ctr = ENABLE;
            new_packed_ok_len = 0;
            new_packed_len = new_packed_ok_len;
            last_str[0] = last_str[1];  //保存最后两个字符，用来确定包头
            last_str[1] = current_str;
            continue;
        }
        last_str[0] = last_str[1];  //保存最后两个字符，用来确定包头
        last_str[1] = current_str;
        if (new_packed_ctr == ENABLE)
        {
            //获取包长度
            new_packed_ok_len = current_str;
            if (new_packed_ok_len > cmd_string_max_size)
                new_packed_ok_len = cmd_string_max_size;  //包内容最大长度有限制
            new_packed_ctr = DISABLE;
            // std::cout<<"runup2 "<< new_packed_len<< new_packed_ok_len<<std::endl;
        }
        else
        {
            //判断包当前大小
            if (new_packed_ok_len <= new_packed_len || new_packed_ok_len > 20)
            {
                // std::cout<<"runup3 "<< new_packed_len<< new_packed_ok_len<<std::endl;
                //包长度已经大于等于理论长度，或者大于２０字节，后续内容无效
                continue;
            }
            else
            {
                //获取包内容
                new_packed_len++;
                cmd_string_buf[new_packed_len - 1] = current_str;
                if (new_packed_ok_len == new_packed_len && new_packed_ok_len > 0)
                {
                    // std::cout<<"runup4 "<<std::endl;
                    //当前包已经处理完成，开始处理
                    galileo_serial_server::GalileoNativeCmds currentCmds;
                    currentCmds.header.stamp = ros::Time::now();
                    currentCmds.header.frame_id = "galileo_serial_server";
                    currentCmds.length = new_packed_ok_len;
                    currentCmds.data.resize(new_packed_ok_len);

                    for (int i = 0; i < new_packed_ok_len; i++)
                    {
                        currentCmds.data[i] = cmd_string_buf[i];
                    }
                    mgalileoCmdsPub_.publish(currentCmds);
                    new_packed_ok_len = 0;
                    new_packed_len = 0;
                }
            }
        }
    }
    return;
}

void StatusPublisher::UpdateStatus(const galileo_serial_server::GalileoStatus& current_receive_status)
{
    boost::mutex::scoped_lock lock(mStausMutex_);

    car_status.nav_status = current_receive_status.navStatus;
    car_status.visual_status = current_receive_status.visualStatus;
    car_status.map_status = current_receive_status.mapStatus;
    car_status.gc_status = current_receive_status.gcStatus;
    car_status.gba_status = current_receive_status.gbaStatus;
    car_status.charge_status = current_receive_status.chargeStatus;
    car_status.loop_status = current_receive_status.loopStatus;
    car_status.power = current_receive_status.power;
    car_status.target_numID = current_receive_status.targetNumID;
    car_status.target_status = current_receive_status.targetStatus;
    car_status.target_distance = current_receive_status.targetDistance;
    car_status.angle_goal_status = current_receive_status.angleGoalStatus;
    car_status.control_speed_x = current_receive_status.controlSpeedX;
    car_status.control_speed_theta = current_receive_status.controlSpeedTheta;
    car_status.current_speed_x = current_receive_status.currentSpeedX;
    car_status.current_speed_theta = current_receive_status.currentSpeedTheta;
    car_status.current_pose_x = current_receive_status.currentPosX;
    car_status.current_pose_y = current_receive_status.currentPosY;
    car_status.current_angle = current_receive_status.currentAngle;
    car_status.busy_status = current_receive_status.busyStatus;
}

void StatusPublisher::run()
{
    ros::Subscriber sub = mNH_.subscribe(galileoStatus_topic_, 1, &StatusPublisher::UpdateStatus, this);
    ros::spin();
}

}  // namespace galileo_serial_server
