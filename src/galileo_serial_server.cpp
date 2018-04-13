#include "galileo_serial_server/AsyncSerial.h"
#include "galileo_serial_server/galileo_serial_server.h"
#define DISABLE 0
#define ENABLE 1

namespace galileo_serial_server
{
StatusPublisher::StatusPublisher(std::string galileoCmds_topic,std::string galileoStatus_topic,CallbackAsyncSerial* cmd_serial)
                                :galileoCmds_topic_(galileoCmds_topic),galileoStatus_topic_(galileoStatus_topic),cmd_serial_(cmd_serial)
{
  mbUpdated_=false;
  mgalileoCmdsPub_ = mNH_.advertise<galileo_serial_server::GalileoNativeCmds>(galileoCmds_topic_,1,true);
  car_status.nav_status=0;
  car_status.visual_status=0;
  car_status.power=0.0f;
  car_status.target_numID=-1;
  car_status.target_status=-1;
  car_status.target_distance=-1.0f;
  car_status.angle_goal_status=-1;
  car_status.control_speed_x=0.0f;
  car_status.control_speed_theta=0.0f;
  car_status.current_speed_x=0.0f;
  car_status.current_speed_theta=0.0f;
  car_status.time_stamp=0;
  cmd_str_ = new char[53];
  cmd_str_[0] = (char)0xcd;
  cmd_str_[1] = (char)0xeb;
  cmd_str_[2] = (char)0xd7;
  cmd_str_[3] = (char)0x31;
  cmd_str_[52] = (char)0x00;
}

void StatusPublisher::Refresh()
{
  boost::mutex::scoped_lock lock(mStausMutex_);
  car_status.time_stamp +=1;
  int * receive_byte=(int *)&car_status;
  memcpy(&cmd_str_[4],&receive_byte[0],48);
  if(NULL!=cmd_serial_)
  {
      cmd_serial_->write(cmd_str_,53);
  }
}

void StatusPublisher::UpdateCmds(const char *data, unsigned int len)
{
    boost::mutex::scoped_lock lock(mCmdsMutex_);
    int i=0,j=0;
    static unsigned char last_str[2]={0x00,0x00};
    static unsigned char new_packed_ctr=DISABLE;//ENABLE表示新包开始，DISABLE 表示上一个包还未处理完；
    static int new_packed_ok_len=0;//包的理论长度
    static int new_packed_len=0;//包的实际长度
    static unsigned char cmd_string_buf[512];
    unsigned char current_str=0x00;
    const int cmd_string_max_size=512;

    for(i=0;i<len;i++)
    {
        current_str=data[i];
       // unsigned int temp=(unsigned int)current_str;
       // std::cout<<temp<<std::endl;
        //判断是否有新包头
      if(last_str[0]==205&&last_str[1]==235&&current_str==215) //包头 205 235 215
        {
            //std::cout<<"runup1 "<<std::endl;
            new_packed_ctr=ENABLE;
            new_packed_ok_len=0;
            new_packed_len=new_packed_ok_len;
            last_str[0]=last_str[1];//保存最后两个字符，用来确定包头
            last_str[1]=current_str;
            continue;
        }
        last_str[0]=last_str[1];//保存最后两个字符，用来确定包头
        last_str[1]=current_str;
        if(new_packed_ctr==ENABLE)
        {

            //获取包长度
            new_packed_ok_len=current_str;
            if(new_packed_ok_len>cmd_string_max_size) new_packed_ok_len=cmd_string_max_size; //包内容最大长度有限制
            new_packed_ctr=DISABLE;
            //std::cout<<"runup2 "<< new_packed_len<< new_packed_ok_len<<std::endl;
        }
        else
        {
            //判断包当前大小
            if(new_packed_ok_len<=new_packed_len||new_packed_ok_len>20)
            {
                //std::cout<<"runup3 "<< new_packed_len<< new_packed_ok_len<<std::endl;
                //包长度已经大于等于理论长度，或者大于２０字节，后续内容无效
                continue;
            }
            else
            {
                //获取包内容
                new_packed_len++;
                cmd_string_buf[new_packed_len-1]=current_str;
                if(new_packed_ok_len==new_packed_len&&new_packed_ok_len>0)
                {
                    // std::cout<<"runup4 "<<std::endl;
                    //当前包已经处理完成，开始处理
                    galileo_serial_server::GalileoNativeCmds currentCmds;
                    currentCmds.header.stamp = ros::Time::now();
                    currentCmds.header.frame_id = "galileo_serial_server";
                    currentCmds.length = new_packed_ok_len;
                    currentCmds.data.resize(new_packed_ok_len);

                    for(int i=0;i<new_packed_ok_len;i++)
                    {
                      currentCmds.data[i] = cmd_string_buf[i];
                    }
                    mgalileoCmdsPub_.publish(currentCmds);
                    new_packed_ok_len=0;
                    new_packed_len=0;
                }
            }
        }
    }
    return;
}

void StatusPublisher::UpdateStatus(const galileo_serial_server::GalileoStatus & current_receive_status)
{
  boost::mutex::scoped_lock lock(mStausMutex_);
  car_status.nav_status = current_receive_status.navStatus;
  car_status.visual_status = current_receive_status.visualStatus;
  car_status.power = current_receive_status.power;
  car_status.target_numID = current_receive_status.targetNumID;
  car_status.target_status = current_receive_status.targetStatus;
  car_status.target_distance = current_receive_status.targetDistance;
  car_status.angle_goal_status = current_receive_status.angleGoalStatus;
  car_status.control_speed_x = current_receive_status.controlSpeedX;
  car_status.control_speed_theta = current_receive_status.controlSpeedTheta;
  car_status.current_speed_x = current_receive_status.currentSpeedX;
  car_status.current_speed_theta = current_receive_status.currentSpeedTheta;
}

void StatusPublisher::run()
{
  ros::Subscriber sub = mNH_.subscribe(galileoStatus_topic_, 1, &StatusPublisher::UpdateStatus, this);
  ros::spin();
}

}
