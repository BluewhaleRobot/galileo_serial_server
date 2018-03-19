#include "galelio_serial_server/AsyncSerial.h"
#include <iostream>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/package.h>

using namespace std;

int main(int argc, char **argv)
{
    cout<<"welcome to galelio serial server,please feel free at home!"<<endl;

    ros::init(argc, argv, "galelio_serial_server");
    ros::start();

    //获取串口参数
    std::string port;
    ros::param::param<std::string>("~port", port, "/dev/ttyUSB1");
    int baud;
    ros::param::param<int>("~baud", baud, 115200);
    cout<<"port:"<<port<<" baud:"<<baud<<endl;

    string galelioCmds_topic,galelioStatus_topic;
    ros::param::param<std::string>("~galelioCmds_topic", galelioCmds_topic, "/galelioSerialServer/cmds");
    ros::param::param<std::string>("~galelioStatus_topic", galelioStatus_topic, "/galelioSerialServer/status");

    try {
        CallbackAsyncSerial serial(port,baud);
        galelio_serial_server::StatusPublisher galelio_server(galelioCmds_topic,galelioStatus_topic,&serial);
        serial.setCallback(boost::bind(&galelio_serial_server::StatusPublisher::UpdateCmds,&galelio_server,_1,_2));
        boost::thread cmd2serialThread(&galelio_serial_server::StatusPublisher::run,&galelio_server);

        ros::Rate r(30);//发布周期为50hz
        while (ros::ok())
        {
            galelio_server.Refresh();//定时发布状态
            r.sleep();
        }
        quit:
        serial.close();

    } catch (std::exception& e) {
        cerr<<"Exception: "<<e.what()<<endl;
    }
    ros::shutdown();
    return 0;
}
