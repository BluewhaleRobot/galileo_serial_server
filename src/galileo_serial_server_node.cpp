#include "galileo_serial_server/AsyncSerial.h"
#include <iostream>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include "galileo_serial_server/galileo_serial_server.h"

using namespace std;

int main(int argc, char **argv)
{
    cout<<"welcome to galileo serial server,please feel free at home!"<<endl;

    ros::init(argc, argv, "galileo_serial_server");
    ros::start();

    //获取串口参数
    std::string port;
    ros::param::param<std::string>("~port", port, "/dev/ttyUSB1");
    int baud;
    ros::param::param<int>("~baud", baud, 115200);
    cout<<"port:"<<port<<" baud:"<<baud<<endl;

    string galileoCmds_topic,galileoStatus_topic;
    ros::param::param<std::string>("~galileoCmds_topic", galileoCmds_topic, "/galileoSerialServer/cmds");
    ros::param::param<std::string>("~galileoStatus_topic", galileoStatus_topic, "/galileoSerialServer/status");

    try {
        CallbackAsyncSerial serial(port,baud);
        galileo_serial_server::StatusPublisher galileo_server(galileoCmds_topic,galileoStatus_topic,&serial);
        serial.setCallback(boost::bind(&galileo_serial_server::StatusPublisher::UpdateCmds,&galileo_server,_1,_2));
        boost::thread cmd2serialThread(&galileo_serial_server::StatusPublisher::run,&galileo_server);

        ros::Rate r(30);//发布周期为50hz
        while (ros::ok())
        {
            galileo_server.Refresh();//定时发布状态
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
