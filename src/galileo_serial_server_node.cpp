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

#include "galileo_serial_server/AsyncSerial.h"
#include <iostream>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include "galileo_serial_server/galileo_serial_server.h"

using namespace std;

int main(int argc, char** argv)
{
    ROS_INFO_STREAM("welcome to galileo serial server,please feel free at home!");

    ros::init(argc, argv, "galileo_serial_server");
    ros::start();

    //获取串口参数
    std::string port;
    ros::param::param<std::string>("~port", port, "/dev/ttyUSB1");
    int baud;
    ros::param::param<int>("~baud", baud, 115200);
    ROS_INFO_STREAM("port:" << port << " baud:" << baud);

    string galileoCmds_topic, galileoStatus_topic;
    ros::param::param<std::string>("~galileoCmds_topic", galileoCmds_topic, "/galileoSerialServer/cmds");
    ros::param::param<std::string>("~galileoStatus_topic", galileoStatus_topic, "/galileoSerialServer/status");

    try
    {
        CallbackAsyncSerial serial(port, baud);
        galileo_serial_server::StatusPublisher galileo_server(galileoCmds_topic, galileoStatus_topic, &serial);
        serial.setCallback(boost::bind(&galileo_serial_server::StatusPublisher::UpdateCmds, &galileo_server, _1, _2));
        boost::thread cmd2serialThread(&galileo_serial_server::StatusPublisher::run, &galileo_server);

        ros::Rate r(30);  //发布周期为50hz
        while (ros::ok())
        {
            if(serial.errorStatus() || serial.isOpen()==false)
            {
                cerr<<"Error: galileo_serial_server port  closed unexpectedly"<<endl;
                break;
            }
            galileo_server.Refresh();  //定时发布状态
            r.sleep();
        }
    quit:
        serial.close();
    }
    catch (std::exception& e)
    {
        ROS_ERROR_STREAM("Exception: " << e.what());
    }
    ros::shutdown();
    return 0;
}
