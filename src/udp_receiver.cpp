#include <math.h>
#include <iostream>
#include <tiff.h>
#include <cmath>
#include <string>
#include <sstream>
#include "std_msgs/String.h"

#include <ros/ros.h>
#include "basic_behavior.h"
#include "decision/UDPReceived.h"



int main(int argc, char **argv)
{
    ros::init(argc, argv, "udp_receiver");
    ros::NodeHandle nodeHandle;

    ros::Publisher publish_udp_receiver = nodeHandle.advertise<decision::UDPReceived>("decision/udp_receiver", 10);

    /*******************Initialize UDP Communication******************/
    UDPInitComm();

    printf("Communication: Listening..\n");

    int count=-1;

    while(ros::ok())
    {

//        // Test part
//        decision::UDPReceived udp_msg;
//        udp_msg.header.stamp = ros::Time::now();
//        for (int i=0;i<10;i++)
//            udp_msg.received_data.at(i) = 1.0;

//        publish_udp_receiver.publish(udp_msg);
//        ROS_INFO("UDP Receiver: Published data for the main node");

        count=UDPRcv(broadcast.inputSocket,broadcast.InputPacket,sizeof(broadcast.InputPacket));

        if(count!=-1)
        {
            if(broadcast.InputPacket[0]==-1000)
            {
                printf(" Listen thread exit safely, time is negative!\n");
                pthread_exit(0);
            }
            else
            {
//                printf("UDP Communication: Rcv: %f %f %f %f %f %f %f %f %f %f \n",broadcast.InputPacket[0],broadcast.InputPacket[1],broadcast.InputPacket[2],
//                        broadcast.InputPacket[3],broadcast.InputPacket[4],broadcast.InputPacket[5],broadcast.InputPacket[6],
//                        broadcast.InputPacket[7],broadcast.InputPacket[8],broadcast.InputPacket[9]);
                decision::UDPReceived udp_msg;
                udp_msg.header.stamp = ros::Time::now();
                for (int i=0;i<10;i++)
                    udp_msg.received_data.at(i) = broadcast.InputPacket[i];

//                ROS_INFO("UDP Communication: Will publish: %f %f %f %f %f %f %f %f %f %f \n",udp_msg.received_data.at(0),udp_msg.received_data.at(1),udp_msg.received_data.at(2),
//                        udp_msg.received_data.at(3),udp_msg.received_data.at(4),udp_msg.received_data.at(5),udp_msg.received_data.at(6),udp_msg.received_data.at(7),
//                         udp_msg.received_data.at(8),udp_msg.received_data.at(9));

                publish_udp_receiver.publish(udp_msg);
            }
        }
        else
            printf("UDPRcv Error!\n");

        ros::spinOnce();

        //sleep(1);



    }
    return 0;

}



