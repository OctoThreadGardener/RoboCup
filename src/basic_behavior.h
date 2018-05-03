#pragma once

#include "../Xabsl/XabslEngine/XabslBasicBehavior.h"
#include <iostream>
#include "Definitions.h"


// Headers for the UDP Communication
#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>


#include <stdio.h>
#include <signal.h>
#include <sys/mman.h>
#include <memory.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/rtc.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <sys/stat.h>
#include <arpa/inet.h>

using namespace std;
using namespace xabsl;


/////////////////////////////////////Socket Init /////////////////////////////////////////////
typedef struct {
    char robotID;
    float OutputPacket[11];//the content of this packet
    float InputPacket[10];
    bool ReceiveExecuted;//updated means that receiver can receive information
    bool SendExecuted;//true means that receiver has received the information,and the sender can send;false means the sender is sending
    int outputSocket;
    int inputSocket;
    int port;
    char buffer[256];
    float fPos;
    float fVel;
    float fGoal;
    bool SocketEnable;
}Broadcast;

/*

Output Packet Structure
FLOAT[0]: Command Type

1-Step straight forward (Use with step_amount, step_length)
2-Step straight backward (Use with step_amount, step_length)
3-Step left (Use with step_amount, step_width)
4-Step right (Use with step_amount, step_width)
5-Walk - curve (Use with target_x, target_y, target_theta)
6-Circle Clockwise (Use with circle_radius, circle_theta)
7-Circle Counter-clockwise (Use with circle_radius, circle_theta)
8-Kick (Use with kick_leg, kick_strength, kick_angle)
9-Nothing/Stop
FLOAT[1]: step_amount for 1-4, target_x for 5, circle_radius for 6-7, kick_leg for 8
FLOAT[2]: step_length for 1-2, step_width for 3-4, target_y for 5, circle_theta for 6-7, kick_strength for 8
FLOAT[3]: target_theta for 5, kick_angle for 8
FLOAT[4]: Head yaw
FLOAT[5]: Head tilt


Input Packet Structure
FLOAT[0]: Time
FLOAT[1]: Robot_mti_Yaw
FLOAT[2]: Robot_mti_Pitch
FLOAT[3]: Robot_mti_Roll
FLOAT[4]: Odometry_dx (positive:front)
FLOAT[5]: Odometry_dy (positive:right)
FLOAT[6]: Odometry_dtheta (positive:clockwise)
FLOAT[7]: Is_Robot_Moving(0:not moving, 1 moving)
FLOAT[8]: Can't Kick(0:Can kick-normal, 1 can't kick)

*/

bool bIsRecData = false;
Broadcast broadcast;
int outputPort=51655;
int inputPort=51656;
int UDPSocket(int port){
    int sockfd;							//定义socket套接字
    struct sockaddr_in sin; 					//定义网络套接字地址结构
    bzero(&sin,sizeof(sin));						//地址结构清零
    sin.sin_family = AF_INET;					//指定使用的通讯协议族
    sin.sin_addr.s_addr = htonl(INADDR_ANY);			//指定接受任何连接
    sin.sin_port = htons(port);					//指定监听的端口
    sockfd = socket(AF_INET,SOCK_DGRAM,0);			//创建UDP套接字
    bind(sockfd,(struct sockaddr *)&sin,sizeof(sin));		//给套接字绑定端口
    return sockfd;
}

int UDPSendOnce(int sock,float* buffer,int size,char* IP,int port){
    struct sockaddr_in address;

    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    address.sin_addr.s_addr = inet_addr(IP);
    int count=sendto(sock,buffer,size,0,(struct sockaddr *)&address,sizeof(address));
    return count;
}

int UDPRcv(int sock,float* buffer,int size){
    struct sockaddr_in sin;
    int sinlen = sizeof(sin);
    //printf("Entered UDPRcv \n");
    int count=recvfrom(sock,buffer,size,0, (struct sockaddr *)&sin,(socklen_t*)&sinlen);
    //printf("Finished UDPRcv \n");
    return count;
}

void SetSocketOption(int socket){
    struct linger {
        u_short l_onoff;
        u_short l_linger;
    };
    linger m_sLinger;
    m_sLinger.l_onoff=1;	//(在closesocket()调用,但是还有数据没发送完毕的时候容许逗留)
    m_sLinger.l_linger=5;	//(容许逗留的时间为5秒)

    int fBroadcast=1;
//	setsockopt(socket,SOL_SOCKET,SO_LINGER,(const char*)&m_sLinger,sizeof(linger));
    setsockopt(socket,SOL_SOCKET,SO_BROADCAST,&fBroadcast,sizeof(fBroadcast));
}

void UDPInitComm()
{
    broadcast.port=51656;


    broadcast.outputSocket=UDPSocket(outputPort);
    broadcast.inputSocket=UDPSocket(inputPort);
    if(broadcast.outputSocket==-1){
        printf("Communication: Create Output socket error!\n");
        broadcast.SocketEnable=false;
    }
    else broadcast.SocketEnable=true;

    SetSocketOption(broadcast.outputSocket);
    SetSocketOption(broadcast.inputSocket);
}

void UDPSendData()
{
    int count=-1;
    count= UDPSendOnce(broadcast.outputSocket,broadcast.OutputPacket,sizeof(broadcast.OutputPacket),"192.168.8.39",51655);
    if(count ==-1)printf("Communication: UDPSend Data Error!\n");
}

//////////////////////////////////////////////////////////////////////////////////////////////


class behavior_initialize_motors : public BasicBehavior
{
public:

    behavior_initialize_motors(xabsl::ErrorHandler &errorHandler)
        : xabsl::BasicBehavior("behavior_initialize_motors", errorHandler)
    {	}

    virtual void registerParameters()
    {

    }

    virtual void execute();
};


class behavior_nothing : public BasicBehavior
{
public:

    behavior_nothing(xabsl::ErrorHandler &errorHandler)
        : xabsl::BasicBehavior("behavior_nothing", errorHandler)
        {	}

        virtual void registerParameters()
        {

        }

        virtual void execute();
};



class behavior_end_game : public BasicBehavior
{
public:

    behavior_end_game(xabsl::ErrorHandler &errorHandler)
        : xabsl::BasicBehavior("behavior_end_game", errorHandler)
        {	}

        virtual void registerParameters()
        {

        }

        virtual void execute();
};

class behavior_initialize : public BasicBehavior
{
public:

    behavior_initialize(xabsl::ErrorHandler &errorHandler)
        : xabsl::BasicBehavior("behavior_initialize", errorHandler)
        {	}

        virtual void registerParameters()
        {

        }

        virtual void execute();
};


class behavior_center_round_clockwise : public BasicBehavior
{
public:

    behavior_center_round_clockwise(xabsl::ErrorHandler &errorHandler)
        : xabsl::BasicBehavior("behavior_center_round_clockwise", errorHandler)
    {	}

    virtual void registerParameters()
    {

    }

    virtual void execute();
};


class behavior_center_round_anticlockwise : public BasicBehavior
{
public:

    behavior_center_round_anticlockwise(xabsl::ErrorHandler &errorHandler)
        : xabsl::BasicBehavior("behavior_center_round_anticlockwise", errorHandler)
    {	}

    virtual void registerParameters()
    {

    }

    virtual void execute();
};


class behavior_round_clockwise : public BasicBehavior
{
public:

    behavior_round_clockwise(xabsl::ErrorHandler &errorHandler)
        : xabsl::BasicBehavior("behavior_round_clockwise", errorHandler)
    {	}

    virtual void registerParameters()
    {

    }

    virtual void execute();
};


class behavior_round_anticlockwise : public BasicBehavior
{
public:

    behavior_round_anticlockwise(xabsl::ErrorHandler &errorHandler)
        : xabsl::BasicBehavior("behavior_round_anticlockwise", errorHandler)
    {	}

    virtual void registerParameters()
    {

    }

    virtual void execute();
};


class behavior_walk : public BasicBehavior
{
public:

    behavior_walk(xabsl::ErrorHandler &errorHandler)
        : xabsl::BasicBehavior("behavior_walk", errorHandler)
    {	}

    virtual void registerParameters()
    {

    }

    virtual void execute();
};

class behavior_step_forward : public BasicBehavior
{
public:

    behavior_step_forward(xabsl::ErrorHandler &errorHandler)
        : xabsl::BasicBehavior("behavior_step_forward", errorHandler)
    {	}

    virtual void registerParameters()
    {

    }

    virtual void execute();
};

class behavior_step_backward : public BasicBehavior
{
public:

    behavior_step_backward(xabsl::ErrorHandler &errorHandler)
        : xabsl::BasicBehavior("behavior_step_backward", errorHandler)
    {	}

    virtual void registerParameters()
    {

    }

    virtual void execute();
};

class behavior_step_left : public BasicBehavior
{
public:

    behavior_step_left(xabsl::ErrorHandler &errorHandler)
        : xabsl::BasicBehavior("behavior_step_left", errorHandler)
    {	}

    virtual void registerParameters()
    {

    }

    virtual void execute();
};

class behavior_step_right : public BasicBehavior
{
public:

    behavior_step_right(xabsl::ErrorHandler &errorHandler)
        : xabsl::BasicBehavior("behavior_step_right", errorHandler)
    {	}

    virtual void registerParameters()
    {

    }

    virtual void execute();
};

class behavior_kick_ball_soft : public BasicBehavior
{
public:

    behavior_kick_ball_soft(xabsl::ErrorHandler &errorHandler)
        : xabsl::BasicBehavior("behavior_kick_ball_soft", errorHandler)
    {	}

    virtual void registerParameters()
    {

    }

    virtual void execute();
};

class behavior_kick_ball_mid : public BasicBehavior
{
public:

    behavior_kick_ball_mid(xabsl::ErrorHandler &errorHandler)
        : xabsl::BasicBehavior("behavior_kick_ball_mid", errorHandler)
    {	}

    virtual void registerParameters()
    {

    }

    virtual void execute();
};

class behavior_kick_ball_strong : public BasicBehavior
{
public:

    behavior_kick_ball_strong(xabsl::ErrorHandler &errorHandler)
        : xabsl::BasicBehavior("behavior_kick_ball_strong", errorHandler)
    {	}

    virtual void registerParameters()
    {

    }

    virtual void execute();
};

/*
 * Additions 2018 for opt_go_to_ball
 */

class behavior_rotate_before_walk: public BasicBehavior
{
public:

    behavior_kick_ball_strong(xabsl::ErrorHandler &errorHandler)
        : xabsl::BasicBehavior("behavior_rotate_before_walk", errorHandler)
    {	}

    virtual void registerParameters()
    {
    }

    // rotate toward the line between the robot and the ball
    // head follows the ball
    // set flag: is_within_tol, is_left
    virtual void execute();
};

class behavior_during_left_walk: public BasicBehavior
{
public:

    behavior_during_left_walk(xabsl::ErrorHandler &errorHandler)
        : xabsl::BasicBehavior("behavior_during_left_walk", errorHandler)
    {	}

    virtual void registerParameters()
    {
    }

    // target position is set to within an acceptable range of the ball
    // walk along an arc between the current position and midpoint(current position, target position)
    // use velocity mode to control the gait
    // head follows the ball
    // set flag: is_left, is_near_enough
    virtual void execute();
};

class behavior_during_right_walk: public BasicBehavior
{
public:

    behavior_during_right_walk(xabsl::ErrorHandler &errorHandler)
        : xabsl::BasicBehavior("behavior_during_right_walk", errorHandler)
    {	}

    virtual void registerParameters()
    {
    }

    // walk along an arc between the current position the target position
    // use velocity mode to control the gait
    // head follows the ball
    // set flag: is_left, is_near_enough
    virtual void execute();
};

class behavior_approach_ball: public BasicBehavior
{
public:

    behavior_approach_ball(xabsl::ErrorHandler &errorHandler)
        : xabsl::BasicBehavior("behavior_approach_ball", errorHandler)
    {	}

    virtual void registerParameters()
    {
    }

    // walk toward the ball
    // use velocity mode
    // head follows the ball
    // set flag: is_ready_to_kick
    virtual void execute();
};

class behavior_stop_walk: public BasicBehavior
{
public:

    behavior_approach_ball(xabsl::ErrorHandler &errorHandler)
        : xabsl::BasicBehavior("behavior_stop_walk", errorHandler)
    {	}

    virtual void registerParameters()
    {
    }

    // use velocity mode
    // set velocity to 0
    // set flag: is_robot_moving
    virtual void execute();
};
