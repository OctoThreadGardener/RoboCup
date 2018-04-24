#include <math.h>
#include "Definitions.h"
#include "tools.h"
#include "../Xabsl/XabslEngine/XabslEngine.h"

#include "basic_behavior.h"
#include "xabsl-debug-interface.h"

using namespace xabsl;
using namespace decision;

MyErrorHandler errorHandler;
bool isInitializing = true;
std_msgs::String console_msg;
stringstream console_strstream;
//int delay_down_flag = 0;// Delay
//int delay_up_flag = 0;// Delay

void xabslEngineRegister(xabsl::Engine *pEngine, MyErrorHandler &errorHandler);
bool processHeadMode();

int udp_check_counter=0;
float last_robot_orientation = 0;
double odom_orientation = 0;
double previous_odom_orientation = 0;
double odom_x = 0;
double odom_y = 0;
double previous_x = 0;
double previous_y = 0;

void CBonUDPReceived(const UDPReceived::ConstPtr udp_msg)
{
    int internal_udp_counter = 0;
    for (int i=0; i<udp_msg->received_data.size();i++)
    {
        broadcast.InputPacket[i] = udp_msg->received_data.at(i);
        if (broadcast.InputPacket[i] == 0)
            internal_udp_counter++;
    }
    if (internal_udp_counter == udp_msg->received_data.size())
        udp_check_counter++;

    if (udp_check_counter>=800)
        ROS_ERROR("Haven't received UDP data");

    ROS_INFO("decision main node: Received: %f %f %f %f %f %f %f %f %f %f \n",broadcast.InputPacket[0],broadcast.InputPacket[1],broadcast.InputPacket[2],
            broadcast.InputPacket[3],broadcast.InputPacket[4],broadcast.InputPacket[5],broadcast.InputPacket[6],
            broadcast.InputPacket[7],broadcast.InputPacket[8],broadcast.InputPacket[9]);

    // Appoint them to the actual variables: robot yaw,pitch,roll. Odometry will be read by localization!
    // Not used now!

    //if the curve starts again, just add to the previous
    if ((broadcast.InputPacket[4] == 0) && (broadcast.InputPacket[5] == 0) && (broadcast.InputPacket[6] == 0))
    {
        previous_odom_orientation = odom_orientation;
        previous_x = odom_x;
        previous_y = odom_y;
    }

    odom_orientation = -broadcast.InputPacket[6] + previous_odom_orientation;
    odom_x = previous_x +  broadcast.InputPacket[4]*cos(previous_odom_orientation) + broadcast.InputPacket[5]*sin(previous_odom_orientation);
    odom_y = previous_y -  broadcast.InputPacket[4]*sin(previous_odom_orientation) + broadcast.InputPacket[5]*cos(previous_odom_orientation);




}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_node_open");
    ros::NodeHandle nodeHandle;

    /*******************Initialize UDP Communication******************/
    UDPInitComm();

    /*******************sevice Client******************/ // NO serviceto be used in this version
    //    ros::ServiceClient walkClient = nodeHandle.serviceClient<walk>(WALK_CONTROL_SRV);
    //    ros::ServiceClient headClient = nodeHandle.serviceClient<head>(HEAD_CONTROL_SRV);
    //    ros::ServiceClient OrientationClient = nodeHandle.serviceClient<Pathplaning_the>(ORI_CONTROL_SRV);

    /*******************message Subscriber*************/
    ros::Subscriber subscriber_gameControl = nodeHandle.subscribe(GAMECONTROL_OUTPUT_TOPIC, 10, CBOnControllingReceived);
    //call back function is stated in the file Definition.h & Definition.cpp
    ros::Subscriber subscriber_goal = nodeHandle.subscribe(GOAL_RECOG_OUTPUT_TOPIC, 10, CBOnGoalReceived);
    ros::Subscriber subscriber_ball = nodeHandle.subscribe(BALL_RECOG_OUTPUT_TOPIC, 10, CBOnBallReceived);
    ros::Subscriber subscriber_obstacles = nodeHandle.subscribe(OBSTACLES_RECOG_OUTPUT_TOPIC, 10, CBOnObstaclesReceived);
    ros::Subscriber subscriber_head_gyro = nodeHandle.subscribe(HEAD_GYRO_OUTPUT_TOPIC, 10, CBOnHeadGyroReceived);
    ros::Subscriber subscriber_localization = nodeHandle.subscribe(LOCALIZATION_OUTPUT_TOPIC, 10, CBOnLocalizationReceived);

    ros::Subscriber subscriber_udp_receiver = nodeHandle.subscribe("decision/udp_receiver", 10, CBonUDPReceived);

    /*******************message publisher*************/
    ros::Publisher publish_decision_console = nodeHandle.advertise<std_msgs::String>("decision_console", 10);

    /*******************publisher of head rotation*************/
    ros::Publisher publish_head_rotation = nodeHandle.advertise<decision::UDPReceived>("decision/head_rotation", 10);

    /*******************Xabsl*************************/
    Engine *pEngine = new Engine(errorHandler, &getSystemTime);
    MyFileInputSource inputSource("/home/robocup17/robocup_ws/src/decision/intermediate_code/2017test1.dat");  //change that according to the required behavior
    //    MyFileInputSource inputSource("/home/sotiris/robocup2017_ws/src/decision/intermediate_code/2017test1.dat");
    xabslEngineRegister(pEngine, errorHandler);
    /*******************Xabsl_behavior****************/


    behavior_initialize_motors _behavior_initialize_motors(errorHandler);
    behavior_nothing _behavior_nothing(errorHandler);
    behavior_end_game _behavior_end_game(errorHandler);
    behavior_initialize _behavior_initialize(errorHandler);
    behavior_center_round_clockwise _behavior_center_round_clockwise(errorHandler);
    behavior_center_round_anticlockwise _behavior_center_round_anticlockwise(errorHandler);
    behavior_round_clockwise _behavior_round_clockwise(errorHandler);
    behavior_round_anticlockwise _behavior_round_anticlockwise(errorHandler);
    behavior_walk _behavior_walk(errorHandler);
    behavior_step_forward _behavior_step_forward(errorHandler);
    behavior_step_backward _behavior_step_backward(errorHandler);
    behavior_step_left _behavior_step_left(errorHandler);
    behavior_step_right _behavior_step_right(errorHandler);
    behavior_kick_ball_soft _behavior_kick_ball_soft(errorHandler);
    behavior_kick_ball_mid _behavior_kick_ball_mid(errorHandler);
    behavior_kick_ball_strong _behavior_kick_ball_strong(errorHandler);

    pEngine->registerBasicBehavior(_behavior_initialize_motors);
    pEngine->registerBasicBehavior(_behavior_nothing);
    pEngine->registerBasicBehavior(_behavior_end_game);
    pEngine->registerBasicBehavior(_behavior_initialize);
    pEngine->registerBasicBehavior(_behavior_center_round_clockwise);
    pEngine->registerBasicBehavior(_behavior_center_round_anticlockwise);
    pEngine->registerBasicBehavior(_behavior_round_clockwise);
    pEngine->registerBasicBehavior(_behavior_round_anticlockwise);
    pEngine->registerBasicBehavior(_behavior_walk);
    pEngine->registerBasicBehavior(_behavior_step_forward);
    pEngine->registerBasicBehavior(_behavior_step_backward);
    pEngine->registerBasicBehavior(_behavior_step_left);
    pEngine->registerBasicBehavior(_behavior_step_right);
    pEngine->registerBasicBehavior(_behavior_kick_ball_soft);
    pEngine->registerBasicBehavior(_behavior_kick_ball_mid);
    pEngine->registerBasicBehavior(_behavior_kick_ball_strong);


    pEngine->createOptionGraph(inputSource);
    ROS_INFO("PAUSE!");
    ros::Rate loop_rate(MAIN_NODE_RUNNING_RATE);
    ROS_INFO("Initializing xabsl engine...\n");
    debug_interface dbg(pEngine);
    ROS_INFO("xabsl engine initialized, FSM starts to run...\n");

    int roundCount = 0;

    currentFrame.foundBallTime = ros::Time::now().toSec();
    currentFrame.foundGoalTime = ros::Time::now().toSec();
    currentFrame.is_robot_moving = false;
        currentFrame.cannot_kick = false;
    //currentFrame.is_robot_moving_ball_flag = false;
    broadcast.InputPacket[7] = 0;
    currentFrame.goalFoundHeadAngleYaw = 0;
    currentFrame.ballFoundHeadAngleYaw = 0;
    broadcast.InputPacket[8] = 0;



    while (ros::ok())
    {
        console_strstream.str("");

        console_strstream << "\nround: " << ++roundCount << endl;

        ros::spinOnce();



        currentFrame.FlushData(-broadcast.InputPacket[3],odom_orientation, odom_x, odom_y);

        ROS_INFO("robot moving: %f",broadcast.InputPacket[7]);
        // Update is_robot_moving
        currentFrame.is_robot_moving = (broadcast.InputPacket[7] == 1);
        currentFrame.cannot_kick = (broadcast.InputPacket[8] == 1);

                // Debug
//        currentFrame.isReady = true;
//        currentFrame.isGameStart = true;
//        currentFrame.isGameOver = false;
//        currentFrame.isAttacker = true;///debug*/
//        currentFrame.sec_state = 0;
//        currentFrame.sec_state_info = 0;

        currentFrame.PrintReceivedData();
        //currentFrame includes the message subscribed from the service

        ROS_INFO("before execute");

        pEngine->execute();

        console_strstream << dbg.showDebugInfo().str();

        if (roundCount != 1)
        {
            processHeadMode();
        }
        // Publish the demanded head angle
        decision::UDPReceived head_rotation_msg;
        head_rotation_msg.received_data.at(0) = currentFrame.headAnglePitch / 180 *M_PI; //rad
        head_rotation_msg.received_data.at(1) = currentFrame.headAngleYaw / 180 *M_PI; //rad

        publish_head_rotation.publish(head_rotation_msg);
ROS_INFO("Sent head rotation %f,%f",head_rotation_msg.received_data.at(0),head_rotation_msg.received_data.at(1));

        ////////////debug
        console_msg.data = console_strstream.str();
        publish_decision_console.publish(console_msg);

        cout << console_strstream.str() << endl;

        //		for (int i=0;i<=10;i++)
        //{
        //        // Test send data
        //        broadcast.OutputPacket[0] = 0;
        //        broadcast.OutputPacket[1] = 0;
        //        broadcast.OutputPacket[2] = 0;
        //        broadcast.OutputPacket[3] = 0;
        //        broadcast.OutputPacket[4] = (i-5)*7;  //negative is right
        //if ((i-5)<=0)
        //broadcast.OutputPacket[5] = 25; // just have head tilted to 25 degrees
        //else
        //broadcast.OutputPacket[5] = 35;

        //        UDPSendData();
        //        ROS_INFO("Sent data via UDP");

        //        loop_rate.sleep();
        //}

        //                // Test send data
        //                broadcast.OutputPacket[0] = 1;
        //                broadcast.OutputPacket[1] = 2;
        //                broadcast.OutputPacket[2] = 3;
        //                broadcast.OutputPacket[3] = 4;
        //                broadcast.OutputPacket[4] = 5;
        //                broadcast.OutputPacket[5] = 25;
        //                broadcast.OutputPacket[6] = 30;
        //                broadcast.OutputPacket[7] = 0;
        //                broadcast.OutputPacket[8] = 0;
        //                broadcast.OutputPacket[9] = 0;
        //                broadcast.OutputPacket[10] = 0;

        // When all the decisions are taken, send the data to the motion NUC via UDP

        broadcast.OutputPacket[7] = currentFrame.received_ball.ball_center_x; // ball_x
        broadcast.OutputPacket[8] = currentFrame.received_ball.ball_center_y; // ball_y

        UDPSendData();

        ROS_INFO("Sent data via UDP:%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",broadcast.OutputPacket[0],broadcast.OutputPacket[1],
                broadcast.OutputPacket[2],broadcast.OutputPacket[3],broadcast.OutputPacket[4],
                broadcast.OutputPacket[5],broadcast.OutputPacket[6],broadcast.OutputPacket[7],
                broadcast.OutputPacket[8],broadcast.OutputPacket[9],broadcast.OutputPacket[10]);

        loop_rate.sleep();

    }

    broadcast.OutputPacket[0] = 255;
    UDPSendData();

    return 0;
}

// Function that
void xabslEngineRegister(xabsl::Engine *pEngine, MyErrorHandler &errorHandler)
{
    pEngine->registerEnumElement("_orientation", "_orientation.Up", Up);
    pEngine->registerEnumElement("_orientation", "_orientation.Down", Down);
    pEngine->registerEnumElement("_orientation", "_orientation.Left", Left);
    pEngine->registerEnumElement("_orientation", "_orientation.Right", Right);
    pEngine->registerEnumElement("_orientation", "_orientation.Mid", Mid);

    pEngine->registerEnumElement("_headMode", "_headMode.FarLeft", FarLeft);
    pEngine->registerEnumElement("_headMode", "_headMode.FarMid", FarMid);
    pEngine->registerEnumElement("_headMode", "_headMode.FarRight", FarRight);
    pEngine->registerEnumElement("_headMode", "_headMode.CloseLeft", CloseLeft);
    pEngine->registerEnumElement("_headMode", "_headMode.CloseMid", CloseMid);
    pEngine->registerEnumElement("_headMode", "_headMode.CloseRight", CloseRight);
    pEngine->registerEnumElement("_headMode", "_headMode.HorizontalTrack", HorizontalTrack);
    pEngine->registerEnumElement("_headMode", "_headMode.VerticalTrack", VerticalTrack);
    pEngine->registerEnumElement("_headMode", "_headMode.BothTrack", BothTrack);


    //Game
    pEngine->registerBooleanInputSymbol("isReady", &currentFrame.isReady);
    pEngine->registerBooleanInputSymbol("isSet", &currentFrame.isSet);
    pEngine->registerBooleanInputSymbol("isGameStart", &currentFrame.isGameStart);
    pEngine->registerBooleanInputSymbol("isGameOver", &currentFrame.isGameOver);
    pEngine->registerDecimalInputSymbol("timeLeft", &currentFrame.timeLeft);
    pEngine->registerBooleanInputSymbol("isPathPlanningRequested", &currentFrame.Pathplan);
    pEngine->registerBooleanInputSymbol("isAttacker", &currentFrame.isAttacker);
    pEngine->registerBooleanInputSymbol("pause", &currentFrame.pause);
    pEngine->registerDecimalInputSymbol("sec_state", &currentFrame.sec_state);
    pEngine->registerDecimalInputSymbol("sec_state_info", &currentFrame.sec_state_info);

    //Ball
    pEngine->registerBooleanInputSymbol("isBallSeen", &currentFrame.isBallSeen);
    pEngine->registerDecimalInputSymbol("ballRange", &currentFrame.ballRange);
    pEngine->registerDecimalInputSymbol("ballBearing", &currentFrame.ballBearing);
    pEngine->registerDecimalInputSymbol("ballLoc.World.x", &currentFrame.ballLoc_world_x);
    pEngine->registerDecimalInputSymbol("ballLoc.World.y", &currentFrame.ballLoc_world_y);
    pEngine->registerDecimalInputSymbol("lostBallDir", &currentFrame.lostBallDir);

    pEngine->registerBooleanInputSymbol("first_time_track_ball", &currentFrame.first_time_track_ball);
    pEngine->registerBooleanInputSymbol("ball_moved_while_moving", &currentFrame.ball_moved_while_moving);
    pEngine->registerDecimalInputSymbol("expected_ball_bearing", &currentFrame.expected_ball_bearing);
    pEngine->registerDecimalInputSymbol("ballFoundHeadAngleYaw", &currentFrame.ballFoundHeadAngleYaw);
pEngine->registerBooleanInputSymbol("cannot_kick", &currentFrame.cannot_kick);

    //Goal
    pEngine->registerBooleanInputSymbol("isGoalSeen", &currentFrame.isGoalSeen);
    pEngine->registerDecimalInputSymbol("goalCenterRange", &currentFrame.goalCenterRange);  // should add the range as well, since we have it
    pEngine->registerDecimalInputSymbol("goalCenterBearing", &currentFrame.goalCenterBearing);
    pEngine->registerDecimalInputSymbol("goalLocLeft.World.x", &currentFrame.goalLocLeft_world_x);
    pEngine->registerDecimalInputSymbol("goalLocLeft.World.y", &currentFrame.goalLocLeft_world_y);
    pEngine->registerDecimalInputSymbol("goalLocRight.World.x", &currentFrame.goalLocRight_world_x);
    pEngine->registerDecimalInputSymbol("goalLocRight.World.y", &currentFrame.goalLocRight_world_y);
    pEngine->registerDecimalInputSymbol("goalFoundHeadAngleYaw", &currentFrame.goalFoundHeadAngleYaw);
    pEngine->registerDecimalInputSymbol("robot_goal_bearing_odom", &currentFrame.robot_goal_bearing_odom);

    //Opponent
    pEngine->registerBooleanInputSymbol("isOpponentSeen", &currentFrame.isOpponentSeen);
    pEngine->registerDecimalInputSymbol("opponentRange", &currentFrame.opponentRange);
    pEngine->registerDecimalInputSymbol("opponentCenterBearing", &currentFrame.opponentCenterBearing);
    pEngine->registerDecimalInputSymbol("opponentCenter.World.x", &currentFrame.opponentCenter_world_x);
    pEngine->registerDecimalInputSymbol("opponentCenter.World.y", &currentFrame.opponentCenter_world_y);

    //Robot
    pEngine->registerDecimalInputSymbol("robotLoc.x", &currentFrame.robotLoc_x);
    pEngine->registerDecimalInputSymbol("robotLoc.y", &currentFrame.robotLoc_y);
    pEngine->registerDecimalInputSymbol("robotLoc.theta", &currentFrame.robotLoc_theta);
    pEngine->registerDecimalInputSymbol("locConfidence", &currentFrame.locConfidence);
    pEngine->registerDecimalInputSymbol("gyroHead", &currentFrame.gyroHead);
    pEngine->registerDecimalInputSymbol("gyroBody", &currentFrame.gyroBody);
    pEngine->registerBooleanInputSymbol("is_robot_moving", &currentFrame.is_robot_moving);
    pEngine->registerDecimalInputSymbol("adjust_theta", &currentFrame.adjust_theta);
    pEngine->registerDecimalInputSymbol("odom_orientation", &currentFrame.odom_orientation);


    pEngine->registerDecimalInputSymbol("kickDestTheta", &currentFrame.kickDestTheta); //not used now

    pEngine->registerDecimalInputSymbol("headAngleYaw", &currentFrame.headAngleYaw);
    pEngine->registerDecimalInputSymbol("headAnglePitch", &currentFrame.headAnglePitch);

    //Output
    pEngine->registerDecimalOutputSymbol("kick_angle", &currentFrame.kickangle);
    pEngine->registerDecimalOutputSymbol("kick_speed", &currentFrame.kick_speed);
    pEngine->registerDecimalOutputSymbol("kick_leg", &currentFrame.kick_leg);
    pEngine->registerDecimalOutputSymbol("target_range", &currentFrame.target_range);
    pEngine->registerDecimalOutputSymbol("target_bearing", &currentFrame.target_bearing);
    pEngine->registerDecimalOutputSymbol("target_theta", &currentFrame.target_theta);
    pEngine->registerEnumeratedOutputSymbol("headMode", "_headMode", (int *)(&currentFrame.headMode));
    pEngine->registerEnumeratedOutputSymbol("lastBallDir", "_headMode", (int *)(&currentFrame.lastBallDir));
    pEngine->registerEnumeratedOutputSymbol("nextGoalDir", "_headMode", (int *)(&currentFrame.nextGoalDir));
    pEngine->registerDecimalOutputSymbol("robot_goal_bearing", &currentFrame.robot_goal_bearing);
    pEngine->registerDecimalOutputSymbol("ball_found_head_angle_yaw", &currentFrame.ball_found_head_angle_yaw);
    pEngine->registerDecimalOutputSymbol("goal_found_head_angle_yaw", &currentFrame.goal_found_head_angle_yaw);

    pEngine->registerBooleanOutputSymbol("first_kick", &currentFrame.first_kick);
    pEngine->registerDecimalOutputSymbol("turned", &currentFrame.turned);
}

// Change this to send the head mode over UDP
bool processHeadMode()
{
    if ((currentFrame.headMode != HorizontalTrack) &&
            (currentFrame.headMode != VerticalTrack) &&
            (currentFrame.headMode != BothTrack))
    {
        switch (currentFrame.headMode)
        {
        case FarLeft:
            //broadcast.OutputPacket[4] = 30;
            broadcast.OutputPacket[4] = 60;
            broadcast.OutputPacket[5] = 25;
            //broadcast.OutputPacket[6] = 30;
            broadcast.OutputPacket[6] = 0;
            break;

        case FarMid:
            broadcast.OutputPacket[4] = 0;
            broadcast.OutputPacket[5] = 25;
            broadcast.OutputPacket[6] = 0;
            break;

        case FarRight:
            //broadcast.OutputPacket[4] = -30;
            broadcast.OutputPacket[4] = -60;
            broadcast.OutputPacket[5] = 25;
            //broadcast.OutputPacket[6] = -30;
            broadcast.OutputPacket[6] = 0;
            break;

        case CloseLeft:
            //broadcast.OutputPacket[4] = 30;
            broadcast.OutputPacket[4] = 60;
            broadcast.OutputPacket[5] = 65;
            //broadcast.OutputPacket[6] = 30;
            broadcast.OutputPacket[6] = 0;
            break;

        case CloseMid:
            broadcast.OutputPacket[4] = 0;
            broadcast.OutputPacket[5] = 65;
            broadcast.OutputPacket[6] = 0;
            break;

        case CloseRight:
            //broadcast.OutputPacket[4] = -30;
            broadcast.OutputPacket[4] = -60;
            broadcast.OutputPacket[5] = 65;
            //broadcast.OutputPacket[6] = -30;
            broadcast.OutputPacket[6] = 0;
            break;
        }
    }
    else
    {
        // Calculate expected ball bearing

        // x,y,theta 4,5,6
        if (broadcast.InputPacket[5] == 0)
        {
            currentFrame.expected_ball_bearing = broadcast.InputPacket[6] - asin(broadcast.InputPacket[4] / currentFrame.ballRange
                    *sin(-currentFrame.headAngleYaw/180*M_PI));
        }
        else
        {
            currentFrame.expected_ball_bearing = broadcast.InputPacket[6] - asin(sqrt(pow(broadcast.InputPacket[4],2)+pow(broadcast.InputPacket[5],2)) / currentFrame.ballRange
                    *sin(M_PI/2-currentFrame.headAngleYaw/180*M_PI-atan(broadcast.InputPacket[4]/broadcast.InputPacket[5])));
        }

        cout << "ballRange" << currentFrame.ballRange << endl;
        cout << "ballBearing" << currentFrame.ballBearing << endl;
        cout << "expected_ball_bearing" << currentFrame.expected_ball_bearing << endl;

        if (currentFrame.headMode == BothTrack)
        {
            if ( (fabs(currentFrame.ballBearing - currentFrame.expected_ball_bearing )*180/M_PI > 15) && currentFrame.isBallSeen == 1)  //if more than 10 degrees to the right, turn head right
            {
                currentFrame.ball_moved_while_moving = true;
            }
            else
            {
                currentFrame.ball_moved_while_moving = false;
            }

            if (fabs(broadcast.OutputPacket[4] - currentFrame.ballBearing ) >90)
            {
                if (broadcast.OutputPacket[4]>0)
                    broadcast.OutputPacket[4] = 90;
                else
                    broadcast.OutputPacket[4] = -90;
            }
            else if (currentFrame.first_time_track_ball)
            {
                broadcast.OutputPacket[4] -= currentFrame.ballBearing;
                currentFrame.first_time_track_ball = false;
            }
            else if (fabs(last_robot_orientation - broadcast.InputPacket[6])/M_PI*180 > 7)
            {
                broadcast.OutputPacket[4] += (last_robot_orientation - broadcast.InputPacket[6])/M_PI*180;
            }
            else if ((currentFrame.ballBearing*180/M_PI) > 10 && currentFrame.isBallSeen == 1)  //if more than 10 degrees to the right, turn head right
            {
                broadcast.OutputPacket[4] -= 3;
                //                if(currentFrame.headAnglePitch < -40)
                //                    broadcast.OutputPacket[4] -= 5;
                //                else
                //                    broadcast.OutputPacket[4] -= 10;
            }
            else if ((currentFrame.ballBearing*180/M_PI) < -10 && currentFrame.isBallSeen == 1)
            {
                broadcast.OutputPacket[4] += 3;
                //                if(currentFrame.headAnglePitch < -40)
                //                    broadcast.OutputPacket[4] += 5;
                //                else
                //                    broadcast.OutputPacket[4] += 10;
            }
            else
            {
                broadcast.OutputPacket[4] += 0;
            }


            if (currentFrame.ballRange > 1.2 && currentFrame.isBallSeen == 1)  //if range more that 1 meter, head up
            {

                broadcast.OutputPacket[5] = 25;
            }
            else if (currentFrame.ballRange <= 1.2 && currentFrame.ballRange >= 0 && currentFrame.isBallSeen == 1)
            {

                broadcast.OutputPacket[5] = 65;
            }
            else
            {
                broadcast.OutputPacket[5] += 0;
            }

        }
    }

    currentFrame.headAnglePitch = broadcast.OutputPacket[5];
    currentFrame.headAngleYaw = broadcast.OutputPacket[4];

    currentFrame.head_angle_yaw_list.push_back(currentFrame.headAngleYaw);

//    // Split the yaw angle between the head and the waist
//    broadcast.OutputPacket[6] = broadcast.OutputPacket[4]/2;
//    broadcast.OutputPacket[4] = broadcast.OutputPacket[4]/2;

    ROS_INFO("Pitch_request: %f, Yaw_request: %f, waist:%f robot orientation dif:%f",currentFrame.headAnglePitch, currentFrame.headAngleYaw,broadcast.OutputPacket[6],(last_robot_orientation - broadcast.InputPacket[6])/M_PI*180);

    last_robot_orientation = broadcast.InputPacket[6];
}
