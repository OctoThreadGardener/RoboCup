#pragma once

#include <iostream>
#include <tiff.h>
#include <cmath>
#include <string>
#include <sstream>
#include "std_msgs/String.h"
#include <tf/tf.h>

#include <ros/ros.h>

//#include <decision/GoalData.h>
#include <stereo_process/Goalpost.h>
//#include <decision/gameControl.h>
#include <gamecontroller/gameControl.h>
#include <stereo_process/Ball.h>
//#include <decision/Ball.h>
#include <stereo_process/Obstacles.h>
#include <decision/gyro_euler.h>
#include <sensor_msgs/Imu.h>
#include <decision/head_angle.h>     //msg

#include <decision/UDPReceived.h>
#include <localization/OutputData.h>
#include "math.h"


//#define DEBUG

using namespace std;
using namespace decision;

const string GOAL_RECOG_OUTPUT_TOPIC = "vision/goalpost";
const string GAMECONTROL_OUTPUT_TOPIC = "game_state";
const string BALL_RECOG_OUTPUT_TOPIC = "vision/ball";
const string OBSTACLES_RECOG_OUTPUT_TOPIC = "vision/obstacles";
const string HEAD_GYRO_OUTPUT_TOPIC = "/imu/data";
const string UDP_RECEIVED_TOPIC = "decision/udp_receiver";
const string LOCALIZATION_OUTPUT_TOPIC = "localization/output_data";

const int MAIN_NODE_RUNNING_RATE = 1;


enum _orientation
{
    Up = 1,
    Down = 2,
    Left = 3,
    Right = 4,
    Mid = 5,
    UnKnown = 6,
};

enum _headMode
{
        FarLeft = 10,
        FarMid = 11,
        FarRight = 12,

    CloseLeft = 20,
    CloseMid = 21,		// straight
    CloseRight = 22,

    //Close = 30,

        HorizontalTrack = 41,
        VerticalTrack = 42,
        BothTrack = 43
};

/// ALL angles of XABSL are in degrees
class DataStructure
{
public:
    stereo_process::Goalpost received_goalpost;
    gamecontroller::gameControl received_gamecontrol;
    stereo_process::Ball received_ball;
    stereo_process::Obstacles received_obstacles;
    gyro_euler received_headgyro;
    localization::OutputData received_localization;


    //Game
    bool isReady;
     bool isSet;
        bool isGameStart;
        bool isGameOver;
        bool Pathplan;
    double timeLeft;
    bool isAttacker;
    bool pause;
    double sec_state;
    // STATE2_NORMAL               0
    // STATE2_PENALTYSHOOT         1
    // STATE2_OVERTIME             2
    // STATE2_TIMEOUT              3
    // STATE2_DIRECT_FREEKICK      4
    // STATE2_INDIRECT_FREEKICK    5
    // STATE2_PENALTYKICK          6
    double sec_state_info;
    // STATE2_INFO_PREPARE         0
    // STATE2_INFO_FREEZE          1
    // STATE2_INFO_EXECUTE         2


    double kickDestTheta; //not used for now

    //Ball

    bool isBallSeen;
    //double ballCenterInImageX;
    //double ballCenterInImageY;
    double ballBearing;
    double ballRange;
    double ballLoc_world_x;
    double ballLoc_world_y;

    double lostBallDir;

    bool first_time_track_ball;
    bool ball_moved_while_moving;
    double expected_ball_bearing;

    double ballFoundHeadAngleYaw;
    bool cannot_kick;



    // Goal
        bool isGoalSeen;
    double goalLocLeft_world_x;
    double goalLocLeft_world_y;
    double goalLocRight_world_x;
    double goalLocRight_world_y;
    double goalCenterRange;
    double goalCenterBearing;

    double goalFoundHeadAngleYaw;

    double robot_goal_bearing_odom;

    //Opponent
    bool isOpponentSeen;
    double opponentCenterBearing;
    double opponentRange;
    double opponentCenter_world_x;
    double opponentCenter_world_y;

    //Robot
    double robotLoc_x;
    double robotLoc_y;
    double robotLoc_theta;
    double locConfidence;
    double gyroHead;
    double gyroBody;
    double gyroInitHeadYawTheta;
    double gyroInitBodyYawTheta;
    double gyro_body_initial;
    double adjust_theta;
    double odom_orientation;

        double headAnglePitch;
        double headAngleYaw;

    bool is_robot_moving;
   // bool is_robot_moving_ball_flag;


    // Outputs

    double kickangle;
    double kick_leg;
        enum _headMode headMode;
    enum _headMode lastBallDir;
    enum _headMode nextGoalDir;
    double target_range;
    double target_bearing;
    double target_theta;
    double kick_speed;
    bool first_kick;
    double turned;
    double robot_goal_bearing;

    double foundBallTime;
    double foundGoalTime;
    bool robot_moved;

    double ball_found_head_angle_yaw;
    double goal_found_head_angle_yaw;

    vector<double> head_angle_yaw_list;

        void FlushData(double _body_gyro,double _odom_orientation, double _odom_x, double _odom_y);
        void PrintReceivedData(void);


    /*
     * Additions 2018 for opt_go_to_ball
     */

    bool is_within_tol;         // true if the angle between the body and the direction toward the ball is small enough
    bool is_left;               // true if the body is to the left of the direction toward the ball
    bool is_near_enough;        // true if near enough to ball, and object tracking mode can be enabled
    bool is_ready_to_kick;      // true when near enough to kick
};

void CBOnGoalReceived(const stereo_process::Goalpost::ConstPtr &msg );
void CBOnControllingReceived(const gamecontroller::gameControl::ConstPtr &msg);
void CBOnBallReceived(const stereo_process::Ball::ConstPtr &msg);
void CBOnObstaclesReceived(const stereo_process::Obstacles::ConstPtr &msg);
void CBOnHeadGyroReceived(const sensor_msgs::Imu::ConstPtr &msg);
void CBOnLocalizationReceived(const localization::OutputData::ConstPtr &msg);

void CBonUDPReceived(const decision::UDPReceived::ConstPtr udp_msg);

double dealWithHeadYawTheta(double inputTheta);
double dealWithBodyYawTheta(double inputTheta);

extern DataStructure currentFrame;
extern DataStructure lastFrame;

extern bool isInitializing;
extern std_msgs::String console_msg;
extern stringstream console_strstream;
