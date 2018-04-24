#include "Definitions.h"




// Callback functions for the received messages: goalpost, game controller, ball, opponent, head gyro, and localization
void CBOnGoalReceived(const stereo_process::Goalpost::ConstPtr &msg)
{
    currentFrame.received_goalpost = *msg;

    if ((currentFrame.received_goalpost.goalpost_number == 2)||(currentFrame.received_goalpost.goalpost_number == 3))
    {
        currentFrame.isGoalSeen = true;
        currentFrame.foundGoalTime = ros::Time::now().toSec();


//    ROS_INFO("goalFoundHeadAngleYaw: error check ");


//    ROS_INFO("goalFoundHeadAngleYaw:%f with delay:%f ",currentFrame.goalFoundHeadAngleYaw,
//             currentFrame.head_angle_yaw_list.size() - trunc(ros::Time::now().toSec()-currentFrame.received_goalpost.header.stamp.toSec()));

    if (((currentFrame.head_angle_yaw_list.size() - trunc(ros::Time::now().toSec()-currentFrame.received_goalpost.header.stamp.toSec())) >10) )

    {
        currentFrame.goalFoundHeadAngleYaw = currentFrame.head_angle_yaw_list.at(currentFrame.head_angle_yaw_list.size() -
                                                                                 trunc(ros::Time::now().toSec()-currentFrame.received_goalpost.header.stamp.toSec())-1);
    }
    else
    {
        currentFrame.goalFoundHeadAngleYaw = 0;
    }
    }


    ROS_INFO("goalFoundHeadAngleYaw:%f with delay:%f ",currentFrame.goalFoundHeadAngleYaw,
             ros::Time::now().toSec()-currentFrame.received_goalpost.header.stamp.toSec());

}

void CBOnControllingReceived(const gamecontroller::gameControl::ConstPtr &msg)
{
    currentFrame.received_gamecontrol = *msg;
    //    ROS_INFO("Received from Gamecontroler:State:%d,Sec_State:%d,time:%d",
    //             msg->state,msg->secondaryState,msg->secsRemaining);
}

void CBOnBallReceived(const stereo_process::Ball::ConstPtr &msg)
{
    currentFrame.received_ball = *msg;

    currentFrame.isBallSeen = true;

    currentFrame.foundBallTime = ros::Time::now().toSec();

        //ROS_INFO("ballFoundHeadAngleYaw: error check ");

    if ((currentFrame.head_angle_yaw_list.size() - trunc(ros::Time::now().toSec()-currentFrame.received_ball.header.stamp.toSec())) >10 )
    {
        currentFrame.ballFoundHeadAngleYaw = currentFrame.head_angle_yaw_list.at(currentFrame.head_angle_yaw_list.size() -
                                                                                 trunc(ros::Time::now().toSec()-currentFrame.received_ball.header.stamp.toSec())-1);
    }
    else
    {
        currentFrame.ballFoundHeadAngleYaw = 0;
    }

//    ROS_INFO("ballFoundHeadAngleYaw:%f with delay:%f ",currentFrame.ballFoundHeadAngleYaw,
//             ros::Time::now().toSec()-currentFrame.received_ball.header.stamp.toSec());

}

void CBOnObstaclesReceived(const stereo_process::Obstacles::ConstPtr &msg)
{
    currentFrame.received_obstacles = *msg;
}

void CBOnHeadGyroReceived(const sensor_msgs::Imu::ConstPtr &msg)
{
    tf::Quaternion imu_q;

    imu_q.setW(msg->orientation.w);
    imu_q.setX(msg->orientation.x);
    imu_q.setY(msg->orientation.y);
    imu_q.setZ(msg->orientation.z);

    //Change from imu data to head_gyro data. For now, imu roll is head pitch and imu pitch is head yaw
    tf::Matrix3x3 imu_m(imu_q);
    double imu_roll, imu_pitch, imu_yaw;
    imu_m.getRPY(imu_roll, imu_pitch, imu_yaw);

    currentFrame.received_headgyro.euler_angle.x = imu_yaw;
    currentFrame.received_headgyro.euler_angle.y = imu_roll;
    currentFrame.received_headgyro.euler_angle.z = imu_pitch;
}

void CBOnLocalizationReceived(const localization::OutputData::ConstPtr &msg)
{
    currentFrame.received_localization = *msg;
}



// Function that refreshes all the data received by the callback functions
void DataStructure::FlushData(double _body_gyro,double _odom_orientation, double _odom_x, double _odom_y)
{

    //this->kickDestTheta = lastFrame.kickDestTheta;//keep the value, this value is changed in the service "angle"
    lastFrame = *this;

    //Game
    this->timeLeft = static_cast<double>(received_gamecontrol.secsRemaining);

    this->isReady = received_gamecontrol.state == 1;
    this->isSet = received_gamecontrol.state == 2;
    this->isGameStart = received_gamecontrol.state == 3;
    this->isGameOver = received_gamecontrol.state == 4 || timeLeft > 600; /// need to change this because rules have changed!!!
    // Need to see how to add the secondary states
    this->isAttacker =  static_cast<bool>(received_gamecontrol.kickOffTeam == 45);
    // STATE2_NORMAL               0
    // STATE2_PENALTYSHOOT         1
    // STATE2_OVERTIME             2
    // STATE2_TIMEOUT              3
    // STATE2_DIRECT_FREEKICK      4
    // STATE2_INDIRECT_FREEKICK    5
    // STATE2_PENALTYKICK          6
    if ((received_gamecontrol.secondaryState == 1) || (received_gamecontrol.secondaryState == 3) || (received_gamecontrol.secondaryState == 4) ||
            (received_gamecontrol.secondaryState == 5) || (received_gamecontrol.secondaryState == 6))
        this->pause = true;
    else
        this->pause = false;
    this->sec_state = received_gamecontrol.secondaryState;
    this->sec_state_info = received_gamecontrol.secondaryStateInfo;

    //Ball
    if (currentFrame.isBallSeen == true)
    {
        if (ros::Time::now().toSec() - currentFrame.foundBallTime > 6) //
        {
            this->isBallSeen = false;
        }
    }

    this->ballBearing = static_cast<double>(received_ball.ball_bearing/M_PI*180);
    if (this->headAnglePitch == 65)
    this->ballRange = static_cast<double>(received_ball.ball_range-0.23);  //as agreed with 2D
    else
        this->ballRange = static_cast<double>(received_ball.ball_range);
    cout << "ballBearing:" << this->ballBearing << endl << "ballRange:" << this->ballRange << endl;
    this->ballLoc_world_x = static_cast<double>(received_localization.ballCenterOnField.x);
    this->ballLoc_world_y = static_cast<double>(received_localization.ballCenterOnField.y);



    //Goal
    if (currentFrame.isGoalSeen == true)
    {
        if ((ros::Time::now().toSec() - currentFrame.foundGoalTime > 6) && (robot_moved)) //
        {
            this->isGoalSeen = false;
        }
    }


    if ((currentFrame.received_goalpost.goalpost_number == 2)||(currentFrame.received_goalpost.goalpost_number == 3))
    {
        this->goalCenterBearing = (static_cast<double>(received_goalpost.goalpost_left_bearing/M_PI*180) + static_cast<double>(received_goalpost.goalpost_right_bearing/M_PI*180)) / 2;
        this->goalCenterRange = (static_cast<double>(received_goalpost.goalpost_left_range) + static_cast<double>(received_goalpost.goalpost_right_range)) / 2;
    }
    this->goalLocLeft_world_x = 4.5;
    this->goalLocLeft_world_y = -1.3;
    this->goalLocRight_world_x = 4.5;
    this->goalLocRight_world_y = 1.3;

    this->robot_goal_bearing_odom =90- atan2((4.5-_odom_x),(_odom_y-0))/M_PI*180 -_odom_orientation/M_PI*180;
    double test1= - odom_orientation/M_PI*180 - atan((_odom_y-0)/(4.5-_odom_x))/M_PI*180 ;


    ROS_INFO("Odom x:%f,y:%f,orient:%f,robot_goal_bearing_odom:%f, test1:%f",_odom_x,_odom_y,_odom_orientation,this->robot_goal_bearing_odom,
             test1);

    //Opponent
    this->isOpponentSeen = static_cast<bool>(received_obstacles.opponent_detected);
    this->opponentCenterBearing = static_cast<double>(received_obstacles.opponent_bearing/M_PI*180);
    this->opponentRange = static_cast<double>(received_obstacles.opponent_range);
    cout << "opponentBearing:" << this->opponentCenterBearing << endl << "ballRange:" << this->opponentRange << endl;
    this->opponentCenter_world_x = static_cast<double>(received_localization.opponentCenterOnField.x);
    this->opponentCenter_world_y = static_cast<double>(received_localization.opponentCenterOnField.y);

    //Robot
    this->robotLoc_x = static_cast<double>(received_localization.robotPose.x);
    this->robotLoc_y = static_cast<double>(received_localization.robotPose.y);
    this->robotLoc_theta = static_cast<double>(received_localization.robotPose.theta/M_PI*180);
    this->locConfidence = static_cast<double>(received_localization.robotPoseConfidence);

    this->gyroHead = dealWithHeadYawTheta(received_headgyro.euler_angle.z/M_PI*180);
    this->gyroBody = dealWithBodyYawTheta(_body_gyro); // xabsl only with degrees
    this->odom_orientation = _odom_orientation/M_PI*180; // xabsl only with degrees

    double a = currentFrame.goalCenterRange;
    double b = currentFrame.ballRange;
    double c = currentFrame.robot_goal_bearing/180*M_PI;
    double d = currentFrame.ballBearing/180*M_PI -currentFrame.headAngleYaw/180*M_PI;
    this->adjust_theta = - (asin(a*sin(c-d)/sqrt(pow(a,2)+pow(b,2)-2*a*b*cos(c-d))) + d)/M_PI*180; // xabsl only with degrees

    //    this->headAnglePitch = received5.angle_head_pitch; // rad
    //    this->headAngleYaw = received5.angle_head_yaw;





    //#endif
}

// Function that prints out the game state, the robot perceived state, and if the ball and goal were seen
void DataStructure::PrintReceivedData(void)
{
    console_strstream << "game state: ";

    switch (currentFrame.received_gamecontrol.state)
    {
    case 0:
        console_strstream << "UnKnown" << endl;
        break;
    case 1:
        console_strstream << "Ready" << endl;
        break;
    case 2:
        console_strstream << "Set" << endl;
        break;
    case 3:
        console_strstream << "Play" << endl;
        break;
    case 4:
        console_strstream << "Finish" << endl;
        break;
    default:
        console_strstream << "UnKnown" << endl;
        break;
    }   //////////////////////////////////////////////////match
    if(currentFrame.isReady == true)
        console_strstream << "Ready" <<endl;
    else
        console_strstream << "UnKnown" <<endl;
    if(currentFrame.isGameStart == true)
        console_strstream << "Play" <<endl;
    else
        console_strstream << "UnKnown" <<endl;
    if(currentFrame.isGameOver == true)
        console_strstream << "Finish" <<endl;
    else
        console_strstream << "UnKnown" <<endl;
    ///////////////////////////////////////////////////////////debug

    console_strstream << "time: " << timeLeft << endl;

    console_strstream <<"kickangle:  " << kickangle << endl;
    console_strstream << "isBallSeen: " << (isBallSeen ? "true" : "false") << endl;
    if (!isBallSeen)
    {
    }
    else
    {
        console_strstream << "ballRange: " << ballRange << "ballBearing: " << ballBearing << endl;
    }

    console_strstream << "isGoalSeen: " << (isGoalSeen ? "true" : "false") << endl;
    if (!isGoalSeen)
    {
    }
    else
    {
        console_strstream << "goalRange: " << goalCenterRange << "goalBearing: " << goalCenterBearing << endl;
    }

    console_strstream << "isOpponentSeen: " << (isOpponentSeen ? "true" : "false") << endl;
    if (!isOpponentSeen)
    {
    }
    else
    {
        console_strstream << "opponentRange: " << opponentRange << " opponentBearing: " << opponentCenterBearing << endl;
    }

    console_strstream << "locConfidence: " << locConfidence << " robot_moved: " << robot_moved << endl;

    console_strstream << "isAttacker: " << (isAttacker ? "true" : " false") << "kick_angle: " << kickangle << endl;

    console_strstream << "is_robot_moving: " << (is_robot_moving ? "true" : "false") << "sec_state: " << sec_state << endl;


    console_strstream << "target_range: " << target_range << " target_bearing: " << target_bearing << " target_theta: " << target_theta << endl;
    console_strstream << "gyroBody: " << gyroBody << " odom_orientation: " << odom_orientation << " adjust_theta: " << adjust_theta << endl;

    console_strstream << "ballFoundHeadAngleYaw: " << ballFoundHeadAngleYaw << " goalFoundHeadAngleYaw: " << goalFoundHeadAngleYaw << endl;
    console_strstream << "cannot_kick: " << cannot_kick << endl;

}


//Function that corrects the head gyro angle and just calculates the difference from the initial yaw angle
double dealWithHeadYawTheta(double inputYawTheta)
{
    if (isInitializing)
    {
        return inputYawTheta;
    }
    else
    {
        double result;
        result = inputYawTheta - currentFrame.gyroInitHeadYawTheta;
        if (result > 180.0)
        {
            result -= 360.0;
        }
        else
        {
            if (result <= -180.0)
            {
                result += 360.0;
            }
        }
        return result;
    }
}

//Function that corrects the body gyro angle and just calculates the difference from the initial yaw angle
double dealWithBodyYawTheta(double inputYawTheta)
{
    if (isInitializing)
    {
        return inputYawTheta;
    }
    else
    {
        double result;
        result = inputYawTheta - currentFrame.gyroInitBodyYawTheta;
        if (result > 180.0)
        {
            result -= 360.0;
        }
        else
        {
            if (result <= -180.0)
            {
                result += 360.0;
            }
        }
        return result;
    }
}

DataStructure currentFrame;
DataStructure lastFrame;
double gyroInitYawTheta;


