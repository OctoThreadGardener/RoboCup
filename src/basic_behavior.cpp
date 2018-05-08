#include "basic_behavior.h"
#include "Definitions.h"

// Need to change this to send the commands over the UDP program
// No need for the service now, because walk and head are controlled
// by the second NUC. Also, the robot body orientation Imu is received
// by the second NUC, so that information will be received along with
// the odometry data.

#define STEP_LENGTH 0.15
#define STEP_WIDTH 0.08


/*

decision_serial_output_data
received_data[0]: Command Type

0- Nothing
1-Walk with speed mode
3-Walk with target point mode
4-Kick left leg
5-Kick right leg
6-Kick (Use with ball_x, ball_y)


FLOAT[1]: v_x for 1, target_x for 3, ball_x for 6
FLOAT[2]: v_y for 1, target_y for 3, ball_y for 6
FLOAT[3]: v_theta for 1, target_theta for 3
FLOAT[4]:
FLOAT[5]:
FLOAT[6]:


*/

void behavior_initialize_motors::execute()
{
    //cout << "[BEHAVIOR]:initialize motors";
    printf("[BEHAVIOR]:initialize motors");

    currentFrame.decision_serial_output_data.received_data[0] = 0;
    currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] = 0;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

}

void behavior_nothing::execute()
{
    //cout << "[BEHAVIOR]:nothing";
    printf("[BEHAVIOR]:nothing");

    currentFrame.decision_serial_output_data.received_data[0] = 0;
    currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] = 0;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    currentFrame.ball_moved_while_moving = false;
    currentFrame.expected_ball_bearing = currentFrame.ballBearing ;
    currentFrame.first_time_track_ball = true;

}

void behavior_end_game::execute()
{
    //cout << "[BEHAVIOR]:end game";
    printf("[BEHAVIOR]:end game");

    currentFrame.decision_serial_output_data.received_data[0] = 0;
    currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] = 0;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

}

void behavior_initialize::execute()
{
    //cout << "[BEHAVIOR]:initialize";
    printf("[BEHAVIOR]:initialize");

    currentFrame.decision_serial_output_data.received_data[0] = 0;
    currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] = 0;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    isInitializing = false;
}


void behavior_center_round_clockwise::execute()
{
    //cout << "[BEHAVIOR]:turn around a center clockwise";
    printf("[BEHAVIOR]:turn around a center clockwise");

    currentFrame.decision_serial_output_data.received_data[0] = 0;
    currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] = 0;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    currentFrame.robot_moved = true;

}

void behavior_center_round_anticlockwise::execute()
{
    //cout << "[BEHAVIOR]:turn around a center anti-clockwise";
    printf("[BEHAVIOR]:turn around a center anti-clockwise");

    currentFrame.decision_serial_output_data.received_data[0] = 0;
    currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] = 0;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    currentFrame.robot_moved = true;
}

void behavior_round_clockwise::execute()
{
    //cout << "[BEHAVIOR]:turn around clockwise";
    printf("[BEHAVIOR]:turn around clockwise");

    currentFrame.decision_serial_output_data.received_data[0] = 3;
    currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] = currentFrame.target_theta/180*M_PI;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    currentFrame.robot_moved = true;

}


void behavior_round_anticlockwise::execute()
{
    //cout << "[BEHAVIOR]:turn around anti-clockwise";
    printf("[BEHAVIOR]:turn around anti-clockwise");

    currentFrame.decision_serial_output_data.received_data[0] = 0;
    currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] = 0;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    currentFrame.robot_moved = true;

}

void behavior_walk::execute()
{
    //cout << "[BEHAVIOR]:walk to a specific target range and bearing";
    printf("[BEHAVIOR]:walk to a specific target range and bearing");

    currentFrame.decision_serial_output_data.received_data[0] = 1;

    currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] = 0;

    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    //currentFrame.robot_moved = true;

}

void behavior_stop_walk::execute()
{
    //cout << "[BEHAVIOR]:walk to a specific target range and bearing";
    printf("[BEHAVIOR]:walk to a specific target range and bearing");

    currentFrame.decision_serial_output_data.received_data[0] = 1;

    currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] = 0;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    //currentFrame.robot_moved = true;

}

void behavior_step_forward::execute()
{
    //cout << "[BEHAVIOR]:take a step forward of distance ";
    printf("[BEHAVIOR]:take a step forward of distance ");

    currentFrame.decision_serial_output_data.received_data[0] = 0;
    currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] = 0;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    currentFrame.robot_moved = true;

}

void behavior_step_backward::execute()
{
    //cout << "[BEHAVIOR]:take a step backward of distance ";
    printf("[BEHAVIOR]:take a step backward of distance ");

    currentFrame.decision_serial_output_data.received_data[0] = 0;
    currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] = 0;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    currentFrame.robot_moved = true;

}

void behavior_step_left::execute()
{
    //cout << "[BEHAVIOR]:take a step left of distance ";
    printf("[BEHAVIOR]:take a step left of distance ");

    currentFrame.decision_serial_output_data.received_data[0] = 0;
    currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] = 0;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    currentFrame.robot_moved = true;

}

void behavior_step_right::execute()
{
    //cout << "[BEHAVIOR]:take a step right of distance ";
    printf("[BEHAVIOR]:take a step right of distance ");

    currentFrame.decision_serial_output_data.received_data[0] = 0;
    currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] = 0;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    currentFrame.robot_moved = true;

}

void behavior_kick_ball_soft::execute()
{
    //cout << "[BEHAVIOR]:kick the ball with low strength ";
    printf("[BEHAVIOR]:kick the ball with low strength ");

    currentFrame.decision_serial_output_data.received_data[0] = 0;
    currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] = 0;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    currentFrame.robot_moved = true;
}


void behavior_kick_ball_mid::execute()
{
    //cout << "[BEHAVIOR]:kick the ball with medium strength ";
    printf("[BEHAVIOR]:kick the ball with medium strength  ");

    currentFrame.decision_serial_output_data.received_data[0] = 0;
    currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] = 0;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    currentFrame.robot_moved = true;
}

void behavior_kick_ball_strong::execute()
{
    //cout << "[BEHAVIOR]:kick the ball with high strength ";


    currentFrame.decision_serial_output_data.received_data[0] = 6;
    currentFrame.decision_serial_output_data.received_data[1] = currentFrame.ballRange * cos(currentFrame.ballBearing);
    currentFrame.decision_serial_output_data.received_data[2] = currentFrame.ballRange * sin(currentFrame.ballBearing);
    currentFrame.decision_serial_output_data.received_data[3] = 0.0; //currentFrame.ballBearing;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

//    // Debug
//    currentFrame.decision_serial_output_data.received_data[1] = 0.2;
//    currentFrame.decision_serial_output_data.received_data[2] = -0.1;
//    currentFrame.decision_serial_output_data.received_data[3] = 0;

    ROS_INFO("[BEHAVIOR]:kick the ball with high strength at (%f,%f,%f)",currentFrame.decision_serial_output_data.received_data[1],
            currentFrame.decision_serial_output_data.received_data[2],currentFrame.decision_serial_output_data.received_data[3]);

    currentFrame.robot_moved = true;
}

void behavior_approach_ball::execute()
{
    //cout << "[BEHAVIOR]:kick the ball with medium strength ";
    printf("[BEHAVIOR]:approach ball  ");


    currentFrame.decision_serial_output_data.received_data[0] = 3;
    currentFrame.decision_serial_output_data.received_data[1] = currentFrame.ballRange * cos(currentFrame.ballBearing) - 0.3;
    currentFrame.decision_serial_output_data.received_data[2] = currentFrame.ballRange * sin(currentFrame.ballBearing);
    currentFrame.decision_serial_output_data.received_data[3] = 0.0;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

//    // Debug
//    currentFrame.decision_serial_output_data.received_data[1] = 0.8;
//    currentFrame.decision_serial_output_data.received_data[2] = 0.1;

    currentFrame.robot_moved = true;

}

void behavior_approach_pose::execute()
{

    int data_counter = 0;
    currentFrame.approach_pose = true;

    /// Walk with goal pose

    currentFrame.decision_serial_output_data.received_data[0] = 3;

    // Go straight to goal
    double    global_next_goal_point_x = currentFrame.robot_goal_x;
    double     global_next_goal_point_y = currentFrame.robot_goal_y;
    double    global_robot_to_next_goal_x = global_next_goal_point_x - currentFrame.robotLoc_x;
    double    global_robot_to_next_goal_y = global_next_goal_point_y - currentFrame.robotLoc_y;

    currentFrame.decision_serial_output_data.received_data[1] = global_robot_to_next_goal_x*cos(currentFrame.robotLoc_theta) + global_robot_to_next_goal_y*sin(currentFrame.robotLoc_theta);
    currentFrame.decision_serial_output_data.received_data[2] = -global_robot_to_next_goal_x*sin(currentFrame.robotLoc_theta) + global_robot_to_next_goal_y*cos(currentFrame.robotLoc_theta);
    currentFrame.decision_serial_output_data.received_data[3] = currentFrame.robot_goal_theta - currentFrame.robotLoc_theta;

    currentFrame.near_target_pose = true;

    ROS_INFO("Trajectory planning: Sent 1 point (%f,%f,%f)",currentFrame.decision_serial_output_data.received_data[1],currentFrame.decision_serial_output_data.received_data[2],
            currentFrame.decision_serial_output_data.received_data[3]);

    data_counter = 3;

    for(int i=data_counter+1;i<currentFrame.decision_serial_output_data.received_data.size();i++)
    {
        currentFrame.decision_serial_output_data.received_data[i] = 0;
    }

}

/*
 * Additions 2018 for opt_go_to_ball
 */

void behavior_rotate_before_walk::execute()
{
    //cout << "[BEHAVIOR]:kick the ball with medium strength ";
    printf("[BEHAVIOR]:behavior_rotate_before_walk ");

    currentFrame.decision_serial_output_data.received_data[0] = 3;
    currentFrame.decision_serial_output_data.received_data[1] = 0;
    currentFrame.decision_serial_output_data.received_data[2] = 0;
    currentFrame.decision_serial_output_data.received_data[3] =  currentFrame.ballBearing;

    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;


    currentFrame.robot_moved = true;

}

void behavior_during_left_walk::execute()
{
    //cout << "[BEHAVIOR]:kick the ball with medium strength ";
    printf("[BEHAVIOR]:behavior_during_left_walk  ");

    currentFrame.decision_serial_output_data.received_data[0] = 1;
    currentFrame.decision_serial_output_data.received_data[1] = 0.2;  //v_x
    currentFrame.decision_serial_output_data.received_data[2] = 0;  //v_y
    currentFrame.decision_serial_output_data.received_data[3] =  0.2 * currentFrame.ballBearing;  //v_theta
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    currentFrame.robot_moved = true;

}

void behavior_during_right_walk::execute()
{
    //cout << "[BEHAVIOR]:kick the ball with medium strength ";
    printf("[BEHAVIOR]:behavior_during_right_walk  ");

    currentFrame.decision_serial_output_data.received_data[0] = 1;
    currentFrame.decision_serial_output_data.received_data[1] = 0.2;  //v_x
    currentFrame.decision_serial_output_data.received_data[2] = 0;  //v_y
    currentFrame.decision_serial_output_data.received_data[3] = 0.2 * currentFrame.ballBearing;  //v_theta
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

    currentFrame.robot_moved = true;

}

void behavior_rotate_after_walk::execute()
{
    //cout << "[BEHAVIOR]:kick the ball with medium strength ";
    printf("[BEHAVIOR]:kick the ball with medium strength  ");

    currentFrame.decision_serial_output_data.received_data[0] = 3;
    currentFrame.decision_serial_output_data.received_data[1] = currentFrame.ballRange * cos(currentFrame.ballBearing);
    currentFrame.decision_serial_output_data.received_data[2] = currentFrame.ballRange * sin(currentFrame.ballBearing);
    currentFrame.decision_serial_output_data.received_data[3] = 0;
    currentFrame.decision_serial_output_data.received_data[4] = 0;
    currentFrame.decision_serial_output_data.received_data[5] = 0;
    currentFrame.decision_serial_output_data.received_data[6] = 0;

//    currentFrame.decision_serial_output_data.received_data[1] = 0.8;
//    currentFrame.decision_serial_output_data.received_data[2] = 0.1;

    currentFrame.robot_moved = true;

}
