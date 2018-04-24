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
10-Initialize motors
FLOAT[1]: step_amount for 1-4, target_x for 5, circle_radius for 6-7, kick_leg for 8
FLOAT[2]: step_length for 1-2, step_width for 3-4, target_y for 5, circle_theta for 6-7, kick_strength for 8
FLOAT[3]: target_theta for 5, kick_angle for 8
FLOAT[4]: Head yaw
FLOAT[5]: Head tilt
FLOAT[6]: Waist yaw
FLOAT[7]: Ball Range
FLOAT[8]: Ball Bearing
FLOAT[9]: Goalpost Range
FLOAT[10]: Goalpost Bearing

*/

void behavior_initialize_motors::execute()
{
    //cout << "[BEHAVIOR]:initialize motors";
    printf("[BEHAVIOR]:initialize motors");

    broadcast.OutputPacket[0] = 10;
    broadcast.OutputPacket[1] = 0;
    broadcast.OutputPacket[2] = 0;
    broadcast.OutputPacket[3] = 0;
    broadcast.OutputPacket[4] = 0;
    broadcast.OutputPacket[5] = 25; // just have head tilted to 25 degrees
    broadcast.OutputPacket[6] = 0;
    broadcast.OutputPacket[7] = 0;
    broadcast.OutputPacket[8] = 0;
    broadcast.OutputPacket[9] = 0;
    broadcast.OutputPacket[10] = 0;

}

void behavior_nothing::execute()
{
    //cout << "[BEHAVIOR]:nothing";
    printf("[BEHAVIOR]:nothing");

    broadcast.OutputPacket[0] = 9;
    broadcast.OutputPacket[1] = 0;
    broadcast.OutputPacket[2] = 0;
    broadcast.OutputPacket[3] = 0;
    //broadcast.OutputPacket[4] = 0;
    //broadcast.OutputPacket[5] = 25; // just have head tilted to 25 degrees

    currentFrame.ball_moved_while_moving = false;
    currentFrame.expected_ball_bearing = currentFrame.ballBearing ;
    currentFrame.first_time_track_ball = true;

}

void behavior_end_game::execute()
{
    //cout << "[BEHAVIOR]:end game";
    printf("[BEHAVIOR]:end game");

    broadcast.OutputPacket[0] = 9;
    broadcast.OutputPacket[1] = 0;
    broadcast.OutputPacket[2] = 0;
    broadcast.OutputPacket[3] = 0;
    broadcast.OutputPacket[4] = 0;
    broadcast.OutputPacket[5] = 25; // just have head tilted to 25 degrees

}

void behavior_initialize::execute()
{
    //cout << "[BEHAVIOR]:initialize";
    printf("[BEHAVIOR]:initialize");

    broadcast.OutputPacket[0] = 9;
    broadcast.OutputPacket[1] = 0;
    broadcast.OutputPacket[2] = 0;
    broadcast.OutputPacket[3] = 0;
    //broadcast.OutputPacket[4] = 0;
    //broadcast.OutputPacket[5] = 25; // just have head tilted to 25 degrees

    if (isInitializing)
    {
        currentFrame.gyroInitHeadYawTheta = currentFrame.received_headgyro.euler_angle.z/M_PI*180;
        currentFrame.gyroInitBodyYawTheta = -broadcast.InputPacket[3];
    }

    isInitializing = false;
}


void behavior_center_round_clockwise::execute()
{
    //cout << "[BEHAVIOR]:turn around a center clockwise";
    printf("[BEHAVIOR]:turn around a center clockwise");

    broadcast.OutputPacket[0] = 6;
    broadcast.OutputPacket[1] = 0.1;  // circle_radius
    broadcast.OutputPacket[2] = currentFrame.target_theta / 180 * M_PI;  // circle_theta in rad
    broadcast.OutputPacket[3] = 0;
    //broadcast.OutputPacket[4] = 0;
    //broadcast.OutputPacket[5] = 25; // just have head tilted to 25 degrees

    currentFrame.robot_moved = true;

}

void behavior_center_round_anticlockwise::execute()
{
    //cout << "[BEHAVIOR]:turn around a center anti-clockwise";
    printf("[BEHAVIOR]:turn around a center anti-clockwise");

    broadcast.OutputPacket[0] = 7;
    broadcast.OutputPacket[1] = 0.1;  // circle_radius
    broadcast.OutputPacket[2] = currentFrame.target_theta / 180 * M_PI;  // circle_theta in rad
    broadcast.OutputPacket[3] = 0;
    //broadcast.OutputPacket[4] = 0;
    //broadcast.OutputPacket[5] = 25; // just have head tilted to 25 degrees

    currentFrame.robot_moved = true;
}

void behavior_round_clockwise::execute()
{
    //cout << "[BEHAVIOR]:turn around clockwise";
    printf("[BEHAVIOR]:turn around clockwise");

    broadcast.OutputPacket[0] = 6;
    broadcast.OutputPacket[1] = 0.2;  // circle_radius
    broadcast.OutputPacket[2] = currentFrame.target_theta / 180 * M_PI;  // circle_theta in rad
    broadcast.OutputPacket[3] = 0;
    //broadcast.OutputPacket[4] = 0;
    //broadcast.OutputPacket[5] = 25; // just have head tilted to 25 degrees
    currentFrame.robot_moved = true;

}


void behavior_round_anticlockwise::execute()
{
    //cout << "[BEHAVIOR]:turn around anti-clockwise";
    printf("[BEHAVIOR]:turn around anti-clockwise");

    broadcast.OutputPacket[0] = 7;
    broadcast.OutputPacket[1] = 0.2;  // circle_radius
    broadcast.OutputPacket[2] = currentFrame.target_theta / 180 * M_PI;  // circle_theta in rad
    broadcast.OutputPacket[3] = 0;
    //broadcast.OutputPacket[4] = 0;
    //broadcast.OutputPacket[5] = 25; // just have head tilted to 25 degrees
    currentFrame.robot_moved = true;

}


void behavior_walk::execute()
{
    //cout << "[BEHAVIOR]:walk to a specific target range and bearing";
    printf("[BEHAVIOR]:walk to a specific target range and bearing");

    //    if ((!currentFrame.is_robot_moving_ball_flag) && (currentFrame.is_robot_moving))
    //    {
    //        currentFrame.is_robot_moving_ball_flag = true;
    //        currentFrame.gyro_body_initial = currentFrame.gyroBody;
    //    }

    // Calculate target_x and target_y from target_range and target_bearing
    broadcast.OutputPacket[0] = 5;

    printf("current target range:%f, target bearing:%f, target theta:%f",
           currentFrame.target_range,currentFrame.target_bearing,currentFrame.target_theta/ 180 * M_PI);

    if ( currentFrame.target_range == -1.0 )  //target for attacker prepare
    {
//        broadcast.OutputPacket[1] = 0.5;
//        broadcast.OutputPacket[2] = -0.5;
//        broadcast.OutputPacket[3] = 0;

        broadcast.OutputPacket[1] = 0.5;
        broadcast.OutputPacket[2] = 0.5;
        broadcast.OutputPacket[3] = 0;
    }
    else if ( currentFrame.target_range == -2.0 )  //target for defender prepare
    {
        broadcast.OutputPacket[1] = 3;
        broadcast.OutputPacket[2] = -0.2;
        broadcast.OutputPacket[3] = -1.05; //60 /180 *M_PI;
    }
    else if ( currentFrame.target_range == -3.0 )  //target for attacker after first kick
    {
        broadcast.OutputPacket[1] = 1;
        broadcast.OutputPacket[2] = 0.75;
        broadcast.OutputPacket[3] = -0.05;
    }
    else
    {
        broadcast.OutputPacket[1] = currentFrame.target_range * cos(currentFrame.target_bearing/ 180 * M_PI-currentFrame.ball_found_head_angle_yaw/ 180 * M_PI);
        broadcast.OutputPacket[2] = - currentFrame.target_range * sin(currentFrame.target_bearing/ 180 * M_PI-currentFrame.ball_found_head_angle_yaw/ 180 * M_PI); // target_y is negative to the right

        if (broadcast.OutputPacket[1]<=0)
            broadcast.OutputPacket[1] = 0;

        if ((fabs(broadcast.OutputPacket[1])<=0.3) && (fabs(broadcast.OutputPacket[2]<=0.3)))
        {
            broadcast.OutputPacket[0] = 9;
            broadcast.OutputPacket[1] = 0;
            broadcast.OutputPacket[2] = 0;
            broadcast.OutputPacket[3] = 0;
        }

        broadcast.OutputPacket[3] = - currentFrame.target_theta/ 180 * M_PI;
        if (broadcast.OutputPacket[3] == 0)
        {
            broadcast.OutputPacket[3] = -0.02;
        }
    }

    //broadcast.OutputPacket[4] = 0;
    //broadcast.OutputPacket[5] = 25; // just have head tilted to 25 degrees
    currentFrame.robot_moved = true;

}

void behavior_step_forward::execute()
{
    //cout << "[BEHAVIOR]:take a step forward of distance ";
    printf("[BEHAVIOR]:take a step forward of distance ");

    broadcast.OutputPacket[0] = 1;
    broadcast.OutputPacket[1] = currentFrame.target_range / STEP_LENGTH;  // step_number
    broadcast.OutputPacket[2] = STEP_LENGTH;  //step_length
    broadcast.OutputPacket[3] = 0;
    //broadcast.OutputPacket[4] = 0;
    //broadcast.OutputPacket[5] = 25; // just have head tilted to 25 degrees
    currentFrame.robot_moved = true;

}

void behavior_step_backward::execute()
{
    //cout << "[BEHAVIOR]:take a step backward of distance ";
    printf("[BEHAVIOR]:take a step backward of distance ");

    broadcast.OutputPacket[0] = 2;
    broadcast.OutputPacket[1] = currentFrame.target_range / STEP_LENGTH;  // step_number
    broadcast.OutputPacket[2] = STEP_LENGTH;  // step_length
    broadcast.OutputPacket[3] = 0;
    //broadcast.OutputPacket[4] = 0;
    //broadcast.OutputPacket[5] = 25; // just have head tilted to 25 degrees
    currentFrame.robot_moved = true;

}

void behavior_step_left::execute()
{
    //cout << "[BEHAVIOR]:take a step left of distance ";
    printf("[BEHAVIOR]:take a step left of distance ");

    broadcast.OutputPacket[0] = 3;
    broadcast.OutputPacket[1] = currentFrame.target_range / STEP_WIDTH;  // step_number
    broadcast.OutputPacket[2] = STEP_WIDTH;  //step_width
    broadcast.OutputPacket[3] = 0;
    //broadcast.OutputPacket[4] = 0;
    //broadcast.OutputPacket[5] = 25; // just have head tilted to 25 degrees
    currentFrame.robot_moved = true;

}

void behavior_step_right::execute()
{
    //cout << "[BEHAVIOR]:take a step right of distance ";
    printf("[BEHAVIOR]:take a step right of distance ");

    broadcast.OutputPacket[0] = 4;
    broadcast.OutputPacket[1] = 5;  // step_number
    broadcast.OutputPacket[2] = 0.1;  //step_width
    broadcast.OutputPacket[3] = 0;
    //broadcast.OutputPacket[4] = 0;
    //broadcast.OutputPacket[5] = 25; // just have head tilted to 25 degrees
    currentFrame.robot_moved = true;

}

void behavior_kick_ball_soft::execute()
{
    //cout << "[BEHAVIOR]:kick the ball with low strength ";
    printf("[BEHAVIOR]:kick the ball with low strength ");

    broadcast.OutputPacket[0] = 8;
    broadcast.OutputPacket[1] = currentFrame.kick_leg;  //kick_leg, 0=right, 1=left
    broadcast.OutputPacket[2] = 0;  //kick_speed, 0=soft, 1=mid, 2=hard
    broadcast.OutputPacket[3] = currentFrame.kickangle;  //kick_angle
    broadcast.OutputPacket[4] = 0;
    broadcast.OutputPacket[5] = 25; // just have head tilted to 25 degrees
}


void behavior_kick_ball_mid::execute()
{
    //cout << "[BEHAVIOR]:kick the ball with medium strength ";
    printf("[BEHAVIOR]:kick the ball with medium strength  ");

    broadcast.OutputPacket[0] = 8;
    broadcast.OutputPacket[1] = currentFrame.kick_leg;  //kick_leg, 0=right, 1=left
    broadcast.OutputPacket[2] = 1;  //kick_speed, 0=soft, 1=mid, 2=hard
    broadcast.OutputPacket[3] = currentFrame.kickangle;  //kick_angle
    broadcast.OutputPacket[4] = 0;
    broadcast.OutputPacket[5] = 25; // just have head tilted to 25 degrees
}

void behavior_kick_ball_strong::execute()
{
    //cout << "[BEHAVIOR]:kick the ball with high strength ";
    printf("[BEHAVIOR]:kick the ball with high strength ");

    broadcast.OutputPacket[0] = 8;
    broadcast.OutputPacket[1] = currentFrame.kick_leg;  //kick_leg, 0=right, 1=left
    broadcast.OutputPacket[2] = 2;  //kick_speed, 0=soft, 1=mid, 2=hard
    broadcast.OutputPacket[3] = currentFrame.kickangle;  //kick_angle
    broadcast.OutputPacket[4] = 0;
    broadcast.OutputPacket[5] = 25; // just have head tilted to 25 degrees
}

