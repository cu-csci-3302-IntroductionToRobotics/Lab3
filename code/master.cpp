#include "master.h"

master::master(){
    // attach to robot interface
    robot.reset(new RobotInterface);
}

void master::Run(){
    // TODO: Fill out with correct values from Robot Spec Sheet (or inspect PROTO definition for the robot)
    double MAX_SPEED = 0.0001; // [rad/s]
    double MAX_SPEED_MS = 0.0001; // [m/s]
    double AXLE_LENGTH = 0.0001; // [m]



    int MOTOR_LEFT = 0; // Left wheel index
    int MOTOR_RIGHT = 1; // Right wheel index

    // These are your pose values that you will update by solving the odometry equations
    double pose_x = 0;
    double pose_y = 0;
    double pose_theta = 0;


    // Rotational Motor Velocity [rad/s]
    double vL = 0;
    double vR = 0;

    // TODO
    // Create you state and goals (waypoints) variable here
    // You have to MANUALLY figure out the waypoints, one sample is provided for you in the instructions

    while (robot->StepSim() != -1) {
        // STEP 2.1: Calculate sources of error
        
        // STEP 2.2: Feedback Controller
        
        // STEP 1: Inverse Kinematics Equations (vL and vR as a function dX and dTheta)
        // Note that vL and vR in code is phi_l and phi_r on the slides/lecture
        
        
        // STEP 2.3: Proportional velocities
        vL = 0; // Left wheel velocity in rad/s
        vR = 0; // Right wheel velocity in rad/s
        
        // STEP 2.4: Clamp wheel speeds
        

        
        // TODO
        // Use Your Lab 2 Odometry code after these 2 comments. We will supply you with our code next week 
        // after the Lab 2 deadline but you free to use your own code if you are sure about its correctness
        
        // NOTE that the odometry should ONLY be a function of 
        // (vL, vR, MAX_SPEED, MAX_SPEED_MS, timestep, AXLE_LENGTH, pose_x, pose_y, pose_theta)
        // Odometry code. Don't change speeds (vL and vR) after this line
        
        
        

        //////////////////// End Odometry Code ////////////////////////////////////
        
        //////////////////// Do not change ////////////////////////////////////////////
        // Bound pose_theta between [-pi, 2pi+pi/2]
        // Important to not allow big fluctuations between timesteps (e.g., going from -pi to pi)
        if(pose_theta > 6.28+3.14/2) 
            pose_theta -= 6.28;
        if(pose_theta < -3.14)
            pose_theta += 6.28;
        //////////////////////////////////////////////////////////////////////////////////////////////

        // TODO
        // Set robot motors to the desired velocities
        robot->SetLeftMotorSpeed(0.0);
        robot->SetRightMotorSpeed(0.0);
    }
}