#include "master.h"

master::master(){
    // attach to robot interface
    robot.reset(new RobotInterface);
}

void master::Run(){
    // Ground Sensor Measurements under this threshold are black
    // measurements above this threshold can be considered white.
    // TODO: Fill this in with a reasonable threshold that separates "line detected" from "no line detected"
    double GROUND_SENSOR_THRESHOLD = 0.0;

    // These are your pose values that you will update by solving the odometry equations
    double pose_x = 0;
    double pose_y = 0;
    double pose_theta = 0;

    // Index into ground_sensors and ground_sensor_readings for each of the 3 onboard sensors.
    int LEFT_IDX = 2;
    int CENTER_IDX = 1;
    int RIGHT_IDX = 0;

    // ePuck Constants
    double EPUCK_AXLE_DIAMETER = 0.053; // ePuck's wheels are 53mm apart.
    double EPUCK_MAX_WHEEL_SPEED = 0; // TODO: To be filled in with ePuck wheel speed in m/s
    double MAX_SPEED = 6.28;

    // Initialize Motors
    robot->SetLeftMotorSpeed(0.0);
    robot->SetRightMotorSpeed(0.0);

    // Allow sensors to properly initialize
    for (int ii = 0; ii < 10; ii++){
        robot->StepSim();
    }

    double vL = 0; // TODO: Initialize variable for left speed
    double vR = 0; // TODO: Initialize variable for right speed

    // vector to store ground sensor values
    vector<double> gsens;

    // Main Control Loop:
    while (robot->StepSim() != -1) {
        // Read ground sensor values
        robot->getGroundSensors(gsens);

        //TODO: Uncomment to see the ground sensor values!
        //for(int ii=0;ii<gsens.size();ii++){
        //    cout << gsens[ii] << ",";
        //}
        //cout << endl;
        

        // Hints: 
        //
        // 1) Setting vL=MAX_SPEED and vR=-MAX_SPEED lets the robot turn
        // right on the spot. vL=MAX_SPEED and vR=0.5*MAX_SPEED lets the
        // robot drive a right curve.
        //
        // 2) If your robot "overshoots", turn slower.
        //
        // 3) Only set the wheel speeds once so that you can use the speed
        // that you calculated in your odometry calculation.
        //
        // 4) Disable all console output to simulate the robot superfast
        // and test the robustness of your approach.
        //
        // TODO: Insert Line Following Code Here                
        
        
        
        // TODO: Call update_odometry Here
        
        // Hints:
        //
        // 1) Divide vL/vR by MAX_SPEED to normalize, then multiply with
        // the robot's maximum speed in meters per second. 
        //
        // 2) SIM_TIMESTEP tells you the elapsed time per step. You need
        // to divide by 1000.0 to convert it to seconds
        //
        // 3) Do simple sanity checks. In the beginning, only one value
        // changes. Once you do a right turn, this value should be constant.
        //
        // 4) Focus on getting things generally right first, then worry
        // about calculating odometry in the world coordinate system of the
        // Webots simulator first (x points down, y points right)

        

        
        // TODO: Insert Loop Closure Code Here
        
        // Hints:
        //
        // 1) Set a flag whenever you encounter the line
        //
        // 2) Use the pose when you encounter the line last 
        // for best results
        
        cout << "Current Pose: [" << pose_x << ", " << pose_y << ", " << pose_theta << "]" << endl;
        robot->SetLeftMotorSpeed(vL);
        robot->SetRightMotorSpeed(vR);
    }
}