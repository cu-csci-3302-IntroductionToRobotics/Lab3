#ifndef ROBOTINTERFACE_H__
#define ROBOTINTERFACE_H__

#include <iostream>
#include <webots/Camera.hpp>
#include <webots/Motor.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <memory>

using namespace webots;
using namespace std;

class RobotInterface : public Supervisor {
public:
  RobotInterface() {

    simulationReset();

    //create supervisor and get the robot node
    robot_node = getFromDef("MY_ROBOT");

    timeStep = (int)getBasicTimeStep(); // set the control time step
    // get device tags from webots
    left_wheel = getMotor("left wheel motor");
    right_wheel = getMotor("right wheel motor");


  }

  void SetLeftMotorSpeed(double speed){
    left_wheel->setPosition(INFINITY);
    left_wheel->setVelocity(speed);
  }

  void SetRightMotorSpeed(double speed){
    right_wheel->setPosition(INFINITY);
    right_wheel->setVelocity(speed);
  }

  int StepSim() {
    return step(timeStep);
  }

private:
  int timeStep;
  Motor* left_wheel;
  Motor* right_wheel;
  Node* robot_node;
};

#endif // ROBOTINTERFACE_H__