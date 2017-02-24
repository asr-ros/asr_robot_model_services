/**

Copyright (c) 2016, Aumann Florian, Borella Jocelyn, Heller Florian, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Stroh Daniel, Trautmann Jeremias, Walter Milena, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

4. The use is explicitly not permitted to any application which deliberately try to kill or do harm to any living creature.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "robot_model_services/robot_model/impl/MILDRobotModelWithExactIK.hpp"
#include "robot_model_services/robot_model/impl/MILDRobotModelWithApproximatedIK.hpp"
#include "robot_model_services/robot_model/impl/MILDRobotModel.hpp"
#include "robot_model_services/robot_model/impl/MILDRobotState.hpp"
#include "robot_model_services/robot_model/RobotState.hpp"
#include <ros/ros.h>
#include "robot_model_services/typedef.hpp"
#include "asr_robot_model_services/CalculateCameraPose.h"
#include "asr_robot_model_services/CalculateCameraPoseCorrection.h"
#include "asr_robot_model_services/IsPositionAllowed.h"
#include "asr_robot_model_services/RobotStateMessage.h"
#include "asr_robot_model_services/GetPose.h"
#include "asr_robot_model_services/GetDistance.h"
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>

using namespace robot_model_services;
//Since not all functionalities are implemented in the robot model with approximated IK,
//two pointers are used for the basic functionalies implemented all child classes and the advanced functionalities only implmented in the RobotModelWithExactIK class
MILDRobotModelPtr basicFunctionRobotModelPtr;
MILDRobotModelWithExactIKPtr advancedFunctionRobotModelPtr;

bool getDistance(asr_robot_model_services::GetDistance::Request &req, asr_robot_model_services::GetDistance::Response &res)
{
      res.distance = basicFunctionRobotModelPtr->getDistance(req.sourcePosition, req.targetPosition);
      return true;
}

bool processCalculateCameraPoseServiceCall(asr_robot_model_services::CalculateCameraPose::Request &req, asr_robot_model_services::CalculateCameraPose::Response &res)
{
    MILDRobotState * sourceRobotState = new MILDRobotState(req.sourceRobotState.pan, req.sourceRobotState.tilt,req.sourceRobotState.rotation,req.sourceRobotState.x,req.sourceRobotState.y);
    RobotStatePtr sourceRobotStatePtr(sourceRobotState);
    res.cameraFrame = basicFunctionRobotModelPtr->calculateCameraPose(sourceRobotStatePtr);
    return true;
}

bool processIsPositionAllowedServiceCall(asr_robot_model_services::IsPositionAllowed::Request &req, asr_robot_model_services::IsPositionAllowed::Response &res)
{
  res.isAllowed = basicFunctionRobotModelPtr->isPositionAllowed(req.targetPosition);
  return true;
}

bool processGetRobotPoseServiceCall(asr_robot_model_services::GetPose::Request &req, asr_robot_model_services::GetPose::Response &res)
{
  res.pose = basicFunctionRobotModelPtr->getRobotPose();
  return true;
}

bool processGetCameraPoseServiceCall(asr_robot_model_services::GetPose::Request &req, asr_robot_model_services::GetPose::Response &res)
{
  res.pose = basicFunctionRobotModelPtr->getCameraPose();
  return true;
}

bool processCalculateCameraPoseCorrectionServiceCall(asr_robot_model_services::CalculateCameraPoseCorrection::Request &req, asr_robot_model_services::CalculateCameraPoseCorrection::Response &res)
{
  MILDRobotState * sourceRobotState = new MILDRobotState(req.sourceRobotState.pan, req.sourceRobotState.tilt,req.sourceRobotState.rotation,req.sourceRobotState.x,req.sourceRobotState.y);
  RobotStatePtr sourceRobotStatePtr(sourceRobotState);
  SimpleVector3 position(req.position.x, req.position.y, req.position.z);
  SimpleQuaternion orientation(req.orientation.w, req.orientation.x, req.orientation.y, req.orientation.z);
  PTUConfig resultConfig = advancedFunctionRobotModelPtr->calculateCameraPoseCorrection(sourceRobotStatePtr, position, orientation);
  res.pan = std::get<0>(resultConfig);
  res.tilt = std::get<1>(resultConfig);
  return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "getMovementCosts");
    ros::NodeHandle n = ros::NodeHandle(ros::this_node::getName());

    double panMin, panMax, tiltMin, tiltMax;
    int robotModelId;
    n.param("panMin", panMin, -60.);
    n.param("panMax", panMax, 60.);
    n.param("tiltMin", tiltMin, -45.);
    n.param("tiltMax", tiltMax, 45.);
    n.param("robotModelId", robotModelId, 1);
    ROS_INFO_STREAM("panMin: " << panMin);
    ROS_INFO_STREAM("panMax: " << panMax);
    ROS_INFO_STREAM("tiltMin: " << tiltMin);
    ROS_INFO_STREAM("tiltMax: " << tiltMax);

    advancedFunctionRobotModelPtr = MILDRobotModelWithExactIKPtr(new MILDRobotModelWithExactIK());
    advancedFunctionRobotModelPtr->setTiltAngleLimits(tiltMin, tiltMax);
    advancedFunctionRobotModelPtr->setPanAngleLimits(panMin, panMax);
    switch (robotModelId) {
    case 1:
        ROS_INFO_STREAM("NBV Service: Using new IK model");
        //Use the same robot model with exact IK for basic and advanced functionalities
        basicFunctionRobotModelPtr = advancedFunctionRobotModelPtr;
        break;
    case 2:
        ROS_INFO_STREAM("NBV Service:: Using old IK model");
        //Use  robot model with exact IK for advanced functionalities only, create a new robot model with approximated IK for basic functionality
        basicFunctionRobotModelPtr = MILDRobotModelWithApproximatedIKPtr(new MILDRobotModelWithApproximatedIK());
        basicFunctionRobotModelPtr->setTiltAngleLimits(tiltMin, tiltMax);
        basicFunctionRobotModelPtr->setPanAngleLimits(panMin, panMax);
        break;
    default:
        std::stringstream ss;
        ss << robotModelId << " is not a valid robot model ID";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
    ros::ServiceServer service_GetDistance = n.advertiseService("GetDistance", getDistance);
    ros::ServiceServer service_CalculateCameraPose = n.advertiseService("CalculateCameraPose", processCalculateCameraPoseServiceCall);
    ros::ServiceServer service_IsPositionAllowed = n.advertiseService("IsPositionAllowed", processIsPositionAllowedServiceCall);
    ros::ServiceServer service_GetRobotPose = n.advertiseService("GetRobotPose", processGetRobotPoseServiceCall);
    ros::ServiceServer service_GetCameraPose = n.advertiseService("GetCameraPose", processGetCameraPoseServiceCall);
    ros::ServiceServer service_CalculateCameraPoseCorrection = n.advertiseService("CalculateCameraPoseCorrection", processCalculateCameraPoseCorrectionServiceCall);

    ROS_INFO("RobotModel Service started.");
    ros::spin();

    return 0;
}
