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

#define BOOST_TEST_STATIC_LINK

#include <ros/ros.h>
#include <boost/array.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <glpk.h>
#include <map>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <dynamic_reconfigure/Reconfigure.h>

#include "asr_msgs/AsrAttributedPoint.h"

#include "robot_model_services/helper/MathHelper.hpp"
#include "robot_model_services/helper/MapHelper.hpp"
#include "robot_model_services/helper/TypeHelper.hpp"
#include "robot_model_services/robot_model/impl/MILDRobotModel.hpp"
#include "robot_model_services/robot_model/impl/MILDRobotModelWithExactIK.hpp"
#include "robot_model_services/robot_model/impl/MILDRobotState.hpp"
#include <tf/tf.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ros/master.h>

using namespace robot_model_services;

class BaseTest {
protected:
    boost::shared_ptr<ros::NodeHandle> mNodeHandle;
    ros::Publisher mInitPosePub;
    bool silent;
public:
    BaseTest();

    BaseTest(bool silent);

    ~BaseTest();

    void init(bool silent);

    void initRosServices();


    robot_model_services::MILDRobotStatePtr getRobotState(const geometry_msgs::Pose &initialPose);

    void waitForEnter();

    SimpleQuaternion euler2Quaternion( const Precision roll, const Precision pitch, const Precision yaw);

    SimpleQuaternion ZXZ2Quaternion( const Precision roll, const Precision pitch, const Precision yaw);

    template<typename T> bool getParameter(const std::string &key, T &parameter)
    {
        if (!mNodeHandle->getParam(key, parameter))
        {
            ROS_ERROR_STREAM(key << " parameter not set!");
            return false;
        }
        else
        {
            return true;
        }
    }
};

