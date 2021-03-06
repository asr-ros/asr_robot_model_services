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

#pragma once

#include "robot_model_services/rating/IKRatingModule.h"
#include "robot_model_services/robot_model/RobotModel.hpp"
#include "robot_model_services/helper/DebugHelper.hpp"

namespace robot_model_services {
    /*!
     * \brief NavigationPathIKRatingModule implements the functionlities offered by IKRatingModule.
     * \author Florian Aumann
     * \date 2015
     * \version 1.0
     * \copyright GNU Public License
     * \sa NavigationPathIKRatingModule
     */
    class NavigationPathIKRatingModule : public IKRatingModule {
    public:
        /*!
         * \brief constructor of the NavigationPathIKRatingModule object
         */
        NavigationPathIKRatingModule(RobotModelPtr robotModel);

        /*!
         * \brief destructor of the NavigationPathIKRatingModule object.
         */
        virtual ~NavigationPathIKRatingModule();

        /*!
         * \brief
         * \param sourcePosition [in] the MILD's current position
         * \param targetPosition [in] the MILD's target position
         * \param sourceRotationBase [in] the MILD's current rotation
         * \param targetRotationBase [in] the MILD's target rotation
         * \return the rating angle (between 0.0 and 1.0)
         */
        double getPanAngleRating(const geometry_msgs::Point &sourcePosition, const geometry_msgs::Point &targetPosition, double sourceRotationBase, double targetRotationBase);
    private:
        RobotModelPtr mRobotModel;
        DebugHelperPtr mDebugHelperPtr;
    };

    /*!
     * \brief Definition for the shared pointer type of the class.
     */
    typedef boost::shared_ptr<NavigationPathIKRatingModule> NavigationPathIKRatingModulePtr;
}
