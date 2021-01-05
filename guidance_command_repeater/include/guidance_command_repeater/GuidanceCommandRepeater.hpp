/*
 * Copyright (C) 2018-2021 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include <mutex>
// ROS
#include <ros/ros.h>
#include <sstream>
#include <std_srvs/Trigger.h>
#include <std_msgs/Float32.h>
// CAV
#include <cav_msgs/SpeedAccel.h>
#include <cav_msgs/LateralControl.h>
#include <cav_msgs/RobotEnabled.h>

namespace guidance_command_repeater {

/*!
 * Main class for the node to handle the ROS interfacing.
 */

class GuidanceCommandRepeater {
    public:
        /*!
        * Constructor.
        * @param nodeHandle the ROS node handle.
        */
        GuidanceCommandRepeater(ros::NodeHandle& nodeHandle);

        /*!
        * Destructor.
        */
        virtual ~GuidanceCommandRepeater();
        
        // Calls SpeedAccelPublisher and WrenchEffortPublisher and LateralControlPublisher
        void publisher();

        // runs publish at a desired frequency
        int rate;

    private:
        /*!
        * Reads and verifies the ROS parameters.
        * @return true if successful.
        */
        bool readParameters();

        //! ROS node handle.
        ros::NodeHandle& nodeHandle_;

        //! ROS speedAccel subscriber.
        ros::Subscriber SpeedAccelSubscriber_;

        /*!
        * ROS topic callback method.
        * @param message the received message.
        */
        void SpeedAccelSubscriberCallback(const cav_msgs::SpeedAccel::ConstPtr& msg);



        //! ROS wrenchEffort subscriber.
        ros::Subscriber WrenchEffortSubscriber_;

        /*!
        * ROS topic callback method.
        * @param message the received message.
        */
        void WrenchEffortSubscriberCallback(const std_msgs::Float32::ConstPtr& msg);



        //! ROS lateralControl subscriber.
        ros::Subscriber LateralControlSubscriber_;

        /*!
        * ROS topic callback method.
        * @param message the received message.
        */
        void LateralControlSubscriberCallback(const cav_msgs::LateralControl::ConstPtr& msg);


        //! ROS topic publishers.
        ros::Publisher SpeedAccelPublisher_;
        std::string SpeedAccelPublisherTopic_;

        /*!
        * ROS speedAccel publisher method.
        * @param message the received message.
        */
        void SpeedAccelPublisher();


        ros::Publisher WrenchEffortPublisher_;
        std::string WrenchEffortPublisherTopic_;

        /*!
        * ROS wrenchEffort publisher method.
        * @param message the received message.
        */
        void WrenchEffortPublisher();

        
        ros::Publisher LateralControlPublisher_;
        std::string LateralControlPublisherTopic_;

        /*!
        * ROS LateralControlPublisher method.
        * @param message the received message.
        */
        void LateralControlPublisher();


        // Messages used to transfer data from subscribers to publishers
        cav_msgs::SpeedAccel::ConstPtr SpeedAccelMsg;
        std_msgs::Float32::ConstPtr WrenchEffortMsg;
        cav_msgs::LateralControl::ConstPtr LateralControlMsg;

        // Locks used to make the data transfer from subscribers to publishers thread safe
        std::mutex SpeedAccelMsgMutex;
        std::mutex WrenchEffortMsgMutex;
        std::mutex LateralControlMsgMutex;

        // Time used to check the timestamp for the last time subscribers reciaved message
        double SpeedAccelTimeTracker;
        double WrenchEffortTimeTracker;
        double LateralControlTimeTracker;

        ros::Duration TimeoutThresh;
        double timeout;

        void InitTimeTracker();

};

}  // namespace guidance_command_repeater