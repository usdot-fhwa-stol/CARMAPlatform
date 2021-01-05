/*
 * Copyright (C) 2019-2021 LEIDOS.
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


#include "bsm_generator.h"

namespace bsm_generator
{
    BSMGenerator::BSMGenerator() : bsm_generation_frequency_(10.0) {}

    void BSMGenerator::initialize()
    {
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        pnh_->param<double>("bsm_generation_frequency", bsm_generation_frequency_, 10.0);
        nh_->param<double>("vehicle_length", vehicle_length_, 5.0);
        nh_->param<double>("vehicle_width", vehicle_width_, 2.0);
        bsm_pub_ = nh_->advertise<cav_msgs::BSM>("bsm_outbound", 5);
        timer_ = nh_->createTimer(ros::Duration(1.0 / bsm_generation_frequency_), &BSMGenerator::generateBSM, this);
        gear_sub_ = nh_->subscribe("transmission_state", 1, &BSMGenerator::gearCallback, this);
        speed_sub_ = nh_->subscribe("vehicle_speed", 1, &BSMGenerator::speedCallback, this);
        steer_wheel_angle_sub_ = nh_->subscribe("steering_wheel_angle", 1, &BSMGenerator::steerWheelAngleCallback, this);
        accel_sub_ = nh_->subscribe("velocity_accel", 1, &BSMGenerator::accelCallback, this);
        yaw_sub_ = nh_->subscribe("yaw_rate_rpt", 1, &BSMGenerator::yawCallback, this);
        brake_sub_ = nh_->subscribe("brake_position", 1, &BSMGenerator::brakeCallback, this);
        pose_sub_ = nh_->subscribe("pose", 1, &BSMGenerator::poseCallback, this);
        heading_sub_ = nh_->subscribe("dual_antenna_heading", 1, &BSMGenerator::headingCallback, this);
        tf2_listener_.reset(new tf2_ros::TransformListener(tf2_buffer_));
    }

    void BSMGenerator::initializeBSM()
    {
        bsm_.core_data.presence_vector = 0;
        bsm_.core_data.size.vehicle_width = vehicle_width_;
        bsm_.core_data.size.vehicle_length = vehicle_length_;
        bsm_.core_data.size.presence_vector = bsm_.core_data.size.presence_vector | bsm_.core_data.size.VEHICLE_LENGTH_AVAILABLE;
        bsm_.core_data.size.presence_vector = bsm_.core_data.size.presence_vector | bsm_.core_data.size.VEHICLE_WIDTH_AVAILABLE;
    }

    void BSMGenerator::run()
    {
        initialize();
        initializeBSM();
        ros::CARMANodeHandle::spin();
    }

    void BSMGenerator::speedCallback(const std_msgs::Float64ConstPtr& msg)
    {
        bsm_.core_data.speed = worker.getSpeedInRange(msg->data);
        bsm_.core_data.presence_vector = bsm_.core_data.presence_vector | bsm_.core_data.SPEED_AVAILABLE;
    }

    void BSMGenerator::gearCallback(const j2735_msgs::TransmissionStateConstPtr& msg)
    {
        bsm_.core_data.transmission.transmission_state = msg->transmission_state;
    }

    void BSMGenerator::steerWheelAngleCallback(const std_msgs::Float64ConstPtr& msg)
    {
        bsm_.core_data.angle = worker.getSteerWheelAngleInRnage(msg->data);
        bsm_.core_data.presence_vector = bsm_.core_data.presence_vector | bsm_.core_data.STEER_WHEEL_ANGLE_AVAILABLE;
    }

    void BSMGenerator::accelCallback(const automotive_platform_msgs::VelocityAccelConstPtr& msg)
    {
        bsm_.core_data.accelSet.longitudinal = worker.getLongAccelInRange(msg->accleration);
        bsm_.core_data.accelSet.presence_vector = bsm_.core_data.accelSet.presence_vector | bsm_.core_data.accelSet.ACCELERATION_AVAILABLE;
    }

    void BSMGenerator::yawCallback(const pacmod_msgs::YawRateRptConstPtr& msg)
    {
        bsm_.core_data.accelSet.yaw_rate = worker.getYawRateInRange(msg->yaw_rate);
        bsm_.core_data.accelSet.presence_vector = bsm_.core_data.accelSet.presence_vector | bsm_.core_data.accelSet.YAWRATE_AVAILABLE;
    }

    void BSMGenerator::brakeCallback(const std_msgs::Float64ConstPtr& msg)
    {
        bsm_.core_data.brakes.wheelBrakes.brake_applied_status = worker.getBrakeAppliedStatus(msg->data);
    }

    void BSMGenerator::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        // Use pose message as an indicator of new location updates
        try
        {
            geometry_msgs::TransformStamped tf = tf2_buffer_.lookupTransform("earth", "host_vehicle", ros::Time(0));
            tf2::Vector3 loc(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
            // TODO ecef_to_geodesic function is not in the library yet
            wgs84_utils::wgs84_coordinate coord =  wgs84_utils::ecef_to_geodesic(loc);
            bsm_.core_data.longitude = coord.lon;
            bsm_.core_data.latitude = coord.lat;
            bsm_.core_data.elev = coord.elevation;
            bsm_.core_data.presence_vector = bsm_.core_data.presence_vector | bsm_.core_data.LONGITUDE_AVAILABLE;
            bsm_.core_data.presence_vector = bsm_.core_data.presence_vector | bsm_.core_data.LATITUDE_AVAILABLE;
            bsm_.core_data.presence_vector = bsm_.core_data.presence_vector | bsm_.core_data.ELEVATION_AVAILABLE;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }
    }

    void BSMGenerator::headingCallback(const novatel_gps_msgs::NovatelDualAntennaHeadingConstPtr& msg)
    {
        bsm_.core_data.heading = worker.getHeadingInRange(msg->heading);
    }

    void BSMGenerator::generateBSM(const ros::TimerEvent& event)
    {
        bsm_.header.stamp = ros::Time::now();
        bsm_.core_data.msg_count = worker.getNextMsgCount();
        bsm_.core_data.id = worker.getMsgId(ros::Time::now());
        bsm_.core_data.sec_mark = worker.getSecMark(ros::Time::now());
        bsm_.core_data.presence_vector = bsm_.core_data.presence_vector | bsm_.core_data.SEC_MARK_AVAILABLE;
        // currently the accuracy is not available because ndt_matching does not provide accuracy measurement
        bsm_.core_data.accuracy.presence_vector = 0;
        bsm_pub_.publish(bsm_);
    }
}
