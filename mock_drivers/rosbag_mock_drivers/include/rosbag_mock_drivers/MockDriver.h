/*
 * Copyright (C) 2020-2021 LEIDOS.
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
#pragma once

#include <ros/ros.h>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <cav_msgs/DriverStatus.h>

#include "rosbag_mock_drivers/ROSComms.h"
#include "rosbag_mock_drivers/MockDriverNode.h"
#include "rosbag_mock_drivers/comm_types.h"
#include "rosbag_mock_drivers/DriverType.h"

namespace mock_drivers
{
/*! \brief The template node for the mock drivers that will handle all of the default driver logic
 *
 * This class has virtual functions that define what to do when the mock driver is run.
 *
 * It will also have build in default mock driver publishers and subscribers baked in, such
 * as the driver discovery publisher.
 */

class MockDriver
{
protected:
  // Helper typedefs for child classes
  template <class T>
  using ConstPtr = boost::shared_ptr<const T>;

  template <class T>
  using ConstPtrRef = const ConstPtr<T>&;

  template <class T>
  using ROSCommsPtr = boost::shared_ptr<ROSComms<T>>;

  template <class T>
  using ConstPtrRefROSComms = ROSComms<ConstPtrRef<T>>;

  template <class T>
  using ConstPtrRefROSCommsPtr = ROSCommsPtr<ConstPtrRef<T>>;

  // Member variables
  MockDriverNode mock_driver_node_;  // Node to use as pub/sub interface

  // Prefix which child classes can apply to their topics to identify bag subscriptions. This is meant to be used when
  // we are subscribing using the addPassthrough functions
  const std::string bag_prefix_ = "/bag/hardware_interface/";

  const std::string driver_discovery_topic_ = "driver_discovery";

  ROSCommsPtr<cav_msgs::DriverStatus> driver_discovery_pub_ptr_ =
      boost::make_shared<ROSComms<cav_msgs::DriverStatus>>(CommTypes::pub, false, 10, driver_discovery_topic_);

  ros::Time last_discovery_pub_ = ros::Time(0);

  /**
   * \brief Virtual method which child classes can override to add functionality which will occur during each spin loop.
   *        This means this function will be nominally called at the rate specified by getRate()
   *
   * \return Return false if this node should shutdown. True otherwise.
   */
  virtual bool onSpin();

  /**
   * \brief Pure virtual method which must be implemented by child classes. This method will be run once at startup.
   *        Child classes should add pub/sub initialization in this method implementation.
   *
   * \return An integer error code. A non-zero value will indicate an initialization failure and result in node
   * shutdown.
   */
  virtual int onRun() = 0;

  /**
   * \brief Helper function to publish the driver discovery message.
   */
  void driverDiscovery();

  /*! \brief Returns the mock driver node for the mock driver (used for testing) */
  MockDriverNode getMockDriverNode() const;

  /**
   * \brief Function adds both a publisher and subscriber of the specified type.
   *        This means a passthrough subscription has been created.
   *        This version of the method should be used for message types which contain a Header at the field "header".
   *        The header field will be automatically updated to the current ros::Time::now().
   *
   * \tparam T ROS message type which the provided topic name applies to.
   *         Should be the base type only such as sensor_msgs/NavSatFix
   *
   * \param sub_topic The topic name to subscribe to.
   * \param pub_topic The topic name to publish to.
   * \param latch Flag idicating if the output publisher should be latched.
   * \param queue_size The size of the queue to use both for the publisher and subscriber
   *
   * TODO: This function can be combined with addPassthroughPubNoHeader using C++ 17's constexpr if statements.
   *
   */
  template <typename T>
  void addPassthroughPub(const std::string& sub_topic, const std::string& pub_topic, bool latch, size_t queue_size);

  /**
   * \brief Function adds both a publisher and subscriber of the specified type.
   *        This means a passthrough subscription has been created.
   *        This version of the method should be used for message types which does NOT contain a Header at the field
   * "header"
   *
   * \tparam T ROS message type which the provided topic name applies to.
   *         Should be the base type only such as sensor_msgs/NavSatFix
   *
   * \param sub_topic The topic name to subscribe to.
   * \param pub_topic The topic name to publish to.
   * \param latch Flag idicating if the output publisher should be latched.
   * \param queue_size The size of the queue to use both for the publisher and subscriber
   *
   * TODO: This function can be combined with addPassthroughPub using C++ 17's constexpr if statements.
   *
   */
  template <typename T>
  void addPassthroughPubNoHeader(const std::string& sub_topic, const std::string& pub_topic, bool latch,
                                 size_t queue_size);

public:
  virtual ~MockDriver() = 0;

  /**
   *  \brief A function to initialize the publishers and subsricers and start the node.
   *         In child classes, this function will trigger a call to onRun().
   *
   *  \return An integer error code. A non-zero value will indicate an initialization failure and result in node
   *          shutdown.
   */
  int run();

  /**
   * \brief Pure Virtual method which child classes must override that returns the list of all driver types that class
   * implements
   *
   * \return A list of DriverType that defines the interfaces the implementing class provides
   */
  virtual std::vector<DriverType> getDriverTypes() = 0;

  /**
   * \brief Pure Virtual method. Returns an integer value which corresponds to the cav_msgs/DriverStatus enum felids
   * representing the status of the driver.
   * 
   * \return Integer value which must match one of the enum fields in cav_msgs/DriverStatus
   */
  virtual uint8_t getDriverStatus() = 0;

  /**
   * \brief Pure virtual method that returns the desired operational rate of a child class.
   * 
   * \return Spin rate in Hz
   */ 
  virtual unsigned int getRate() = 0;

  /**
   * \brief Callback which will be triggered at the rate specified by getRate(). This callback will also trigger the onSpin() method. 
   * 
   * \return False if this node should shutdown. True otherwise.
   */ 
  bool spinCallback();
};

}  // namespace mock_drivers

#include "impl/MockDriver.tpp" // Include for templated function implementations