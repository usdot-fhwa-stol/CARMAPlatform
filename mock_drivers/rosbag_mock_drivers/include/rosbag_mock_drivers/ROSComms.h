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

#include "comm_types.h"
#include "std_msgs/String.h"

namespace mock_drivers
{
/*! \brief This class is used to transfer all the information required to initialize ros topics/services
 *
 * The template type is the type of the message that is being published/subscribed to, all the other parameters just
 * need to be filled in as desired
 * 
 * NOTE: The usefulness of this abstraction is not immediately clear. Future work may consider removing it entirely or restructuing it to better support unit testing.
 */
template <typename...>
class ROSComms;

template <typename T>
/**
 * \brief Class which serves as an abstraction of a pub/sub framework.
 * 
 * \tparam T Message type. This should be a base message type and does not support services. For example t = std_msgs/Float64 
 */
class ROSComms<T>
{
private:
  std::function<void(T)> callback_function_;
  CommTypes comm_type_;
  bool latch_;
  int queue_size_;
  std::string topic_;

public:
  /**
   * \brief Returns the latched status of this data stream
   * 
   * \return True if latched. False otherwise
   */ 
  bool getLatch()
  {
    return latch_;
  }

  /**
   * \brief Returns the data stream queue size.
   * 
   * \return Size of data stream queue
   */ 
  int getQueueSize()
  {
    return queue_size_;
  }

  /**
   * \brief Returns the name string (topic) associated with the data stream
   * 
   * \return Topic name
   */ 
  std::string getTopic()
  {
    return topic_;
  }

  /**
   * \brief returns the comms type of this object
   * 
   * \return Comms type enum
   */ 
  CommTypes getCommType()
  {
    return comm_type_;
  }

  /**
   * \brief Callback function which is triggered to pass data into this comms abstraction.
   * 
   * \param msg The message to pass
   */ 
  void callback(T msg);

  /**
   * \brief Returns an instance of the type this object is parameterized on. This is used for forwarding the data type.
   * 
   * \return Instance of T
   */ 
  T getTemplateType();

  /// \brief Default constructor
  ROSComms();
  /**
   * \brief Publisher constructor
   * 
   * \param ct The comms type which should be pub
   * \param latch The latched status of this publication
   * \param qs The queue size of this publication
   * \param t The name of the topic
   */ 
  ROSComms(CommTypes ct, bool latch, int qs, std::string t);

  /**
   * \brief Publisher constructor
   * 
   * \param ct The comms type which should be pub
   * \param latch The latched status of this publication
   * \param qs The queue size of this publication
   * \param t The name of the topic
   */ 
  ROSComms(std::function<void(T)> cbf, CommTypes ct, bool latch, int qs, std::string t);
};

/**
 * \brief Class which serves as an abstraction of a service framework.
 * 
 * \tparam M Request Type. This should be a base service request type
 * \tparam T Response Type. This should be the base service response type
 */
template <typename M, typename T>
class ROSComms<M, T>
{
private:
  std::function<void(M, T)> callback_function_;
  CommTypes comm_type_;
  std::string topic_;

public:
  /**
   * \brief Returns the name string (topic) associated with the service
   * 
   * \return Topic name
   */ 
  std::string getTopic()
  {
    return topic_;
  }
  /**
   * \brief returns the comms type of this object
   * 
   * \return Comms type enum
   */ 
  CommTypes getCommType()
  {
    return comm_type_;
  }

  /**
   * \brief Callback function which is triggered to pass data into this comms abstraction.
   * 
   * \param req The request message to pass
   * \param res The response message to pass
   * 
   * \return True if the callback was executed successfully
   */ 
  bool callback(M req, T res);

  /**
   * \brief Returns an instance of the type this object is parameterized on. This is used for forwarding the data type.
   * 
   * \return Instance of M
   */ 
  M getReqType();

  /**
   * \brief Returns an instance of the type this object is parameterized on. This is used for forwarding the data type.
   * 
   * \return Instance of T
   */ 
  T getResType();

  /// \brief Default constructor
  ROSComms();
  /**
   * \brief Service constructor
   * 
   * \param cbf Service callback function. First parameter is the service request. The second is the out parameter for the response.
   * \param ct The comms type. Should be srv
   * \param t The topic name
   */ 
  ROSComms(std::function<bool(M, T)> cbf, CommTypes ct, std::string t);
};
}  // namespace mock_drivers

#include "ROSComms.ipp"