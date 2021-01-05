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

namespace mock_drivers
{
// this verstion of the ROSComms object is used for publishers and subscribers
template <typename T>
ROSComms<T>::ROSComms(){

};

/*! \brief Constructor for a publisher ROSComms */
template <typename T>
ROSComms<T>::ROSComms(CommTypes ct, bool latch, int qs, std::string t)
{
  comm_type_ = ct;
  latch_ = latch;
  queue_size_ = qs;
  topic_ = t;
};

/*! \brief Constructor for a subscriber ROSComms */
template <typename T>
ROSComms<T>::ROSComms(std::function<void(T)> cbf, CommTypes ct, bool latch, int qs, std::string t)
{
  callback_function_ = cbf;
  comm_type_ = ct;
  latch_ = latch;
  queue_size_ = qs;
  topic_ = t;
};

/*! \brief Used to get an object of the message type that can then be decltyped */
template <typename M>
M ROSComms<M>::getTemplateType()
{
  M element;
  return element;
}

/*! \brief Used to call the ROSComms callback function*/
template <typename T>
void ROSComms<T>::callback(T msg)
{
  // call the callback_function_ function
  this->callback_function_(msg);
}

/*! \brief This version of the ROSComms object is used for services*/
template <typename M, typename T>
ROSComms<M, T>::ROSComms(std::function<bool(M, T)> cbf, CommTypes ct, std::string t)
{
  callback_function_ = cbf;
  comm_type_ = ct;
  topic_ = t;
}

/*! \brief Used to call the ROSComms callback function*/
template <typename M, typename T>
bool ROSComms<M, T>::callback(M req, T res)
{
  // call the callback_function_ function
  this->callback_function_(req, res);
}

/*! \brief Used to get an object of the service response type that can then be decltyped */
template <typename M, typename T>
T ROSComms<M, T>::getResType()
{
  T element;
  return element;
}

/*! \brief Used to get an object of the service request type that can then be decltyped */
template <typename M, typename T>
M ROSComms<M, T>::getReqType()
{
  M element;
  return element;
}

}  // namespace mock_drivers