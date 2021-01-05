#pragma once

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

#include <vector>
#include <carma_wm/Geometry.h>

namespace inlanecruising_plugin
{
namespace smoothing
{
/**
 * \brief Interface to a spline interpolator that can be used to smoothly interpolate between points
 */ 
class SplineI
{
public:
  /**
   * @brief Virtual destructor to ensure delete safety for pointers to implementing classes
   *
   */
  virtual ~SplineI(){};

  /**
   * \brief Set key points which the spline will interpolate between
   * 
   * \param points The key points
   */ 
  virtual void setPoints(std::vector<lanelet::BasicPoint2d> points) = 0;

  /**
   * \brief Get the y value for the given x value
   * 
   * \param x The value to solve the spline for
   * 
   * \return The y value that matches x
   */ 
  virtual double operator()(double x) const = 0;
};
};  // namespace smoothing
};  // namespace inlanecruising_plugin