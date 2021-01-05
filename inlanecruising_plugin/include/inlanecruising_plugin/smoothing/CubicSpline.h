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
#include <inlanecruising_plugin/third_party_library/spline.h>
#include <inlanecruising_plugin/smoothing/SplineI.h>

namespace inlanecruising_plugin
{
namespace smoothing
{
/**
 * \brief Realization of SplineI that uses the tk::spline library for interpolation 
 */ 
class CubicSpline : public SplineI
{
public:
  ~CubicSpline(){};
  void setPoints(std::vector<lanelet::BasicPoint2d> points) override;
  double operator()(double x) const override;

private:
  tk::spline spline_;
};
};  // namespace smoothing
};  // namespace inlanecruising_plugin