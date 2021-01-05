
/*------------------------------------------------------------------------------
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

------------------------------------------------------------------------------*/

#include <inlanecruising_plugin/smoothing/CubicSpline.h>

namespace inlanecruising_plugin
{
namespace smoothing
{
void CubicSpline::setPoints(std::vector<lanelet::BasicPoint2d> points)
{
  std::vector<double> x;
  std::vector<double> y;
  for (const auto& p : points)
  {
    x.push_back(p.x());
    y.push_back(p.y());
  }
  spline_.set_points(x, y, true);
}
double CubicSpline::operator()(double x) const
{
  return spline_(x);
}
};  // namespace smoothing
};  // namespace inlanecruising_plugin