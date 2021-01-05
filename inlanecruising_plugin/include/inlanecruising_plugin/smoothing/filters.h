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
#include <deque>
namespace inlanecruising_plugin
{
namespace smoothing
{
/**
 * \brief Extremely simplie moving average filter
 * 
 * \param input The points to be filtered
 * \param window_size The number of points to use in the moving window for averaging
 * 
 * \return The filterted points
 */
std::vector<double> moving_average_filter(const std::vector<double> input, int window_size)
{
  int i = 0;
  double average;
  std::deque<double> samples;
  std::vector<double> output;
  output.reserve(input.size());
  for (auto value : input)
  {
    if (i < window_size)
    {
      samples.push_back(value);
    }
    else
    {
      samples.pop_front();
      samples.push_back(value);
    }

    double total = 0;
    for (auto s : samples)
    {
      total += s;
    }

    double average = total / samples.size();
    output.push_back(average);
    i++;
  }

  return output;
}
};  // namespace smoothing
};  // namespace inlanecruising_plugin