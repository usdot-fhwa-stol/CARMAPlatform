#pragma once

/*
 * Copyright (C) 2019-2020 LEIDOS.
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
#include <algorithm>
#include <stdexcept>
namespace basic_autonomy
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
 */// NEW
std::vector<double> moving_average_filter(const std::vector<double> input, int window_size, bool ignore_first_point=true)
{
  if (window_size % 2 == 0) {
    throw std::invalid_argument("moving_average_filter window size must be odd");
  }

  std::vector<double> output;
  output.reserve(input.size());

  if (input.size() == 0) {
    return output;
  }

  int start_index = 0;
  if (ignore_first_point) {
    start_index = 1;
    output.push_back(input[0]);
  }

  for (int i = start_index; i<input.size(); i++) {
    
    
    double total = 0;
    int sample_min = std::max(0, i - window_size / 2);
    int sample_max = std::min((int) input.size() - 1 , i + window_size / 2);

    int count = sample_max - sample_min + 1;
    std::vector<double> sample;
    sample.reserve(count);
    for (int j = sample_min; j <= sample_max; j++) {
      total += input[j];
    }
    output.push_back(total / (double) count);

  }

  return output;
}

};  // namespace smoothing
};  // namespace basic_autonomy