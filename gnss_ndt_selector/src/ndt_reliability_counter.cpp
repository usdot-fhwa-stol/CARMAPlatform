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

#include "ndt_reliability_counter.h"

namespace localizer
{
    NDTReliabilityCounter::NDTReliabilityCounter() : NDTReliabilityCounter(2.0, 3) {}
    
    NDTReliabilityCounter::NDTReliabilityCounter(double score_upper_limit, int message_upper_limit) : 
                                                score_upper_limit_(score_upper_limit),
                                                unreliable_message_upper_limit_(message_upper_limit) {}

    void NDTReliabilityCounter::onNDTScore(float score)
    {
        // check smaller than 0 to handle overflow
        if(score > score_upper_limit_ || score < 0.0)
		{
			// only increase counter when it is smaller than or equal to the upper limit to avoid overflow
			if(ndt_reliability_counter <= unreliable_message_upper_limit_ * 2)
			{
				ndt_reliability_counter++;
			}
		} else
		{
			if(ndt_reliability_counter != 0)
            {
                --ndt_reliability_counter;
            }
		}
    }

    int NDTReliabilityCounter::getNDTReliabilityCounter()
    {
        return ndt_reliability_counter;
    }


}