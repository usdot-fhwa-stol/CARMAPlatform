#ifndef NDT_RELIABILITY_COUNTER_H
#define NDT_RELIABILITY_COUNTER_H

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

namespace localizer
{
    class NDTReliabilityCounter
    {
        public:

            NDTReliabilityCounter();
            NDTReliabilityCounter(double score_upper_limit, int message_upper_limit);

            // Create public getter function for a private variable
            int getNDTReliabilityCounter();

            // function to handle new ndt score
            void onNDTScore(float score);

        private:
            
            // counter used to measure ndt matching reliability
            int ndt_reliability_counter {0};
            // if above this number, this ndt msg is not reliable
            double score_upper_limit_;
            // if receiving this number of continuous unreliable score, current ndt matching result is not reliable
            int unreliable_message_upper_limit_;
    };
}

#endif // NDT_RELIABILITY_COUNTER_H