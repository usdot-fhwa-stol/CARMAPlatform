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

#ifndef __ARBITRATOR_INCLUDE_BEAM_SEARCH_STRATEGY_HPP__
#define __ARBITRATOR_INCLUDE_BEAM_SEARCH_STRATEGY_HPP__

#include "search_strategy.hpp"

namespace arbitrator 
{
    /**
     * \brief Implementation of SearchStrategy by usage of Beam Search
     * 
     * Beam Search is similar to breadth-first search with the exception that 
     * only a certain number of nodes at each step are retained to the next. This
     * is referred to as the "beam width" of the search and is applied to the
     * sorted open list at the end of every search step. This provides a reasonable
     * optimization by ensuring that not all nodes need to be expanded.
     * 
     * A beam width of n=1 is identical to naive greedy search.
     * 
     * A beam width of n=inf is identical to breadth-first search
     */
    class BeamSearchStrategy : public SearchStrategy
    {
        public:
            /**
             * \brief Constructor for BeamSearchStrategy
             * \param beam_width The number of nodes to include in the search beam during each depth of the search
             *      an infinite value would be equivalent to breadth-first search. A value of 1 is equivalent to naive
             *      greedy search.
             */
            BeamSearchStrategy(int beam_width) :
                beam_width_(beam_width) {};

            /**
             * \brief Prioritize the plans and eliminate those outside the beam width
             * \param plans The plans to evaluate as (plan, cot ) pairs
             * \return The sorted list of up to size beam_width
             */
            std::vector<std::pair<cav_msgs::ManeuverPlan, double>> prioritize_plans(std::vector<std::pair<cav_msgs::ManeuverPlan, double>> plans) const;
        private:
            int beam_width_;
    };
}

#endif //__ARBITRATOR_INCLUDE_BEAM_SEARCH_STRATEGY_HPP__