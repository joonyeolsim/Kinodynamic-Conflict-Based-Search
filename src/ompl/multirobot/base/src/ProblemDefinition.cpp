/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Justin Kottinger */

#include "ompl/multirobot/base/ProblemDefinition.h"
#include "ompl/base/goals/GoalState.h"


ompl::multirobot::base::ProblemDefinition::ProblemDefinition(SpaceInformationPtr si): 
    si_(std::move(si)), solutions_(std::make_shared<PlannerSolutionSet>())
{
}

void ompl::multirobot::base::ProblemDefinition::setStartAndGoalStatesAtIndex(unsigned int index, const ompl::base::State *start, 
        const ompl::base::State *goal, double threshold)
{
    clearStartStatesAtIndex(index);
    addStartStateAtIndex(index, start);
    setGoalStateAtIndex(index, goal, threshold);
}

void ompl::multirobot::base::ProblemDefinition::setGoalStateAtIndex(unsigned int index, const ompl::base::State *goal, double threshold)
{
    clearGoalAtIndex(index);
    auto gs(std::make_shared<ompl::base::GoalState>(si_->getIndividual(index)));
    gs->setState(goal);
    gs->setThreshold(threshold);
    setGoalAtIndex(index, gs);
}
