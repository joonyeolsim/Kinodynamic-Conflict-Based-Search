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

#include <ompl/multirobot/base/SpaceInformation.h>
#include <ompl/multirobot/base/ProblemDefinition.h>
#include <ompl/multirobot/geometric/planners/pp/PP.h>

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/ScopedState.h>

#include <yaml-cpp/yaml.h>
#include <ompl/config.h>
#include <iostream>
#include <fstream>
#include <utility>
#include <chrono>

using namespace std;
namespace omrb = ompl::multirobot::base;
namespace omrg = ompl::multirobot::geometric;
namespace ob = ompl::base;
namespace og = ompl::geometric;


/* 
When performing Multi-Robot Motion Planning, it is often the case that 
robots are treated as "dynamic obstacles" by other robots (e.g. Prioritized Planning and Kinodynamic Conflict-Based Search). 
Thus, we have extended OMPL to account for dynamic obstacles. To do so, one must implement an additional method called 
``areStatesValid" from the StateValidityChecker class. The method should return true if state1 and state2 are not in collision.
state2 is a pair consisting of the SpaceInformation and state of the ``other" robot. The SpaceInformation object is included 
so that heterogeneous robots can be properly accounted for. The example code below shows how to do this. Keep in mind that 
time-dependence is handled generically within OMPL. Please see ob::StateValidityChecker::isValid(const State *state, const double time) 
for details.
*/
class MyDemoStateValidityChecker: public ob::StateValidityChecker
{
public:
    MyDemoStateValidityChecker(ob::SpaceInformationPtr &si): ob::StateValidityChecker(si)
    {
    }

    // Answers the question: is the robot described by `si_` at `state` valid?
    bool isValid(const ompl::base::State *state) const override
    {
        // cast the abstract state type to the type we expect
        const auto *se3state = state->as<ob::SE2StateSpace::StateType>();

        // extract the first component of the state and cast it to what we expect
        const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

        // extract the second component of the state and cast it to what we expect
        const auto *rot = se3state->as<ob::SO2StateSpace::StateType>(1);

        // one must code required logic to figure out if state at pos & rot is valid.

        // this returns a value that is always true but uses the two variables we define, so we avoid compiler warnings
        return (const void*)rot != (const void*)pos;
        return true;
    }

    // Answers the question: does the robot described by `si_` at `state1` avoid collision with some other robot described by a different `si` located at `state2`?
    bool areStatesValid(const ompl::base::State* state1, const std::pair<const ompl::base::SpaceInformationPtr,const ompl::base::State*> state2) const override
    {
        // one can get the names of robots via these commands
        std::string robot1 = si_->getStateSpace()->getName();
        std::string robot2 = state2.first->getStateSpace()->getName();

        // one can get the states of robots via these commands
        auto robot1_state = state1;
        auto robot2_state = state2.second;

        // one must code required logic to figure out if robot1 at state1 collides with robot2 at state2

        // this returns a value that always true but avoids compiler warnings
        return (const void*)robot1_state != (const void*)robot2_state;
    }
};

void plan()
{
    string fileName = "../../benchmark/OpenEnv/OpenEnv_5_0.yaml";
    YAML::Node config = YAML::LoadFile(fileName);

    auto robotNum = config["robotNum"].as<int>();
    auto startPoints = config["startPoints"].as<vector<vector<double>>>();
    auto goalPoints = config["goalPoints"].as<vector<vector<double>>>();
    auto dimension = config["dimension"].as<int>();
    auto spaceLimit = config["spaceLimit"].as<vector<double>>();
    auto robotRadii = config["robotRadii"].as<vector<double>>();
    auto lambdaFactor = config["lambdaFactor"].as<double>();
    auto maxVelocity = config["maxVelocity"].as<double>();
    auto maxExpandDistance = config["maxExpandDistance"].as<double>();
    auto maxIteration = config["maxIteration"].as<int>();
    auto rectangleObstacles = config["rectangleObstacles"].as<vector<vector<double>>>();

    // construct an instance of multi-robot space information
    auto ma_si(std::make_shared<omrb::SpaceInformation>());
    auto ma_pdef(std::make_shared<omrb::ProblemDefinition>(ma_si));
    ma_pdef->unlock();
    // construct four individuals that operate in SE3
    for (int i = 0; i < robotNum; i++)
    {
        // construct the state space we are planning in
        auto space(std::make_shared<ob::SE2StateSpace>());

        // set the bounds for the R^3 part of SE(3)
        ob::RealVectorBounds bounds(dimension);
        bounds.setLow(0);
        for (int j = 0; j < dimension; j++)
            bounds.setHigh(j,spaceLimit[j]);

        space->setBounds(bounds);

        // construct an instance of  space information from this state space
        auto si(std::make_shared<ob::SpaceInformation>(space));

        // set state validity checking for this space
        si->setStateValidityChecker(std::make_shared<MyDemoStateValidityChecker>(si));

        // name the state space parameter (not required but helpful for robot-to-robot collision checking)
        si->getStateSpace()->setName("Robot " + std::to_string(i));

        // set up the space information
        si->setup();

        // add the individual information to the multi-robot SpaceInformation
        ma_si->addIndividual(si);

        // create a random start state for individual
        ob::ScopedState<ob::SE2StateSpace> start(space);
        for (int j = 0; j < dimension; j++){
            start->as<ob::RealVectorStateSpace::StateType>(0)->values[j] = startPoints[i][j];
        }

        // create a random goal state for individual
        ob::ScopedState<ob::SE2StateSpace> goal(space);
        for (int j = 0; j < dimension; j++){
            goal->as<ob::RealVectorStateSpace::StateType>(0)->values[j] = goalPoints[i][j];
        }

        // create a problem definition for individual
        auto pdef(std::make_shared<ob::ProblemDefinition>(si));

        // set the start and goal states for individual
        pdef->setStartAndGoalStates(start, goal);

        // add the individual information to the multi-robot ProblemDefinition
        ma_pdef->addIndividual(pdef);  
    }

    // lock the multi-robot SpaceInformation and ProblemDefinitions when done adding individuals
    ma_si->lock();
    ma_pdef->lock();

    // plan for all agents using a prioritized planner (PP)
    auto planner = std::make_shared<omrg::PP>(ma_si);
    planner->setProblemDefinition(ma_pdef); // be sure to set the problem definition

    auto start = std::chrono::high_resolution_clock::now();
    bool solved = planner->as<omrb::Planner>()->solve(1.0);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    double executionTime = (double) duration / 1e+9;

    if (solved)
    {
        std::ofstream fout("../../solutions/OpenEnv_5_0_solution.yaml");
        // Save Solution in YAML format
        YAML::Emitter out;
        out << YAML::BeginSeq;
        auto solution = ma_pdef->getSolutionPlan()->as<omrg::PlanGeometric>()->getPaths();
        for (auto& path : solution) {
            out << YAML::BeginSeq;
            int time = 0;
            for (auto &state : path->getStates()) {
                auto x = state->as<ob::SE2StateSpace::StateType>()->getX();
                auto y = state->as<ob::SE2StateSpace::StateType>()->getY();
                out << YAML::Flow << YAML::BeginSeq << x << y << time << YAML::EndSeq;
                time++;
            }
            out << YAML::EndSeq;
        }
        out << YAML::EndSeq;
        fout << out.c_str() << endl;
        std::cout << "Found solution!" << std::endl;
    }
}

int main(int /*argc*/, char ** /*argv*/)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
    plan();

    return 0;
}
