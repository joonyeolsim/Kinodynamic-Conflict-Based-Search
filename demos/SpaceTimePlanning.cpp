/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2021, Technische Universität Berlin (TU Berlin)
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
*   * Neither the name of the TU Berlin nor the names of its
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

/* Author: Francesco Grothe */

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SpaceTimeStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/STRRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include "common.h"

#include <ompl/config.h>
#include <iostream>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <fstream>

using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;

/**
 * Demonstration of planning through space time using the Animation state space and the SpaceTimeRRT* planner.
 */
double robotRadius;
vector<double> spaceLimit;
vector<shared_ptr<Obstacle>> obstacles;
vector<vector<vector<double>>> dynamicObstacles;
vector<double> maxTimes;

// bool isCollideAgent(double agent_x, double agent_y, double agent_r, double center_x, double center_y, double width, double height) {
//     // 직사각형의 영역을 구한다
//     double rectLeft = center_x - width/2;
//     double rectRight = center_x + width/2;
//     double rectTop = center_y - height/2;
//     double rectBottom = center_y + height/2;
//
//     // 원의 중심이 사각형 내부에 있는 경우, 충돌한다
//     if (agent_x > rectLeft && agent_x < rectRight && agent_y > rectTop && agent_y < rectBottom) {
//         return true;
//     }
//
//     // 원의 중심이 사각형 바깥에 있을 때, 사각형의 가장 가까운 경계와 원의 중심 사이의 거리를 계산한다
//     double closestX = (agent_x < rectLeft) ? rectLeft : (agent_x > rectRight) ? rectRight : agent_x;
//     double closestY = (agent_y < rectTop) ? rectTop : (agent_y > rectBottom) ? rectBottom : agent_y;
//
//     double distX = agent_x - closestX;
//     double distY = agent_y - closestY;
//
//     // 거리와 원의 반지름을 비교한다
//     return (distX * distX + distY * distY) <= (agent_r * agent_r);
// }

bool isStateValid(const ob::State *state)
{
    // extract the space component of the state and cast it to what we expect
    const auto x = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
    const auto y = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1];

    // extract the time component of the state and cast it to what we expect
    const auto t = state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;

    // check if it is in area
    if (x - robotRadius < 0 || x + robotRadius > spaceLimit[0] || y - robotRadius < 0 || y + robotRadius > spaceLimit[1])
        return false;

    // check if it is in obstacle
    for (const auto& obstacle : obstacles){
        if (obstacle->constrained(make_tuple(x, y), robotRadius)) return false;
    }

    // return a value that is always true
    return t >= 0 && x < std::numeric_limits<double>::infinity() && y < std::numeric_limits<double>::infinity();
}

class SpaceTimeMotionValidator : public ob::MotionValidator {

public:
    explicit SpaceTimeMotionValidator(const ob::SpaceInformationPtr &si) : MotionValidator(si),
      vMax_(si_->getStateSpace().get()->as<ob::SpaceTimeStateSpace>()->getVMax()),
      stateSpace_(si_->getStateSpace().get()) {};

    bool checkMotion(const ob::State *s1, const ob::State *s2) const override
    {
        // assume motion starts in a valid configuration, so s1 is valid
        if (!si_->isValid(s2)) {
            invalid_++;
            return false;
        }

        // check if motion is forward in time and is not exceeding the speed limit
        auto *space = stateSpace_->as<ob::SpaceTimeStateSpace>();
        auto deltaPos = space->distanceSpace(s1, s2);
        auto deltaT = s2->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position -
                      s1->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;

        if (!(deltaT > 0 && deltaPos <= vMax_)) {
            invalid_++;
            return false;
        }

        // check if the path between the states is unconstrained (perform interpolation).
        // extract the space component of the state and cast it to what we expect
        auto x1 = s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
        auto y1 = s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1];
        auto t1 = s1->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;

        auto x2 = s2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
        auto y2 = s2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1];
        auto t2 = s2->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;

        double distance = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
        int numSteps = static_cast<int>(std::ceil(distance / robotRadius));

        for (int i = 0; i <= numSteps; i++){
            double t = i / static_cast<double>(numSteps);
            double x = x1 * (1 - t) + x2 * t;
            double y = y1 * (1 - t) + y2 * t;
            // check if it is in dynamic path
            for (int j = 0; j < dynamicObstacles.size(); j++){
                // check if it is greater than the max time
                if (t > maxTimes[j]){
                    double dist = sqrt(pow(dynamicObstacles[j].back()[0] - x, 2) + pow(dynamicObstacles[j].back()[1] - y, 2));
                    if (dist <= (robotRadius + robotRadius)){
                        invalid_++;
                        return false;
                    }
                }
                else{
                    for (auto oState : dynamicObstacles[j]){
                        // check if it is in the same time
                        if (t - deltaT <= oState[2] && t + deltaT >= oState[2]){
                            double dist = sqrt(pow(oState[0] - x, 2) + pow(oState[1] - y, 2));
                            if (dist <= (robotRadius + robotRadius)){
                                invalid_++;
                                return false;
                            }
                        }
                    }
                }
            }
        }
        return true;
    }

    bool checkMotion(const ompl::base::State *, const ompl::base::State *,
                     std::pair<ob::State *, double> &) const override
    {
        throw ompl::Exception("SpaceTimeMotionValidator::checkMotion", "not implemented");
    }

private:
    double vMax_; // maximum velocity
    ob::StateSpace *stateSpace_; // the animation state space for distance calculation
};

void plan(string mapname, string obs, string robotnum, string testnum)
{
    string benchmarkPath = "benchmark/" + mapname + "_" + obs + "/agents" + robotnum + "/" + mapname + "_" + obs + "_" +
                       robotnum + "_" + testnum + ".yaml";
    string solutionPath = "solution/" + mapname + "_" + obs + "/agents" + robotnum + "/" + mapname + "_" + obs + "_" +
                          robotnum + "_" + testnum + "_solution.txt";
    string dataPath = "data/" + mapname + "_" + obs + "/agents" + robotnum + "/" + mapname + "_" + obs + "_" + robotnum +
                      "_" + testnum + "_data.txt";

    YAML::Node config = YAML::LoadFile(benchmarkPath);

    auto robotNum = config["agentNum"].as<int>();
    auto startPoints = config["startPoints"].as<vector<vector<double>>>();
    auto goalPoints = config["goalPoints"].as<vector<vector<double>>>();
    auto dimension = 2;

    for (size_t i = 0; i < config["obstacles"].size(); ++i) {
        if (mapname == "CircleEnv") {
            auto center = config["obstacles"][i]["center"].as<std::vector<double>>();
            auto radius = config["obstacles"][i]["radius"].as<double>();
            obstacles.emplace_back(make_shared<CircularObstacle>(center[0], center[1], radius));
        } else {
            auto center = config["obstacles"][i]["center"].as<std::vector<double>>();
            auto height = config["obstacles"][i]["height"].as<double>();
            auto width = config["obstacles"][i]["width"].as<double>();
            obstacles.emplace_back(make_shared<RectangularObstacle>(center[0], center[1], width, height));
        }
    }

    vector<Point> start_points;
    vector<Point> goal_points;
    for (size_t i = 0; i < config["startPoints"].size(); ++i) {
        auto start = config["startPoints"][i].as<std::vector<double>>();
        auto goal = config["goalPoints"][i].as<std::vector<double>>();
        start_points.emplace_back(start[0], start[1]);
        goal_points.emplace_back(goal[0], goal[1]);
    }

    robotRadius = 0.5;
    spaceLimit = {40, 40};

    auto start_time = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < robotNum; i++){
        // set maximum velocity
        double vMax = 5.0;

        // construct the state space we are planning in
        auto vectorSpace(std::make_shared<ob::RealVectorStateSpace>(dimension));
        auto space = std::make_shared<ob::SpaceTimeStateSpace>(vectorSpace, vMax);

        // set the bounds for R1
        ob::RealVectorBounds bounds(dimension);
        bounds.setLow(0.0);
        bounds.setHigh(0, spaceLimit[0]);
        bounds.setHigh(1, spaceLimit[1]);
        vectorSpace->setBounds(bounds);

        // set time bounds. Planning with unbounded time is also possible when using ST-RRT*.
        // space->setTimeBounds(0.0, 10.0);

        // create the space information class for the space
        ob::SpaceInformationPtr si = std::make_shared<ob::SpaceInformation>(space);

        // set state validity checking for this space
        si->setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });
        si->setMotionValidator(std::make_shared<SpaceTimeMotionValidator>(si));

        // define a simple setup class
        og::SimpleSetup ss(si);

        // create a start state
        ob::ScopedState<> start(space);
        start[0] = startPoints[i][0]; // pos
        start[1] = startPoints[i][1]; // pos

        // create a goal state
        ob::ScopedState<> goal(space);
        goal[0] = goalPoints[i][0]; // pos
        goal[1] = goalPoints[i][1]; // pos

        // set the start and goal states
        ss.setStartAndGoalStates(start, goal);

        // construct the planner
        auto *strrtStar = new og::STRRTstar(si);

        // set planner parameters
        strrtStar->setRange(vMax);

        // set the used planner
        ss.setPlanner(ob::PlannerPtr(strrtStar));

        // attempt to solve the problem within one second of planning time
        ob::PlannerStatus solved = ss.solve(300.0);

        if (solved)
        {
            std::cout << "Found solution for agent " + to_string(i) << std::endl;
            // print the path to screen
            auto solution = ss.getSolutionPath();
            dynamicObstacles.emplace_back();
            for (auto &state : solution.getStates()){
                const auto x = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
                const auto y = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1];
                const auto t = state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
                dynamicObstacles.back().push_back({x, y, t});
            }
            maxTimes.push_back(dynamicObstacles.back().back()[2]);
        }
        else
            std::cout << "No solution found for agent " + to_string(i) << std::endl;

    }
    bool okayFlag = true;
    if (dynamicObstacles.size() == robotNum){
        for (int k = 0; k < robotNum; k++){
            auto goalState = dynamicObstacles[k].back();
            for (int l = 0; l < robotNum; l++){
                if (k != l){
                    for (const auto& jState : dynamicObstacles[l]){
                        if (jState[2] > goalState[2])
                        {
                            double dist = sqrt(pow(goalState[0] - jState[0], 2) + pow(goalState[1] - jState[1], 2));
                            if (dist < (robotRadius + robotRadius)){
                                cout << goalState[0] << " " << goalState[1] << " " << jState[0] << " " << jState[1] << endl;
                                cout << "Agent " + to_string(k) + " and agent " + to_string(l) + " collide at goal" << endl;
                                okayFlag = false;
                            }
                        }
                    }
                }
            }
        }

        if (okayFlag){
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
            double executionTime = (double) duration / 1e+9;

            // Save Solution in YAML format
            ofstream file(solutionPath, ios::out);
            if (!file.is_open()) {
                cerr << "Error opening file: " << solutionPath << endl;
                return;
            }
            vector<double> sumOfSpaceDistances;
            vector<double> sumOfTimeDistances;
            for (int i = 0; i < robotNum; i++){
                auto path = dynamicObstacles[i];
                double spaceDistance = 0;
                double last_x = numeric_limits<double>::max();
                double last_y = numeric_limits<double>::max();
                file << "Agent " << i << ":";
                for (const auto &pState : path) {
                    auto x = pState[0];
                    auto y = pState[1];
                    auto time = pState[2];
                    file << "(" << x << "," << y << "," << time << ")->";
                    if (last_x != numeric_limits<double>::max() && last_y != numeric_limits<double>::max())
                        spaceDistance += sqrt(pow(x - last_x, 2) + pow(y - last_y, 2));
                    last_x = x;
                    last_y = y;
                }
                file << endl;
                sumOfSpaceDistances.push_back(spaceDistance);
                sumOfTimeDistances.push_back(path.back()[2]);
            }
            file.close();

            // Save Sum of Space and Time Distances in CSV format
            std::ofstream dataOut(dataPath);

            // Assuming executionTime is some variable that holds the execution time
            dataOut << std::accumulate(sumOfSpaceDistances.begin(), sumOfSpaceDistances.end(), 0.0) << ","
                    << std::accumulate(sumOfTimeDistances.begin(), sumOfTimeDistances.end(), 0.0) << ","
                    << *std::max_element(sumOfSpaceDistances.begin(), sumOfSpaceDistances.end()) << ","
                    << *std::max_element(sumOfTimeDistances.begin(), sumOfTimeDistances.end()) << ","
                    << executionTime << ",";
            dataOut.close();

            std::cout << "ST-RRT PP Found solution!" << std::endl;
        }
    }
}

int main(int argc, char* argv[])
{
    string mapname;
    string obs;
    string robotnum;
    string testnum;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-m") == 0 && i + 1 < argc) {
            mapname = argv[i + 1];
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            obs = argv[i + 1];
        } else if (strcmp(argv[i], "-r") == 0 && i + 1 < argc) {
            robotnum = argv[i + 1];
        } else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
            testnum = argv[i + 1];
        }
    }

    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    plan(mapname, obs, robotnum, testnum);

    return 0;
}