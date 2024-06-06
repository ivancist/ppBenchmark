#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>
#include <octomap/octomap.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include "WebSocketServer.h"
#include <nlohmann/json.hpp>


namespace ob = ompl::base;
namespace og = ompl::geometric;

WebSocketServer wsServer;
octomap::OcTree tree("../octomap.bt");
DynamicEDTOctomap *distmap;
double min[3], max[3];

bool isStateValid(const ob::State *state) {
    // cast the abstract state type to the type we expect
    const auto *pos = state->as<ob::RealVectorStateSpace::StateType>();

    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return distmap->getDistance(octomap::point3d(pos->values[0], pos->values[1], pos->values[2])) > 0.6;
}

void plan(websocketpp::connection_hdl hdl) {// construct the state space we are planning in
    auto start_time = std::chrono::high_resolution_clock::now();
    auto space(std::make_shared<ob::RealVectorStateSpace>(3));

    // set the bounds for the R^3 part of SE(3)
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, min[0]);
    bounds.setHigh(0, max[0]);
    bounds.setLow(1, min[1]);
    bounds.setHigh(1, max[1]);
    bounds.setLow(2, min[2]);
    bounds.setHigh(2, max[2]);

    space->setBounds(bounds);

    // construct an instance of  space information from this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // set state validity checking for this space
    si->setStateValidityChecker(isStateValid);

    ob::ScopedState<> start(space);
    start[0] = -3;
    start[1] = 3;
    start[2] = 1;

    ob::ScopedState<> goal(space);
    goal[0] = 15;
    goal[1] = 1;
    goal[2] = 2;


    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // create a planner for the defined space
    auto planner(std::make_shared<og::RRT>(si));

    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);
//    planner->as<og::RRTstar>()->setGoalBias(0.2); //Goal Bias

    // perform setup steps for the planner
    planner->setup();


    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = planner->ob::Planner::solve(2);

    if (solved) {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        std::cout << "Found solution" << std::endl;
        og::PathGeometric *path = pdef->getSolutionPath()->as<og::PathGeometric>();
        auto states = path->getStates();
        nlohmann::json pathArray;
        for (const auto &state: states) {
            const auto *pos = state->as<ob::RealVectorStateSpace::StateType>();
            pathArray.push_back({{"x", pos->values[0]},
                                 {"y", pos->values[1]},
                                 {"z", pos->values[2]}});
        }
        auto time = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now() - start_time).count();
        nlohmann::json jsonArray = {
                {"path", pathArray},
                {"time", time}
        };
        std::string jsonString = jsonArray.dump();
        wsServer.binarySend(hdl, "octomap_path", jsonString);
        nlohmann::json completeJson = {
                {"path",      pathArray},
                {"time",      time},
                {"num_nodes", states.size()},
                {"cost",      path->length()}
        };
        wsServer.binarySend(hdl, "octomap_completed", completeJson.dump());
    } else
        std::cout << "No solution found" << std::endl;
}

//void planWithSimpleSetup()
//{
//    // construct the state space we are planning in
//    auto space(std::make_shared<ob::SE3StateSpace>());
//
//    // set the bounds for the R^3 part of SE(3)
//    ob::RealVectorBounds bounds(3);
//    bounds.setLow(-1);
//    bounds.setHigh(1);
//
//    space->setBounds(bounds);
//
//    // define a simple setup class
//    og::SimpleSetup ss(space);
//
//    // set state validity checking for this space
//    ss.setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });
//
//    // create a random start state
//    ob::ScopedState<> start(space);
//    start.random();
//
//    // create a random goal state
//    ob::ScopedState<> goal(space);
//    goal.random();
//
//    // set the start and goal states
//    ss.setStartAndGoalStates(start, goal);
//
//    // this call is optional, but we put it in to get more output information
//    ss.setup();
//    ss.print();
//
//    // attempt to solve the problem within one second of planning time
//    ob::PlannerStatus solved = ss.solve(1.0);
//
//    if (solved)
//    {
//        std::cout << "Found solution:" << std::endl;
//        // print the path to screen
//        ss.simplifySolution();
//        ss.getSolutionPath().print(std::cout);
//    }
//    else
//        std::cout << "No solution found" << std::endl;
//}

void onOpenCallback(websocketpp::connection_hdl hdl) {
    auto stoppableThreadPtr = std::make_shared<StoppableThread>();
    stoppableThreadPtr->startThread([hdl, stoppableThreadPtr]() {
        std::stringstream buffer;
        tree.writeBinaryData(buffer);
        std::string str = buffer.str();
        wsServer.binarySend(hdl, "octomap", str);
        plan(hdl);
    });
    stoppableThreadPtr->detach();
    wsServer.runningThreads[hdl.lock().get()] = std::move(stoppableThreadPtr);
}

int main(int /*argc*/, char ** /*argv*/) {
    tree.getMetricMin(min[0], min[1], min[2]);
    tree.getMetricMax(max[0], max[1], max[2]);
    distmap = new DynamicEDTOctomap(1.0, &tree, octomap::point3d(min[0], min[1], min[2]),
                                    octomap::point3d(max[0], max[1], max[2]), false);
    distmap->update();
    std::cout << "Starting Server..." << std::endl;

    wsServer.setOnOpenCallback([](websocketpp::connection_hdl hdl) {
        onOpenCallback(hdl); // Call your original onOpenCallback function
    });
    wsServer.start();
    WebSocketServer::join();
//    plan();
//
//    planWithSimpleSetup();

    return 0;
}