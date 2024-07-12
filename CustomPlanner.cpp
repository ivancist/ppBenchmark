//
// Created by Ivan Cisternino on 17/05/24.
//

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/geometric/PathGeometric.h>
#include "CustomPlanner.h"

namespace ompl {
    CustomPlanner::CustomPlanner(const base::SpaceInformationPtr &si, std::shared_ptr <Environment> &env, bool optimize)
            : base::Planner(si, std::string("Custom").append(optimize ? "Opt" : "")) {
        // Set default values for the parameters
        specs_.optimizingPaths = true;
        specs_.recognizedGoal = base::GOAL_STATE;

        addPlannerProgressProperty("iterations INTEGER", [this] { return rrtStar.numIterationsProperty(); });

        this->optimize = optimize;
        this->env = *env;
    }

    CustomPlanner::~CustomPlanner() {
        std::cout << "Destructor called" << std::endl;
    }

    base::PlannerStatus CustomPlanner::solve(const base::PlannerTerminationCondition &ptc) {
        // Ensure the planner is configured correctly
        checkValidity();

        base::Goal *goal = pdef_->getGoal().get();
        Node goalNode{goal->as<base::GoalState>()->getState()->as<base::RealVectorStateSpace::StateType>()->values[0],
                      goal->as<base::GoalState>()->getState()->as<base::RealVectorStateSpace::StateType>()->values[1],
                      goal->as<base::GoalState>()->getState()->as<base::RealVectorStateSpace::StateType>()->values[2],
                      nullptr, std::numeric_limits<double>::max()};
        base::State *startState = pdef_->getStartState(0);
        Node startNode{startState->as<base::RealVectorStateSpace::StateType>()->values[0],
                       startState->as<base::RealVectorStateSpace::StateType>()->values[1],
                       startState->as<base::RealVectorStateSpace::StateType>()->values[2],
                       nullptr, 0};

        std::shared_ptr <StoppableThread> stoppableThreadPtr = std::make_shared<StoppableThread>();
        if (optimize) {
            rrtStar.MAX_OPTIMIZING_ITERATIONS = 100 * 1000;
            // Your RRT* algorithm starts here
            // You might need to adjust the parameters based on your specific implementation
            stoppableThreadPtr->startThread([ptc, stoppableThreadPtr]() {
                while (ptc() == false) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                stoppableThreadPtr->stopThread();
            });
            stoppableThreadPtr->detach();
        }
        FinalReturn result = rrtStar.rrtStar(&startNode, &goalNode, env, .6, nullptr, websocketpp::connection_hdl(),
                                             stoppableThreadPtr);

        rrtStar.pathPruning(result.path);
//        rrtStar.pathSmoothing(result.path, .5, 20);

        // Assuming you have a way to convert your FinalReturn to an OMPL solution
        // For example, you might store the path in a vector of States and add it to the problem definition
        if (result.path != nullptr) {
            geometric::PathGeometric *solutionPath = new geometric::PathGeometric(si_);
            for (Node *node: *result.path) {
                // Convert Node to State and add to the solution path
                // This part depends on how your Node structure maps to the state space
                base::State *state = si_->allocState();
                state->as<base::RealVectorStateSpace::StateType>()->values[0] = node->x;
                state->as<base::RealVectorStateSpace::StateType>()->values[1] = node->y;
                state->as<base::RealVectorStateSpace::StateType>()->values[2] = node->z;
                solutionPath->append(state);
            }
            pdef_->addSolutionPath(base::PlannerSolution(base::PathPtr(solutionPath)));
        }
        return base::PlannerStatus::EXACT_SOLUTION;
    }


} // ompl