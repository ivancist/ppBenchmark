#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/State.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include "ompl/tools/benchmark/Benchmark.h"
#include "ompl/geometric/planners/est/EST.h"
#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/geometric/planners/prm/PRM.h"
#include "ompl/geometric/planners/prm/PRMstar.h"
#include "ompl/geometric/planners/sbl/SBL.h"
#include "ompl/geometric/planners/kpiece/KPIECE1.h"
#include "ompl/geometric/planners/kpiece/LBKPIECE1.h"

#include "octomap/octomap.h"
#include "dynamicEDT3D/dynamicEDTOctomap.h"
#include "CustomPlanner.h"

namespace geometric = ompl::geometric;
namespace base = ompl::base;

std::shared_ptr<Environment> env = std::make_shared<Environment>();
octomap::OcTree* tree = new octomap::OcTree("../octomap.bt");
double min[3], max[3];

// A function that matches the ompl::base::PlannerAllocator type.
// It will be used later to allocate an instance of EST
ompl::base::PlannerPtr myConfiguredPlanner(const ompl::base::SpaceInformationPtr &si) {
    geometric::EST *est = new ompl::geometric::EST(si);
    est->setRange(100.0);
    return base::PlannerPtr(est);
}

bool isStateValid(const ompl::base::State *state) {
    // cast the abstract state type to the type we expect
    const auto *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();

    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return env->distmap->getDistance(octomap::point3d(pos->values[0], pos->values[1], pos->values[2])) > 0.6;
}

int main() {
    initializeEnvironment(env, tree, 1.0);

    auto space(std::make_shared<ompl::base::RealVectorStateSpace>(3));
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(0, env->offset_x);
    bounds.setHigh(0, env->offset_x + env->x);
    bounds.setLow(1, env->offset_y);
    bounds.setHigh(1, env->offset_y + env->y);
    bounds.setLow(2, env->offset_z);
    bounds.setHigh(2, env->offset_z + env->z);

    space->setBounds(bounds);
    // Create a state space for the space we are planning in
    ompl::geometric::SimpleSetup ss(space);

    // set state validity checking for this space
    ss.setStateValidityChecker([](const ompl::base::State *state) { return isStateValid(state); });

    ompl::base::ScopedState<> start(space);
    start[0] = -3;
    start[1] = 3;
    start[2] = 1;

    ompl::base::ScopedState<> goal(space);
    goal[0] = 15;
    goal[1] = 1;
    goal[2] = 2;

    // Configure the problem to solve: set start state(s)
    // and goal representation
    // Everything must be set up to the point ss.solve()
    // can be called. Setting up a planner is not needed
    ss.setStartAndGoalStates(start, goal);
    ss.solve();

// First we create a benchmark class:
    ompl::tools::Benchmark b(ss, "Benchmark");

// We add the planners to evaluate.

//    b.addPlanner(base::PlannerPtr(new geometric::PRM(ss.getSpaceInformation())));
//    auto rrt = base::PlannerPtr(new geometric::RRT(ss.getSpaceInformation()));
//    rrt->as<geometric::RRT>()->setGoalBias(0.2);
//    b.addPlanner(rrt);
//    b.addPlanner(base::PlannerPtr(new ompl::CustomPlanner(ss.getSpaceInformation(),env,false)));

    auto rrtstar = base::PlannerPtr(new geometric::RRTstar(ss.getSpaceInformation()));
    rrtstar->as<geometric::RRTstar>()->setGoalBias(0.2);
    rrtstar->as<geometric::RRTstar>()->setFocusSearch(false);
    rrtstar->as<geometric::RRTstar>()->setSampleRejection(false);
    b.addPlanner(rrtstar);
    b.addPlanner(base::PlannerPtr(new ompl::CustomPlanner(ss.getSpaceInformation(),env,true)));

//// For planners that we want to configure in specific ways,
//// the ompl::base::PlannerAllocator should be used:
//    b.addPlannerAllocator(std::bind(&myConfiguredPlanner, std::placeholders::_1));
//// etc.

// Now we can benchmark: 5 second time limit for each plan computation,
// 100 MB maximum memory usage per plan computation, 50 runs for each planner
// and true means that a text-mode progress bar should be displayed while
// computation is running.
    ompl::tools::Benchmark::Request req;
    req.maxTime = 2.0;
    req.maxMem = 100.0;
    req.runCount = 20;
    req.displayProgress = true;
    b.benchmark(req);

// This will generate a file of the form ompl_host_time.log
    b.saveResultsToFile();
}