#ifndef PPBENCHMARK_CUSTOMPLANNER_H
#define PPBENCHMARK_CUSTOMPLANNER_H

#include <ompl/base/Planner.h>

// often useful headers:
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>
#include "RRT_star.h"

namespace ompl {

    class CustomPlanner : public base::Planner {
    public:
        CustomPlanner(const base::SpaceInformationPtr &si, std::shared_ptr<Environment> &env, bool optimize = false);

        virtual ~CustomPlanner();

        virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

    private:
        std::vector<Node *> tree;
        Environment env;
        RRTStar rrtStar;
        bool optimize = false;
    };

}

#endif //PPBENCHMARK_CUSTOMPLANNER_H
