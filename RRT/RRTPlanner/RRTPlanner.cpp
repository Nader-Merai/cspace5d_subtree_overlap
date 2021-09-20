/*
 * This is a RRT planner.
 *
 * Date: Aug/20/2018
 * Author: Wei Du
 *
 * Version: 1.0
 *
 */


#include "RRTPlanner.h"


RRTPlanner::RRTPlanner() {
    numOfSamples = 0;
    env = nullptr;
    goalstateid = -1;
    startstateid = -1;
} 

RRTPlanner::RRTPlanner(EnvironmentCONTXYZYAW* env_, int numOfSamples_):env(env_), numOfSamples(numOfSamples_) {
    if (nullptr == env_){
        std::cerr << "env does not exist." << std::endl;
        exit(0);
    }

    if (numOfSamples <= 10) {
        std::cerr << "Number of samples are too small." << std::endl;
        exit(0);
    }

    goalstateid = env_->GetGoalStateID();
    startstateid = env_->GetStartStateID();
}

RRTPlanner::~RRTPlanner(){
    if (nullptr != env) {
        env = nullptr;
    }
}

bool RRTPlanner::Run()  {

    int start_tree = Init(startstateid);
    if (start_tree) {

        for (int k = 0; k < numOfSamples; ++k ) {
            if (k % 10000 == 0) std::cout << "we're at sample: " << k << std::endl << std::flush;
            int s_randID = Random_state();

            auto ext = Extend(start_tree, s_randID);

            if(ext == Goal){
                return true;
            }
        }
        return false;
    }   

    return false;
}


/*
 *bool RRTPlanner::GoalReached(int stateID) {
 *    return env->IsWithinGoalRegion(stateID);
 *}
 */

int RRTPlanner::Init(int stateID) {
    return env->InitTree(stateID);
}

int RRTPlanner::Random_state() {
    return env->GetRandomSample();
}

EXT RRTPlanner::Extend(int treeID, int sampleStateID) {
    int s_near = NearestNeighbor(treeID, sampleStateID);

    int s_new = NewConfig(s_near, sampleStateID);
    

    if(s_new == goalstateid) return Goal;

    if (-1 != s_new) {

        if(s_new != sampleStateID) {
            return Advanced;
        } else {
            return Reached;
        }
    }
    return Trapped;
}

int RRTPlanner::NearestNeighbor(int treeID, int queryStateID) {
    return env->GetNN(treeID, queryStateID);
}

int RRTPlanner::NewConfig(int parentID, int sampleStateID) {
    return env->GetSuccs(parentID, sampleStateID);
}


