/*
 * This is a RRT planner.
 *
 * Date: Aug/20/2018
 * Author: Wei Du
 *
 * Version: 1.0
 *
 */

#ifndef __RRTPLANNER_H_
#define __RRTPLANNER_H_


#include <vector>
#include <iostream>
#include "environment_xyzyaw.h"

enum EXT{Reached, Advanced, Trapped, Goal};

class RRTPlanner {
public:

    RRTPlanner();
    ~RRTPlanner();

    /*
     * initialize the planner.
     */
    RRTPlanner(EnvironmentCONTXYZYAW* env_, int numOfSamples_);
   
    /*
     * run planner returns true if goal is reached;
     */
    virtual bool Run();


protected:

    EnvironmentCONTXYZYAW* env;
    int goalstateid;
    int startstateid;
    int numOfSamples;

    virtual bool GoalReached(int stateID);

    /*
     * inintialize a tree rooted at the input state.
     * returns the id of the tree.
     * -1 if the initialization is not successful.
     */
    virtual bool Init(int stateID);
 
    /*
     * Sampling
     * Return the stateID of sample state;
     */
    virtual int Random_state();

    /*
     * Extend the tree with treeID to the sample
     */
    virtual EXT Extend(int treeID, int sampleStateID);


    /*
     * return the stateID of the nearest enighbor
     */
    virtual int NearestNeighbor(int treeID, int queryStateID);

    /*
     * get a new state
     * return -1 if the new one is invalid, otherwise the state ID new state
     */
    virtual int NewConfig(int parentID, int sampleStateID);
};

#endif
