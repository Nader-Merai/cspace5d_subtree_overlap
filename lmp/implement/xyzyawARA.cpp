#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <vector>

#include <contxyzyaw/headers.h>
#include <contxyzyaw/environment_xyzyaw.h>

#define EPS 10.0

using namespace std; 

void createFootprint(vector<sbpl_2Dpt_t>& perimeter){
    sbpl_2Dpt_t pt_m;
    double halfwidth = 0.025;
    double halflength = 0.025;
    pt_m.x = -halflength;
    pt_m.y = -halfwidth;
    perimeter.push_back(pt_m);
    pt_m.x = halflength;
    pt_m.y = -halfwidth;
    perimeter.push_back(pt_m);
    pt_m.x = halflength;
    pt_m.y = halfwidth;
    perimeter.push_back(pt_m);
    pt_m.x = -halflength;
    pt_m.y = halfwidth;
    perimeter.push_back(pt_m);
}

void initializeEnv(EnvironmentCONTXYZYAW& env,
                   vector<sbpl_2Dpt_t>& perimeter,
                   char* envCfgFilename, char* envStlFilename){
  if (!env.InitializeEnv(envCfgFilename,envStlFilename)) {
        printf("ERROR: InitializeEnv failed\n");
    }
}


void setEnvStartGoal(EnvironmentCONTXYZYAW& env,
                     double start_x, double start_y, double start_z, double start_yaw,
                     double start_vxy, double start_vz, double start_w,
                     double goal_x,  double goal_y,  double goal_z, double goal_yaw,
                     double goal_vxy, double goal_vz, double goal_w,
                     int& start_id, int& goal_id){

    env.SetGoalTolerance(0.1, 0.1, 0.1, 6.2832); 
    start_id = env.SetStart(start_x, start_y, start_z, start_yaw, start_vxy, start_vz, start_w);
    goal_id = env.SetGoal(goal_x, goal_y, goal_z, goal_yaw, goal_vxy, goal_vz, goal_w );
}


void initializePlanner(SBPLPlanner*& planner,
                       EnvironmentCONTXYZYAW& env,
                       int start_id, int goal_id,
                       double initialEpsilon,
                       bool bsearchuntilfirstsolution){

    // ensure forward search
    bool bsearch =true;
    planner = new ARAPlanner(&env, bsearch, EPS);

    // set planner properties
    if (planner->set_start(start_id) == 0) {
        printf("ERROR: failed to set start state\n");
        throw new SBPL_Exception();
    }
    if (planner->set_goal(goal_id) == 0) {
        printf("ERROR: failed to set goal state\n");
        throw new SBPL_Exception();
    }

    planner->set_initialsolution_eps(initialEpsilon);
    planner->set_search_mode(bsearchuntilfirstsolution);
}



int runPlanner(SBPLPlanner* planner, int allocated_time_secs,
        vector<int>&solution_stateIDs){

    clock_t start = clock();
    int bRet = planner->replan(allocated_time_secs, &solution_stateIDs);
    clock_t end = clock();
    double diff = double(end - start)/CLOCKS_PER_SEC;
    std::cout<<"Time: "<<diff<<" s"<<std::endl;

    if (bRet)
        printf("Solution is found\n");
    else
        printf("Solution does not exist\n");
    return bRet;
}

void writeSolution(EnvironmentCONTXYZYAW& env, vector<int> solution_stateIDs,
                   const char* filename){
    std::string discrete_filename(std::string(filename) + std::string(".discrete"));
    FILE* fSol_discrete = fopen(discrete_filename.c_str(), "w");
    FILE* fSol = fopen(filename, "w");
    if (fSol == NULL) {
        printf("ERROR: could not open solution file\n");
        throw SBPL_Exception();
    }

    // environment resolution
    double res = env.EnvCONTXYZYAWCfg.cellsize_m;
    // write the discrete solution to file
    for (size_t i = 0; i < solution_stateIDs.size(); i++) {
        double x, y, z, yaw, vxy, vz, w;
        env.GetCoordFromState(solution_stateIDs[i], x, y, z, yaw, vxy, vz, w);

        int dsctx, dscty, dsctz, dsctyaw;
        dsctx = CONTXY2DISC(x, res);
        dscty = CONTXY2DISC(y, res);
        dsctz = CONTXY2DISC(z, res);
        dsctyaw = (int)(yaw/ANGLERESOLUTION + 0.500);
        fprintf(fSol_discrete, "%d %d %d %d\n", dsctx, dscty, dsctz, dsctyaw);
    }
    fclose(fSol_discrete);

    // write the continuous solution to file
    vector<sbpl_xyz_rpy_pt_t>  xyzyawPath;
    env.ConvertStateIDPathintoXYZYawPath(&solution_stateIDs, &xyzyawPath);
    for (unsigned int i = 0; i < xyzyawPath.size(); i++) {
        fprintf(fSol, "%.3f %.3f %.3f %.3f\n", xyzyawPath.at(i).x,
                                               xyzyawPath.at(i).y,
                                               xyzyawPath.at(i).z,
                                               xyzyawPath.at(i).yaw);
    }
    fclose(fSol);
}

// read and store star & goal info
void ReadinStartGoal(char* SGFilename, std::vector<double> &startgoal){
    startgoal.clear();
    ifstream sg;
    std::string line;
    std::string buf;

    sg.open(SGFilename);
    if(!sg.is_open()) {
        std::cout << "SGFile open failed." << std::endl;
        exit(0);
    }
    
    while(getline(sg, line)) {
        std::stringstream ss(line);
        ss >> buf;
        while (ss >> buf)

            startgoal.push_back(atof(buf.c_str()));
    }
    
    if ( 15 != startgoal.size() ){
        std::cout << "Wrong .sg file format, parameters missing!" << std::endl;
        exit(0);
    }
}

 //ARA
void planxythetalat(char* envCfgFilename, char* envStlFilename, char* SGFilename, char* mode){
    // set the perimeter of the robot
    vector<sbpl_2Dpt_t> perimeter;
    createFootprint(perimeter);
    std::vector<double> startgoal;


    // initialize an environment
    // continuousspace environment
    EnvironmentCONTXYZYAW env;
    env.mode = atoi(mode);
    env.SetupR(0.06);
    initializeEnv(env, perimeter, envCfgFilename, envStlFilename);

    ReadinStartGoal(SGFilename, startgoal);

    // specify a start and goal state
    int start_id, goal_id;
    // start x, y, z, yaw, vxy, vz, w
    // goal  x, y, z, yaw, vxy, vz, w
    setEnvStartGoal(env,
            startgoal[0], startgoal[1], startgoal[2], startgoal[3],
            startgoal[4], startgoal[5], startgoal[6], 
            startgoal[7], startgoal[8], startgoal[9], startgoal[10],
            startgoal[11], startgoal[12], startgoal[13],
            start_id, goal_id);

    double R = startgoal[14];
    // env.SetupR(R);
    

    // initialize a planner with start and goal state
    SBPLPlanner* planner = NULL;
    double initialEpsilon = EPS;
    //bool bsearchuntilfirstsolution = true;
    bool bsearchuntilfirstsolution = false;

    initializePlanner(planner, env, start_id, goal_id, initialEpsilon,
                      bsearchuntilfirstsolution);

   
    // plan
    vector<int> solution_stateIDs;
    int allocated_time_secs = 120; // in seconds
    runPlanner(planner, allocated_time_secs, solution_stateIDs);

    // print stats
    env.PrintTimeStat(stdout);

    // write out solutions
    std::string filename("sol.txt");
    writeSolution(env, solution_stateIDs, filename.c_str());

    delete planner;
}




int main(int argc, char *argv[])
{   
    planxythetalat(argv[1], argv[2], argv[3], argv[4]);
}
