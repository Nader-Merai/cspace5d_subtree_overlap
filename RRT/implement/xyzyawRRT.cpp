#include <cstring>
#include <iostream>
#include <string>
#include <ctime>
#include <vector>

//#include <contxyzyaw/headers.h>
#include <RRTPlanner.h>
#include <contxyzyaw/environment_xyzyaw.h>

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


void initializePlanner(RRTPlanner* &planner,
                       EnvironmentCONTXYZYAW& env) { 
    
    // number of samples
    planner = new RRTPlanner(&env, 450000); 
}

int runPlanner(RRTPlanner* planner) {


    clock_t start = clock();

    int bRet = planner->Run();

    clock_t end = clock();
    double diff = double(end - start)/CLOCKS_PER_SEC;
    std::cout<<"Time: "<<diff<<" s"<<std::endl;

    if (bRet)
        printf("Solution is found.\n");
    else
        printf("Solution does not exist.\n");
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


// RRT
void planxythetalat(char* envCfgFilename, char* envStlFilename, char* SGFilename){
    // set the perimeter of the robot
    vector<sbpl_2Dpt_t> perimeter;
    createFootprint(perimeter);
    std::vector<double> startgoal;

    // initialize an environment
    // continuousspace environment
    EnvironmentCONTXYZYAW env;
    initializeEnv(env, perimeter, envCfgFilename, envStlFilename);

    ReadinStartGoal(SGFilename, startgoal);

    // specify a start and goal state
    int start_id, goal_id;

    // test that RRT works
    /*
     *setEnvStartGoal(env,
     *        0.2, 1.5, 1.0, 0.0,
     *        0.036, 0.0, 0.0,
     *        8.00, 8.2275, 1.0, 0.0,
     *        0.036, 0.0, 0.0, 
     *        start_id, goal_id);
     */
    setEnvStartGoal(env,
            startgoal[0], startgoal[1], startgoal[2], startgoal[3],
            startgoal[4], startgoal[5], startgoal[6], 
            startgoal[7], startgoal[8], startgoal[9], startgoal[10],
            startgoal[11], startgoal[12], startgoal[13],
            start_id, goal_id);


    // initialize a planner with start and goal state
    RRTPlanner* planner = nullptr;

    initializePlanner(planner, env); 

    // plan
    vector<int> solution_stateIDs;
    runPlanner(planner);

    // print stats
    env.PrintTimeStat(stdout);

    // write out solutions
    std::string filename("sol.txt");
    writeSolution(env, solution_stateIDs, filename.c_str());

    delete planner;
}




int main(int argc, char *argv[])
{   
    planxythetalat(argv[1], argv[2], argv[3]);
}
