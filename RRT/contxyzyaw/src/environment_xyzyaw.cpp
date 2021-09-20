/*
 * This is a environment in continuous space. Based on sbpl library environment template.
 *
 * It's used for planning in (x,y,z,yaw) space. Roll and pitch are assumed to be 0.
 * Robot  is assumed to be a point robot.
 *
 * Need another map configuration file to describe the environment:
 * Height, width, length, resolution etc;
 *
 * Date: Jun/4/2018
 * By: Wei Du
 *
 * Version: 1.0
 * This version dose not require preprocessed motion primitive file.
 * The successors are generated from actions(which are described by formulations) onlion.
 *
 */

#include <cmath>
#include <cstring>
#include <ctime>
#include <cassert>
#include <fstream>
#include <iostream>

// sbpl config
#include <iostream>
#include <contxyzyaw/environment_xyzyaw.h>
#include <sbpl/utils/key.h>
#include <sbpl/utils/mdp.h>
#include <sbpl/utils/mdpconfig.h>

#include <smpl/geometry/voxelize.h>
#include <contxyzyaw/stl_reader.h>
#include <boost/functional/hash.hpp>

#if TIME_DEBUG
static clock_t time3_addallout = 0;
static clock_t time_gethash = 0;
static clock_t time_createhash = 0;
static clock_t time_getsuccs = 0;
static clock_t time_neighbor = 0;
#endif


#if !defined(MAX)
#define MAX(A,B) ( (A) > (B) ? (A):(B) )
#endif

#if !defined(MIN)
#define MIN(A,B) ( (A) < (B) ? (A):(B) )
#endif

static long int checks = 0;
static int numofNewSuccs = 0;
static int sample_times = 0;

// sorting pairs
bool sortbysec( const std::pair<size_t, double> &a,
        const std::pair<size_t, double> &b ){
    return (a.second < b.second);
}


// state hashentry setup
std::size_t hashkey(double x_, double y_, double z_, double yaw_,
        double vxy_, double vz_, double omega_){

    // to specify the precision
    std::size_t x, y, z, yaw, vxy, vz, omega;
    x = boost::hash_value(x_);
    y = boost::hash_value(y_);
    z = boost::hash_value(z_);
    yaw = boost::hash_value(yaw_);
    vxy = boost::hash_value(vxy_);

    std::size_t seed = 37;
    boost::hash_combine(seed, x);
    boost::hash_combine(seed, y);
    boost::hash_combine(seed, z);
    boost::hash_combine(seed, yaw);
    boost::hash_combine(seed, vxy);

    return seed;
}


EnvironmentCONTXYZYAW::EnvironmentCONTXYZYAW(){
    EnvCONTXYZYAWCfg.obsthresh = CNTSPACE_DEFAULTOBSTHRESH; // the value that pretty much makes it disabled
    EnvCONTXYZYAWCfg.cost_inscribed_thresh = EnvCONTXYZYAWCfg.obsthresh;
    // the value that pretty much maeks it disabled
    EnvCONTXYZYAWCfg.cost_possibly_circumscribed_thresh = -1;

    iteration = 0;
    bucketsize = 0; //fixed bucket size
    blocksize = 1;

    EnvCONTXYZYAW.bInitialized = false;

    EnvCONTXYZYAWCfg.actionwidth = CNTSPACE_DEFAULT_ACTIONWIDTH;

    EnvCONTXYZYAWCfg.NumYawDirs = CONTXYZYAW_YAWDIRS;

    // no memory allocated in cfg yet
    EnvCONTXYZYAWCfg.ActionsV = nullptr;
    EnvCONTXYZYAWCfg.PredActionsV = nullptr;
    kdtree = nullptr;
    EnvAction = nullptr;

    debug.open("debug_state.txt");

}

EnvironmentCONTXYZYAW::~EnvironmentCONTXYZYAW(){
    SBPL_PRINTF("destorying XYZYAW\n");

    // delete actions
    if( EnvCONTXYZYAWCfg.ActionsV != nullptr ){
        for (int tind=0; tind < EnvCONTXYZYAWCfg.NumYawDirs; ++tind){
            delete[] EnvCONTXYZYAWCfg.ActionsV[tind];
        }
        delete[] EnvCONTXYZYAWCfg.ActionsV;
        EnvCONTXYZYAWCfg.ActionsV = nullptr;
    }
    if( EnvCONTXYZYAWCfg.PredActionsV != nullptr ){
        delete[] EnvCONTXYZYAWCfg.PredActionsV;
        EnvCONTXYZYAWCfg.PredActionsV = nullptr;
    }

    if( nullptr != kdtree ){
        delete kdtree;
    }
    debug.close();
}


// not having a motion primitive file in this environment.
bool EnvironmentCONTXYZYAW::InitializeEnv(const char* cfgEnvFile,  const char* stlEnvFile){

    // sbpl error handling;
    SBPL_INFO("InitializeEnv start: cfgEnvFile=%s  stlEnvFile=%s\n", cfgEnvFile,  stlEnvFile);
    fflush(stdout);
    FILE* fCfg1 = fopen(cfgEnvFile, "r");
    FILE* fCfg2 = fopen(stlEnvFile, "r");
    if (nullptr == fCfg1){
        std::stringstream ss;
        ss << "ERROR: unable to open " << cfgEnvFile;
        throw SBPL_Exception(ss.str());
    }
    fclose(fCfg1);

    if (nullptr == fCfg2){
        std::stringstream ss;
        ss << "ERROR: unable to open " << stlEnvFile;
        throw SBPL_Exception(ss.str());
    }
    fclose(fCfg2);

    // Read in envrionment cfg file
    ReadinEnvParam(cfgEnvFile);

    //Read in file function.
    ReadConfiguration(stlEnvFile);

    InitGeneral();

    if( EnvCONTXYZYAWCfg.StartYaw < 0 || 
            EnvCONTXYZYAWCfg.StartYaw >= 2 * M_PI ){
        throw new SBPL_Exception("ERROR: illegal start coordiantes for yaw");
    }
    if( EnvCONTXYZYAWCfg.EndYaw < 0 || 
            EnvCONTXYZYAWCfg.EndYaw >= 2 * M_PI ){
        throw new SBPL_Exception("ERROR: illegal end coordiantes for yaw");
    }


    SBPL_PRINTF("size of env: %d  by %d by %d\n", EnvCONTXYZYAWCfg.EnvLength, EnvCONTXYZYAWCfg.EnvWidth, EnvCONTXYZYAWCfg.EnvHeight);
    return true;

}   // InitializeEnv


int EnvironmentCONTXYZYAW::InitTree(int stateID) {
    assert(stateID < StateID2CoordTable.size());

    // state as root
    auto HashEntry = StateID2CoordTable.at(stateID);

    // kdtree
    treeNodes.nodes.push_back(HashEntry);
    
    // dimension is 4, x, y, z, yaw
    kdtree = new kd_tree_t(4 /*dim*/, treeNodes, nanoflann::KDTreeSingleIndexAdaptorParams(10) );

    int size = treeNodes.kdtree_get_point_count();
    kdtree->addPoints((size_t)size-1, (size_t)size-1);

    debug << HashEntry->X << " "
        << HashEntry->Y << " "
        << HashEntry->Z << " "
        << HashEntry->Yaw << std::endl;

    return 1;
}


int EnvironmentCONTXYZYAW::GetRandomSample() {
    sample_times++;

    EnvCONTXYZYAWHashEntry_t* HashEntry;

    int seed = rand() % 100;

    double envLength = EnvCONTXYZYAWCfg.EnvLength * EnvCONTXYZYAWCfg.cellsize_m;
    double envWidth = EnvCONTXYZYAWCfg.EnvWidth * EnvCONTXYZYAWCfg.cellsize_m;
    double envHeight = EnvCONTXYZYAWCfg.EnvHeight * EnvCONTXYZYAWCfg.cellsize_m;
    double minspeed = EnvAction->GetMinSpeed();
    double maxspeed = EnvAction->GetMaxSpeed();
    double vz_ = EnvAction->GetVZ();
    double w_ = EnvAction->GetW();


    // produce a state {x, y, z, yaw, vxy, vz, w}
    while(true) {
        double x, y, z, yaw;
        
        // then sample in goal region
        if( seed < 65) {

            x = bias.x_ + static_cast<double> (rand()) / (static_cast<double> (RAND_MAX / bias.rangex));
            y = bias.y_ + static_cast<double> (rand()) / (static_cast<double> (RAND_MAX / bias.rangey));
            z = bias.z_ + static_cast<double> (rand()) / (static_cast<double> (RAND_MAX / bias.rangez));
            yaw = bias.yaw_ + static_cast<double> (rand()) / (static_cast<double> (RAND_MAX / (bias.rangeyaw)));

        } else {

            x = static_cast<double> (rand()) / (static_cast<double> (RAND_MAX / envLength));
            y = static_cast<double> (rand()) / (static_cast<double> (RAND_MAX / envWidth));
            z = static_cast<double> (rand()) / (static_cast<double> (RAND_MAX / envHeight));
            yaw = static_cast<double> (rand()) / (static_cast<double> (RAND_MAX / (M_PI * 2)));

        }

        double vxy = minspeed + static_cast<double> (rand()) / (static_cast<double> (RAND_MAX / (maxspeed - minspeed)));
        double vz = vz_ * (rand()%3 - 1);
        double w = w_ * (rand() %3 - 1);

        // sample states could be invalid
        if (nullptr == (HashEntry = GetHashEntry( x, y, z, yaw, vxy, vz, w))) {

            HashEntry = CreateNewHashEntry( x, y, z, yaw, vxy, vz, w);
            break;

            // the rand sample exist
        } else {        
            continue;   
        }
    }   // end while 

    return HashEntry->stateID;
}


int EnvironmentCONTXYZYAW::GetNN(int treeID, int queryStateID) {

    auto HashEntry = StateID2CoordTable.at(queryStateID);

    /* nanoflann */
    double query_pt[4] = {HashEntry->X, HashEntry->Y, HashEntry->Z, HashEntry->Yaw};
    const size_t numofneighbors = 1;
    size_t neibIndex[numofneighbors];
    double distance[numofneighbors];


    int size = treeNodes.kdtree_get_point_count();
    //std::cout << "size: " << size << std::endl;

    nanoflann::KNNResultSet<double> resultSet(numofneighbors);
    resultSet.init(neibIndex, distance);
    auto found = kdtree->findNeighbors(resultSet, query_pt, nanoflann::SearchParams(0));

    if (found) {

        auto neighborEntry = treeNodes.nodes.at(neibIndex[0]);

        // the rand sample is in the tree already
        assert( neighborEntry->stateID != HashEntry->stateID); 

        return  neighborEntry->stateID;
    }

    return -1;
}

void EnvironmentCONTXYZYAW::InitializeEnvironment(){
    EnvCONTXYZYAWHashEntry_t* HashEntry;

    // nearest neighbor
    long long int maxsize=EnvCONTXYZYAWCfg.EnvWidth * EnvCONTXYZYAWCfg.EnvHeight * EnvCONTXYZYAWCfg.EnvLength;

    std::cout<<"Length: " << EnvCONTXYZYAWCfg.EnvLength << " Width:  "<<EnvCONTXYZYAWCfg.EnvWidth
        <<" Height: "<< EnvCONTXYZYAWCfg.EnvHeight << std::endl;

    SBPL_PRINTF("environment stores states in hash table");

    StateID2CoordTable.clear();

    // create start state
    if( nullptr == (HashEntry = this->GetHashEntry(
                    EnvCONTXYZYAWCfg.StartX,
                    EnvCONTXYZYAWCfg.StartY,
                    EnvCONTXYZYAWCfg.StartZ,
                    EnvCONTXYZYAWCfg.StartYaw,
                    EnvCONTXYZYAWCfg.StartVXY,
                    EnvCONTXYZYAWCfg.StartVZ,
                    EnvCONTXYZYAWCfg.StartW)) ){

        HashEntry = CreateNewHashEntry(
                EnvCONTXYZYAWCfg.StartX,
                EnvCONTXYZYAWCfg.StartY,
                EnvCONTXYZYAWCfg.StartZ,
                EnvCONTXYZYAWCfg.StartYaw,
                EnvCONTXYZYAWCfg.StartVXY,
                EnvCONTXYZYAWCfg.StartVZ,
                EnvCONTXYZYAWCfg.StartW );

    }
    EnvCONTXYZYAW.startstateid = HashEntry->stateID;

    // create goal state
    if( nullptr == (HashEntry = this->GetHashEntry(
                    EnvCONTXYZYAWCfg.EndX,
                    EnvCONTXYZYAWCfg.EndY,
                    EnvCONTXYZYAWCfg.EndZ,
                    EnvCONTXYZYAWCfg.EndYaw,
                    EnvCONTXYZYAWCfg.EndVXY,
                    EnvCONTXYZYAWCfg.EndVZ,
                    EnvCONTXYZYAWCfg.EndW)) ){

        HashEntry = CreateNewHashEntry(
                EnvCONTXYZYAWCfg.EndX,
                EnvCONTXYZYAWCfg.EndY,
                EnvCONTXYZYAWCfg.EndZ,
                EnvCONTXYZYAWCfg.EndYaw,
                EnvCONTXYZYAWCfg.EndVXY,
                EnvCONTXYZYAWCfg.EndVZ,
                EnvCONTXYZYAWCfg.EndW );
    }
    EnvCONTXYZYAW.goalstateid = HashEntry->stateID;

    EnvCONTXYZYAW.bInitialized = true;

}   // InitializeEnvironment

void EnvironmentCONTXYZYAW::InitializeEnvConfig(){
    EnvAction = new AirplaneAction();
    EnvAction->InitializeAction();
    EnvCONTXYZYAWCfg.nominalvel_mpersecs = EnvAction->GetMaxSpeed();


    std::srand(std::time(nullptr));
}

bool EnvironmentCONTXYZYAW::InitGeneral(){
    // Initializeaction
    InitializeEnvConfig(); 

    // Initialize Environment
    InitializeEnvironment();

    return true;
}


void EnvironmentCONTXYZYAW::ComputeHeuristicValues(){
    SBPL_PRINTF("Precomputing heuristics...\n");

    int goalx, goaly, goalz;
    goalx = CONTXY2DISC(EnvCONTXYZYAWCfg.EndX, EnvCONTXYZYAWCfg.cellsize_m);
    goaly = CONTXY2DISC(EnvCONTXYZYAWCfg.EndY, EnvCONTXYZYAWCfg.cellsize_m);
    goalz = CONTXY2DISC(EnvCONTXYZYAWCfg.EndZ, EnvCONTXYZYAWCfg.cellsize_m);


    SBPL_PRINTF("done\n");
}

// read in configuration parameters of environmet
void EnvironmentCONTXYZYAW::ReadinEnvParam(const char* cfgEnvFile){
    // read inthe configuration of environment and initialize
    // EnvNAVXYTHETACCfg structure

    char sTemp[1024], sTemp1[1024];
    unsigned char dTemp;
    int x, y, z;

    FILE* fCfg = fopen(cfgEnvFile, "r");

    // discretization(cells)
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early (discretization)");
    }
    strcpy(sTemp1, "discretization(cells):");
    if (strcmp(sTemp1, sTemp) != 0) {
        std::stringstream ss;
        ss << "ERROR: configuration file has incorrect format (discretization)" <<
            " Expected " << sTemp1 << " got " << sTemp;
        throw SBPL_Exception(ss.str());
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early (discretization)");
    }
    EnvCONTXYZYAWCfg.EnvLength = atoi(sTemp);

    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early (discretization)");
    }
    EnvCONTXYZYAWCfg.EnvWidth = atoi(sTemp);

    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early (discretization)");
    }
    EnvCONTXYZYAWCfg.EnvHeight = atoi(sTemp);


    //cellsize
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    strcpy(sTemp1, "cellsize(kilometers):");
    if (strcmp(sTemp1, sTemp) != 0) {
        std::stringstream ss;
        ss << "ERROR: configuration file has incorrect format" <<
            " Expected " << sTemp1 << " got " << sTemp;
        throw SBPL_Exception(ss.str());
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvCONTXYZYAWCfg.cellsize_m = atof(sTemp); 

    // start(meters,rads):
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvCONTXYZYAWCfg.StartX = atof(sTemp);

    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvCONTXYZYAWCfg.StartY = atof(sTemp);

    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvCONTXYZYAWCfg.StartZ = atof(sTemp);

    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvCONTXYZYAWCfg.StartYaw = atof(sTemp);

    if (EnvCONTXYZYAWCfg.StartX < 0 ||
            EnvCONTXYZYAWCfg.StartX >= EnvCONTXYZYAWCfg.EnvLength/EnvCONTXYZYAWCfg.cellsize_m){
        throw SBPL_Exception("ERROR: illegal start coordinates");
    }

    if (EnvCONTXYZYAWCfg.StartY < 0 ||
            EnvCONTXYZYAWCfg.StartY >= EnvCONTXYZYAWCfg.EnvWidth/EnvCONTXYZYAWCfg.cellsize_m){
        throw SBPL_Exception("ERROR: illegal start coordinates");
    }

    if (EnvCONTXYZYAWCfg.StartZ < 0 ||
            EnvCONTXYZYAWCfg.StartZ >= EnvCONTXYZYAWCfg.EnvWidth/EnvCONTXYZYAWCfg.cellsize_m){
        throw SBPL_Exception("ERROR: illegal start coordinates");
    }


    // end(meters,rads):
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvCONTXYZYAWCfg.EndX = atof(sTemp);

    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvCONTXYZYAWCfg.EndY = atof(sTemp);

    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvCONTXYZYAWCfg.EndZ = atof(sTemp);

    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvCONTXYZYAWCfg.EndYaw = atof(sTemp);


    if (EnvCONTXYZYAWCfg.EndX < 0 ||
            EnvCONTXYZYAWCfg.EndX >= (double)EnvCONTXYZYAWCfg.EnvLength/EnvCONTXYZYAWCfg.cellsize_m){
        throw SBPL_Exception("ERROR: illegal end coordinates");
    }

    if (EnvCONTXYZYAWCfg.EndY < 0 ||
            EnvCONTXYZYAWCfg.EndY >= (double)EnvCONTXYZYAWCfg.EnvWidth/EnvCONTXYZYAWCfg.cellsize_m){
        throw SBPL_Exception("ERROR: illegal end coordinates");
    }

    if (EnvCONTXYZYAWCfg.EndZ < 0 ||
            EnvCONTXYZYAWCfg.EndZ >= (double)EnvCONTXYZYAWCfg.EnvHeight/EnvCONTXYZYAWCfg.cellsize_m){
        throw SBPL_Exception("ERROR: illegal end coordinates");
    }


    EnvCONTXYZYAWCfg.nominalvel_mpersecs = 1.0;


    //NNtable = new std::vector<EnvCONTXYZYAWHashEntry_t*>**[EnvCONTXYZYAWCfg.EnvWidth+1];
    //for(int i = 0; i<= EnvCONTXYZYAWCfg.EnvWidth; ++i){
    //NNtable[i] = new std::vector<EnvCONTXYZYAWHashEntry_t*>* [EnvCONTXYZYAWCfg.EnvLength + 1];
    //for(int j = 0; j<= EnvCONTXYZYAWCfg.EnvLength; ++j){
    //NNtable[i][j] = new std::vector<EnvCONTXYZYAWHashEntry_t*> [EnvCONTXYZYAWCfg.EnvHeight];
    //}
    //}

    fclose(fCfg);

}   // ReadinEnvParam


// Read an stl file and convert it into voxels based on the environment resolution
// environment description file should be read first before readin this file
// if the resolution is too small, then too much voxels would be generated
// causing std::bad_alloc error
void EnvironmentCONTXYZYAW::ReadConfiguration( const char* stlEnvFile){


    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::uint32_t> indices;
    std::vector<Eigen::Vector3d> voxels;
    vertices.clear();
    indices.clear();
    voxels.clear();
    walls.clear();

    // load stl file
    std::vector<double> coords, normals;
    std::vector<std::uint32_t> tris, solids;

    try{
        stl_reader::ReadStlFile(stlEnvFile, coords, normals, tris, solids);
        const size_t numTris = tris.size()/3;

        for(size_t itri = 0; itri < numTris; ++itri){
            indices.push_back((std::uint32_t)itri*3 + 0);
            indices.push_back((std::uint32_t)itri*3 + 1);
            indices.push_back((std::uint32_t)itri*3 + 2);

            for(size_t icorner = 0; icorner < 3; ++icorner){
                double* c = &coords[3*tris[3*itri + icorner]];

                Eigen::Vector3d currentVertex( (double)c[0],(double)c[1], (double)c[2] );
                vertices.push_back(currentVertex);
            }
        }

    }catch (std::exception& e){
        std::cout << e.what() << std::endl;
    }

    // voxelize mesh
    bool fill = false;
    smpl::geometry::VoxelizeMesh(vertices, indices, EnvCONTXYZYAWCfg.cellsize_m, voxels, fill);

    // convert voxels in continuous space to discrete space    once and for all
    int x,y,z;
    int tempx, tempy, tempz;
    for(auto point:voxels){
        x = CONTXY2DISC(point.x(), EnvCONTXYZYAWCfg.cellsize_m);
        y = CONTXY2DISC(point.y(), EnvCONTXYZYAWCfg.cellsize_m);
        z = CONTXY2DISC(point.z(), EnvCONTXYZYAWCfg.cellsize_m);
        if(z < 0) z = 0;

        // thicken the wall
        for(int i = -1; i<2; ++i){
            tempx = x +i;
            if(tempx < 0) continue;
            for(int j = -1; j<2; ++j){
                tempy = y +j;
                if(tempy < 0) continue;
                for(int k = -3; k<2; ++k){
                    tempz = z +k;
                    if(tempz < 0)continue;
                    Eigen::Vector3i pt(tempx, tempy, tempz);
                    walls.insert(pt);
                }
            }
        }
    }

}   // ReadConfiguration


int EnvironmentCONTXYZYAW::SetStart(double x_m, double y_m, double z_m, double yaw_r,
        double vxy, double vz, double w){

    if(!IsWithinMapCell(x_m,y_m,z_m)){
        SBPL_ERROR("ERROR: trying to set a start cell %.3f %.3f %.3f that is outside of map\n",x_m , y_m, z_m);
        return -1;
    }
    SBPL_PRINTF("env: setting start to %.3f %.3f %.3f %.3f \n", x_m, y_m, z_m, yaw_r);

    if( !IsValidCell(x_m, y_m, z_m ) ){
        SBPL_PRINTF("WARNING: start configuration %.3f %.3f %.3f is invalid\n", x_m, y_m, z_m, yaw_r);
    }


    EnvCONTXYZYAWHashEntry_t* OutHashEntry;
    if(nullptr == (OutHashEntry = GetHashEntry(x_m, y_m, z_m, yaw_r,vxy,vz,w)) ){
        printf("creating new start.\n");
        OutHashEntry=CreateNewHashEntry(x_m, y_m, z_m, yaw_r,vxy,vz,w);
    }

    if(EnvCONTXYZYAW.startstateid != OutHashEntry->stateID ){
        bNeedtoRecomputeStartHeuristics = true;
        // because termination condition can be not all states TODO - make it dependent on term. condition
        bNeedtoRecomputeGoalHeuristics = true;
    }

    // set start
    EnvCONTXYZYAW.startstateid = OutHashEntry->stateID;
    EnvCONTXYZYAWCfg.StartX = x_m;
    EnvCONTXYZYAWCfg.StartY = y_m;
    EnvCONTXYZYAWCfg.StartZ = z_m;
    EnvCONTXYZYAWCfg.StartYaw = yaw_r;
    EnvCONTXYZYAWCfg.StartVXY = vxy;
    EnvCONTXYZYAWCfg.StartVZ = vz;
    EnvCONTXYZYAWCfg.StartW = w;

    return EnvCONTXYZYAW.startstateid;

}   // SetStart


int EnvironmentCONTXYZYAW::SetGoal(double x_m, double y_m, double z_m, double yaw_r,
        double vxy, double vz, double w){

    if(!IsWithinMapCell(x_m,y_m,z_m)){
        SBPL_ERROR("ERROR: trying to set a goal cell %.3f %.3f %.3f that is outside of map\n",x_m , y_m, z_m);
        return -1;
    }
    SBPL_PRINTF("env: setting goal to %.3f %.3f %.3f %.3f \n", x_m, y_m, z_m, yaw_r);

    if( !IsValidCell(x_m, y_m, z_m) ){
        SBPL_PRINTF("WARNING: goal configuration %.3f %.3f %.3f is invalid\n", x_m, y_m, z_m, yaw_r);
    }


    EnvCONTXYZYAWHashEntry_t* OutHashEntry;
    if(nullptr == (OutHashEntry = GetHashEntry(x_m, y_m, z_m, yaw_r,vxy,vz,w)) ){
        printf("creating new goal.\n");
        OutHashEntry=CreateNewHashEntry(x_m, y_m, z_m, yaw_r,vxy,vz,w);
    }

    if(EnvCONTXYZYAW.goalstateid != OutHashEntry->stateID ){
        // because termination condition may not plan all the way to the new goal
        bNeedtoRecomputeStartHeuristics = true;
        // because goal heuristic changes 
        bNeedtoRecomputeGoalHeuristics = true;
    }

    // set goal 
    OutHashEntry->parentStateID = -10;
    EnvCONTXYZYAW.goalstateid = OutHashEntry->stateID;
    EnvCONTXYZYAWCfg.EndX = x_m;
    EnvCONTXYZYAWCfg.EndY = y_m;
    EnvCONTXYZYAWCfg.EndZ = z_m;
    EnvCONTXYZYAWCfg.EndYaw = yaw_r;
    EnvCONTXYZYAWCfg.EndVXY = vxy;
    EnvCONTXYZYAWCfg.EndVZ = vz;
    EnvCONTXYZYAWCfg.EndW = w;

    debug << OutHashEntry->X << " "
        << OutHashEntry->Y << " "
        << OutHashEntry->Z << " "
        << OutHashEntry->Yaw << std::endl;


    
    // set sample goal region
    bias.rangex = bias.pct * EnvCONTXYZYAWCfg.cellsize_m * EnvCONTXYZYAWCfg.EnvLength; 
    bias.rangey = bias.pct * EnvCONTXYZYAWCfg.cellsize_m * EnvCONTXYZYAWCfg.EnvWidth; 
    bias.rangez = bias.pct * EnvCONTXYZYAWCfg.cellsize_m * EnvCONTXYZYAWCfg.EnvHeight; 
    bias.rangeyaw = bias.pct * M_PI * 2.0;
    bias.x_ = EnvCONTXYZYAWCfg.EndX - bias.rangex/2.0;
    bias.y_ = EnvCONTXYZYAWCfg.EndY - bias.rangey/2.0;
    bias.z_ = EnvCONTXYZYAWCfg.EndZ - bias.rangez/2.0;
    bias.yaw_ = EnvCONTXYZYAWCfg.EndYaw - bias.rangeyaw/2.0;


    //ComputeHeuristicValues();
    return EnvCONTXYZYAW.goalstateid;

}   // SetGoal

bool EnvironmentCONTXYZYAW::IsWithinMapCell(double x, double y, double z){
    return ( x>= 0 && x < (int)EnvCONTXYZYAWCfg.EnvLength &&
            y>= 0 && y < (int)EnvCONTXYZYAWCfg.EnvWidth &&
            z>= 0 && z < (int)EnvCONTXYZYAWCfg.EnvHeight);  
}

void EnvironmentCONTXYZYAW::EnsureHeuristicsUpdated(bool bGoalHeuristics){

    // backward search
    if(bNeedtoRecomputeStartHeuristics && !bGoalHeuristics){

        int startx, starty, startz;
        startx = CONTXY2DISC(EnvCONTXYZYAWCfg.StartX, EnvCONTXYZYAWCfg.cellsize_m);
        starty = CONTXY2DISC(EnvCONTXYZYAWCfg.StartY, EnvCONTXYZYAWCfg.cellsize_m);
        startz = CONTXY2DISC(EnvCONTXYZYAWCfg.StartZ, EnvCONTXYZYAWCfg.cellsize_m);


        bNeedtoRecomputeStartHeuristics = false;
        //SBPL_PRINTF("2dsolcost_infullunits=%d\n", (int)(grid2Dsearchfromstart->getlowerboundoncostfromstart_inmm(
        //EnvNAVXYTHETACCfg.StartX_c, EnvNAVXYTHETACCfg.StartY_c) / EnvNAVXYTHETACCfg.nominalvel_mpersecs));
    }

    // forwardsearch
    if(bNeedtoRecomputeGoalHeuristics && bGoalHeuristics){

        int goalx, goaly, goalz;
        goalx = CONTXY2DISC(EnvCONTXYZYAWCfg.EndX, EnvCONTXYZYAWCfg.cellsize_m);
        goaly = CONTXY2DISC(EnvCONTXYZYAWCfg.EndY, EnvCONTXYZYAWCfg.cellsize_m);
        goalz = CONTXY2DISC(EnvCONTXYZYAWCfg.EndZ, EnvCONTXYZYAWCfg.cellsize_m);


        bNeedtoRecomputeGoalHeuristics = false;
        //SBPL_PRINTF("2dsolcost_infullunits=%d\n", (int)(grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(
        //EnvNAVXYTHETACCfg.StartX_c, EnvNAVXYTHETACCfg.StartY_c) / EnvNAVXYTHETACCfg.nominalvel_mpersecs));
    }
}



int EnvironmentCONTXYZYAW::GetFromToHeuristic(int FromStateID, int ToStateID){
    //heuristic is always used 
#if USE_HEUR == 0
    return 0;
#endif

#if DEBUG
    if( FromStateID >= (int)StateID2CoordTable.size() ||
            ToStateID >= (int)StateID2CoordTable.size() ){
        SBPL_ERROR("ERROR in EnvCONTXYZYAW... function: stateID illegal\n");
        throw SBPL_Exception("");
    }
#endif

    // get x,y,z,yaw for the state
    EnvCONTXYZYAWHashEntry_t* FromHashEntry = StateID2CoordTable[FromStateID];
    EnvCONTXYZYAWHashEntry_t* ToHashEntry = StateID2CoordTable[ToStateID];

    // check if one of the gridsearches already computed and then use it.

    return (int)(CONTXYZYAW_COSTMULT_MTOMM *
            EuclideanDistance_m(FromHashEntry->X, FromHashEntry->Y, FromHashEntry->Z,
                ToHashEntry->X, ToHashEntry->Y, ToHashEntry->Z) /
            EnvCONTXYZYAWCfg.nominalvel_mpersecs);

}   // GetFromToHeuristic 


void EnvironmentCONTXYZYAW::GetCoordFromState(
        int stateID, double& x, double& y, double &z, double &yaw,
        double& vxy, double &vz, double &w) const{
    EnvCONTXYZYAWHashEntry_t* HashEntry = StateID2CoordTable[stateID];
    x = HashEntry->X;
    y = HashEntry->Y;
    z = HashEntry->Z;
    yaw = HashEntry->Yaw;
    vxy = HashEntry->VXY;
    vz = HashEntry->VZ;
    w = HashEntry->W;
}

int EnvironmentCONTXYZYAW::GetStateFromCoord(
        double x, double y, double z, double yaw, double vxy, double vz, double w){
    EnvCONTXYZYAWHashEntry_t* OutHashEntry;
    if( nullptr == (OutHashEntry = GetHashEntry(x,y,z,yaw,vxy,vz,w)) ){
        OutHashEntry = CreateNewHashEntry(x,y,z,yaw,vxy,vz,w);
    }
    return OutHashEntry->stateID;
}

int EnvironmentCONTXYZYAW::GetGoalHeuristic(int stateID){
    return 0;
}   // GetGoalHeuristic


double EnvironmentCONTXYZYAW::EuclideanDistance_m(
        double x1, double y1, double z1, double x2, double y2, double z2){

    double sqdist = (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2);
    return std::sqrt(sqdist);
}


// when cell (0,0) changes its status it also does the same for the 3D states
// whose incoming actions are potentially affected when cell (0,0) changes its
// status
void EnvironmentCONTXYZYAW::ComputeReplanningData() {}

bool EnvironmentCONTXYZYAW::IsWall(int x_, int y_, int z_) {
    Eigen::Vector3i p(x_, y_, z_); 
    return walls.count(p);
}

bool EnvironmentCONTXYZYAW::IsValidCell(double x, double y, double z){
    int discx, discy, discz;
    discx = CONTXY2DISC(x, EnvCONTXYZYAWCfg.cellsize_m);
    discy = CONTXY2DISC(y, EnvCONTXYZYAWCfg.cellsize_m);
    discz = CONTXY2DISC(z, EnvCONTXYZYAWCfg.cellsize_m);


    //std::cout << "discxyz: " << discx << " " << discy << " " << discz << std::endl;

    return ( !IsWall(discx, discy, discz) &&
            discx >= 0 && discx < EnvCONTXYZYAWCfg.EnvLength &&
            discy >= 0 && discy < EnvCONTXYZYAWCfg.EnvWidth &&
            discz >= 0 && discz < EnvCONTXYZYAWCfg.EnvHeight  
           );
}

bool EnvironmentCONTXYZYAW::IsValidCell(int discx, int discy, int discz) {
    std::cout << "IsValidCell discretized version is called!" << std::endl;
    return false;
}

// normalizeDiscAngle
EnvCONTXYZYAWHashEntry_t*
EnvironmentCONTXYZYAW::GetHashEntry(double x, double y, double z, double yaw,
        double vxy, double vz, double w){
#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    auto key = hashkey(x, y, z, yaw, vxy, vz, w);
    auto search = Coord2StateIDHashTable.find(key);

    if(search != Coord2StateIDHashTable.end()){
        return search->second;
    }

#if TIME_DEBUG
    time_gethash += clock()-currenttime;
#endif

    return nullptr;
}

EnvCONTXYZYAWHashEntry_t*
EnvironmentCONTXYZYAW::CreateNewHashEntry(double x, double y, double z, double yaw,
        double vxy, double vz, double w){
#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    EnvCONTXYZYAWHashEntry_t* HashEntry = new EnvCONTXYZYAWHashEntry_t();
    HashEntry->X = x;
    HashEntry->Y = y;
    HashEntry->Z = z;
    HashEntry->Yaw = yaw;
    HashEntry->VXY = vxy;
    HashEntry->VZ = vz;
    HashEntry->W = w;
    
    // insert into NNtable
    int discx, discy, discz;
    discx = CONTXY2DISC(x, EnvCONTXYZYAWCfg.cellsize_m);
    discy = CONTXY2DISC(y, EnvCONTXYZYAWCfg.cellsize_m);
    discz = CONTXY2DISC(z, EnvCONTXYZYAWCfg.cellsize_m);


    // create a new id, which is the last elements's index in StateID2CoordTable
    HashEntry->stateID = StateID2CoordTable.size();
    StateID2CoordTable.push_back(HashEntry);

    Coord2StateIDHashTable[hashkey(x,y,z,yaw,vxy,vz,w)] = HashEntry;

    // I do not know what this is @_@
    // insert into initialize the mappings 
    int* entry = new int[NUMOFINDICES_STATEID2IND];
    StateID2IndexMapping.push_back(entry);

    for(int i=0; i<NUMOFINDICES_STATEID2IND; ++i){
        StateID2IndexMapping[HashEntry->stateID][i] = -1;
    }
    if(HashEntry->stateID != (int)StateID2IndexMapping.size()-1){
        throw SBPL_Exception("ERROR in Env... function: last state has incorrect stateID");
    }


#if TIME_DEBUG
    time_createhash += clock()-currenttime;
#endif

    return HashEntry;

}   // CreateNewHashEntry


int EnvironmentCONTXYZYAW::GetActionCost(double SourceX, double SourceY, double SourceZ,
        double SourceYaw, EnvCONTXYZYAWAction_t* action) {}

void EnvironmentCONTXYZYAW::GetSuccs(
        int SourceStateID, 
        std::vector<int> *SuccIDV,
        std::vector<int> *CostV,
        std::vector<EnvCONTXYZYAWAction_t*>* actionV) {

#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    int aind;

    SuccIDV->clear();
    CostV->clear();

    if(actionV != nullptr){
        actionV->clear();
        actionV->reserve(EnvCONTXYZYAWCfg.actionwidth);
    }

    if(SourceStateID == EnvCONTXYZYAW.goalstateid) return;

    // predcessor state
    EnvCONTXYZYAWHashEntry_t* HashEntry = StateID2CoordTable[SourceStateID];

    std::vector<double> sourceState = {HashEntry->X, HashEntry->Y, HashEntry->Z,
                        HashEntry->Yaw, HashEntry->VXY, HashEntry->VZ, HashEntry->W};

    std::vector<double> endState;
    std::vector<sbpl_xyz_rpy_pt_t> interpts;

    EnvCONTXYZYAWAction_t* xyzyawaction;
    bool validcell=true;
    int cost;                                       // -1 if the action is illegal


    for (aind = 0; aind < EnvCONTXYZYAWCfg.actionwidth; ++aind) {
        endState.clear();

        EnvAction->GetTrajAndCost(aind, sourceState, endState, interpts, cost);

        if (-1 == cost) {
            continue;
        }

        for (auto temp:interpts) {

            if(!IsValidCell( (double)temp.x, (double)temp.y, (double)temp.z )) {
                validcell = false;
                break;
            }

        }

        if(!validcell) {
            validcell = true;
            continue;
        }
    
        // then its a valid successor
        EnvCONTXYZYAWHashEntry_t* OutHashEntry;
        if (IsWithinGoalRegion(endState[0],endState[1],endState[2],endState[3])){

            OutHashEntry = StateID2CoordTable[EnvCONTXYZYAW.goalstateid];
            OutHashEntry->parentStateID = SourceStateID;

        }else{

            if (nullptr == (OutHashEntry = GetHashEntry(endState[0], endState[1],
                            endState[2], endState[3],
                            endState[4], endState[5],
                            endState[6]) )){

                OutHashEntry = CreateNewHashEntry(endState[0], endState[1],
                        endState[2], endState[3],
                        endState[4], endState[5],
                        endState[6] ); 
                OutHashEntry->parentStateID = SourceStateID;
                //OutHashEntry->actIdx = aind;
            }
        }

        // all success: new and old;
        SuccIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);

        xyzyawaction = new EnvCONTXYZYAWAction_t((unsigned char) aind, sourceState[3], 
                endState[3], cost);
        xyzyawaction->intermptV = std::move(interpts);

        if ( nullptr != actionV){
            actionV->push_back(xyzyawaction);
        }
    }


#if TIME_DEBUG
	time_getsuccs += clock()-currenttime;
#endif

}   // GetSuccs


int EnvironmentCONTXYZYAW::GetSuccs(int parentID, int sampleStateID) {

    std::vector<int> SuccIDV;
    std::vector<int> CostV;
    std::vector<EnvCONTXYZYAWAction_t*> actionV;

    //std::cout << "parentID: " << parentID << std::endl;
    
    GetSuccs(parentID, &SuccIDV, &CostV, &actionV);

    if (0 ==SuccIDV.size()) {
        return -1;
    }

    int stateid = -1;
    double dist = 1000;
    for (auto i : SuccIDV) {
        double d = GetDistance(i, sampleStateID);
        if (d < dist ) {
            dist =  d;
            stateid = i;
        }
    }
    
    assert(-1 != stateid);
    auto HashEntry = StateID2CoordTable.at(stateid);
    treeNodes.nodes.push_back(HashEntry);
    int size = treeNodes.kdtree_get_point_count();
    kdtree->addPoints((size_t)size-1, (size_t)size-1);

    debug << HashEntry->X << " "
        << HashEntry->Y << " "
        << HashEntry->Z << " "
        << HashEntry->Yaw << std::endl;

    return stateid;
}


void EnvironmentCONTXYZYAW::GetSuccs(
        int SourceStateID, 
        std::vector<int>* SuccIDV, 
        std::vector<int>* CostV){
        GetSuccs(SourceStateID, SuccIDV, CostV, nullptr);
}

void EnvironmentCONTXYZYAW::PrintState(int stateID, bool bVerbose, FILE *fOut){
#if DEBUG
    if(stateID >= (int)StateID2CoordTable.size()){
        SBPL_ERROR("ERROR in EnvCONTXYZYAW... function: stateID illegal (2)\n");
        throw SBPL_Exception();
    }
#endif
    
    //std::ofstream output;
    //output.open("debug_state.txt", std::ios_base::app);

    if(nullptr == fOut){
        fOut = stdout;
    }

    EnvCONTXYZYAWHashEntry_t* HashEntry = StateID2CoordTable[stateID];

    if(stateID == EnvCONTXYZYAW.goalstateid && bVerbose){
        SBPL_FPRINTF(fOut, "the state is a goal state\n");
    }

    if(bVerbose){
        SBPL_FPRINTF(fOut, "X=%f Y=%f Z=%f Yaw=%f VXY=%f VZ=%f W=%f \n", 
        HashEntry->X, HashEntry->Y, HashEntry->Z, HashEntry->Yaw, HashEntry->VXY, HashEntry->VZ, HashEntry->W);

        //output<< HashEntry->X <<" "<< HashEntry->Y << " "<< HashEntry->Z << " " << HashEntry->Yaw << std::endl;

        //output.close();
    }else{
        SBPL_FPRINTF(fOut, "%.3f %.3f %.3f %.3f\n", HashEntry->X,
                HashEntry->Y, HashEntry->Z, HashEntry->Yaw);
    }

}   // PrintState


void EnvironmentCONTXYZYAW::ConvertStateIDPathintoXYZYawPath(
        std::vector<int>* stateIDPath, std::vector<sbpl_xyz_rpy_pt_t>* xyzyawPath){

    int totalcost = 0;

    xyzyawPath->clear();
    stateIDPath->clear();
    auto goalentry = StateID2CoordTable[EnvCONTXYZYAW.goalstateid];
    if (-10 == goalentry->parentStateID ) {
        std::cout << "goal is not reached.\n";
        return;
    }
    
    EnvCONTXYZYAWHashEntry_t* tempEntry = goalentry;
    // then goal is expanded and do backtrack
    while (tempEntry->stateID != EnvCONTXYZYAW.startstateid) {
        stateIDPath->push_back(tempEntry->stateID); 
        tempEntry = StateID2CoordTable[tempEntry->parentStateID];
    }

    std::vector<int> CostV;
    std::vector<int> SuccIDV;
    std::vector<EnvCONTXYZYAWAction_t*> actionV;

    for (auto i = stateIDPath->size() - 1; i > 0; --i) {
        CostV.clear();
        SuccIDV.clear();
        actionV.clear();

        int sourceID = stateIDPath->at(i);
        int targetID = stateIDPath->at(i-1);
        GetSuccs(sourceID, &SuccIDV, &CostV, &actionV);

        bool found = false;
        for (auto i = 0; i < SuccIDV.size(); ++i) {
            if(targetID == SuccIDV[i]){
                totalcost += CostV[i];
                for (auto pt : actionV[i]->intermptV) {
                    xyzyawPath->push_back(pt);
                }
                found = true;
                break;
            }
        }

        if(targetID == EnvCONTXYZYAW.goalstateid) {
            int closest = -1;
            double dist = 10000;
            for (auto i = 0; i < SuccIDV.size(); ++i) {
                double d = GetDistance(EnvCONTXYZYAW.goalstateid, SuccIDV[i]);
                if(dist > d) {
                    dist = d;
                    closest = i;
                }
            }

            totalcost += CostV[closest];
            for (auto pt : actionV[closest]->intermptV) {
                xyzyawPath->push_back(pt);
            }

            std::cout << "solution cost: " << totalcost << std::endl;
            std::cout << "Number of samples: " << sample_times << std::endl;
            found = true;
            break;
        }


        if (!found && targetID != EnvCONTXYZYAW.goalstateid) {
            double targetx, targety, targetz, targetyaw, targetvxy, targetvz, targetw;
            double sourcex, sourcey, sourcez, sourceyaw, sourcevxy, sourcevz, sourcew;

            GetCoordFromState(sourceID, sourcex, sourcey, sourcez, sourceyaw,
                    sourcevxy, sourcevz, sourcew);
            GetCoordFromState(targetID, targetx, targety, targetz, targetyaw,
                    targetvxy, targetvz, targetw);
            std::cout<<sourceID << "  "<< targetID << std::endl;
            std::cout<<sourcex<<" "<< sourcey<<" "<<sourcez<<" "<< sourceyaw<<" -> "
                << targetx << " "<< targety<<" "<< targetz<<" "<< targetyaw<<std::endl;
            auto temp = StateID2CoordTable[targetID];
            std::cout << "parent of target: " << temp->parentStateID << std::endl;


            throw SBPL_Exception("ERROR: successor not found for transition");
        }
        
    }
}       // ConvertStateIDPathintoXYZYawPath


int EnvironmentCONTXYZYAW::SizeofCreatedEnv(){
    return (int)StateID2CoordTable.size();
}

void EnvironmentCONTXYZYAW::PrintEnv_Config(FILE* fOut){
    // implement this if the planner needs to print out EnvCONTXYZYAW.configuartion
    throw SBPL_Exception("ERROR in EnvCONTXYZYAW... function: PrintEnv_Config is undefined");
}

void EnvironmentCONTXYZYAW::PrintTimeStat(FILE* fOut){
#if TIME_DEBUG
   SBPL_FPRINTF(fOut, "time_neighbor = %f secs, time_gethash = %f secs, time_createhash = %f secs, "
				"time_getsuccs = %f\n",
				time_neighbor/(double)CLOCKS_PER_SEC, time_gethash/(double)CLOCKS_PER_SEC,
				time_createhash/(double)CLOCKS_PER_SEC, time_getsuccs/(double)CLOCKS_PER_SEC);
#endif
   //printf( "time_neighbor = %f secs, time_gethash = %f secs, time_createhash = %f secs, "
                //"time_getsuccs = %f\n",
                //time_neighbor/(double)CLOCKS_PER_SEC, time_gethash/(double)CLOCKS_PER_SEC,
                //time_createhash/(double)CLOCKS_PER_SEC, time_getsuccs/(double)CLOCKS_PER_SEC);
//#endif
}

void EnvironmentCONTXYZYAW::PrintHashTableHist(FILE* Out) {}

void EnvironmentCONTXYZYAW::SetGoalTolerance(double tol_x, double tol_y, double tol_z, double tol_yaw){
    EnvCONTXYZYAWCfg.tol_x = tol_x;
    EnvCONTXYZYAWCfg.tol_y = tol_y;
    EnvCONTXYZYAWCfg.tol_z = tol_z;
    EnvCONTXYZYAWCfg.tol_yaw = tol_yaw;
}

bool EnvironmentCONTXYZYAW::IsWithinGoalRegion(int stateID) {
    assert(stateID < StateID2CoordTable.size());
    
    auto HashEntry = StateID2CoordTable[stateID];

    return IsWithinGoalRegion(HashEntry->X, HashEntry->Y, HashEntry->Z, HashEntry->Yaw);

}


// meter meter meter rad
bool EnvironmentCONTXYZYAW::IsWithinGoalRegion( double x, double y, double z, double yaw){
    if( std::abs(x-EnvCONTXYZYAWCfg.EndX) < EnvCONTXYZYAWCfg.tol_x &&
        std::abs(y-EnvCONTXYZYAWCfg.EndY) < EnvCONTXYZYAWCfg.tol_y &&
        std::abs(z-EnvCONTXYZYAWCfg.EndZ) < EnvCONTXYZYAWCfg.tol_z &&
        std::abs(yaw - EnvCONTXYZYAWCfg.EndYaw) < EnvCONTXYZYAWCfg.tol_yaw){
        return true;
    }
    return false;
}


double EnvironmentCONTXYZYAW::GetDistance(int stateID1, int stateID2) {
    auto HashEntry1 = StateID2CoordTable.at(stateID1);
    auto HashEntry2 = StateID2CoordTable.at(stateID2);

    double diff0 = HashEntry1->X - HashEntry2->X;
    double diff1 = HashEntry1->Y - HashEntry2->Y;
    double diff2 = HashEntry1->Z - HashEntry2->Z;
    double result = diff0*diff0 + diff1*diff1 + diff2*diff2;
    
    result += 0.0020*( 1-std::cos(HashEntry1->Yaw - HashEntry2->Yaw) );

    return result;
}


int EnvironmentCONTXYZYAW::GetGoalStateID() {
    return EnvCONTXYZYAW.goalstateid;
}
int EnvironmentCONTXYZYAW::GetStartStateID() {
    return EnvCONTXYZYAW.startstateid;
}