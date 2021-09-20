/*
 * This is a environment in continuous space. Based on the sbpl library environment template.
 *
 * It's used for planning in (x,y,z,yaw) space, roll and pitch angles are assumed
 * to be zeros.
 * Robot is assumed to be a point robot.
 *
 * Date: Jun/4/2018
 * By: Wei Du
 *
 * Version: 1.0
 * This version dose not require preprocessed motion primitive file.
 * The successors are generated from actions(which are described by formulations) online.
 *
 */

#ifndef __ENVIRONMENT_XYZYAW__
#define __ENVIRONMENT_XYZYAW__

#include <cstdio>
#include <vector>
#include <ctime>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <fstream>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <sbpl/utils/utils.h>
#include <eigen3/Eigen/Dense>
#include <sbpl/discrete_space_information/environment.h>

#include "motion.h"
#include "nanoflann.hpp"
#include "headers.h"

#define PENALTY 1

#define CNTSPACE_DEFAULTOBSTHRESH 254
// number of actions per x,y,z,yaw state
// x y (acc) z(speed) w(yaw speed)
#define CNTSPACE_DEFAULT_ACTIONWIDTH 27

// yaw increases as we go counterclockwise
// number of yaw values - should be power of 2
#define CONTXYZYAW_YAWDIRS 16
#define ANGLERESOLUTION M_PI/16

#define CONTXYZYAW_COSTMULT_MTOMM 1000

class CMDPSTATE;

//Structures
struct EnvCONTXYZYAWAction_t{
    EnvCONTXYZYAWAction_t():
        aind(0), startYaw(0), endYaw(0), cost(0) {}

    EnvCONTXYZYAWAction_t(unsigned char aind_, double startYaw_, double endYaw_, unsigned int cost_):
        aind(aind_), startYaw(startYaw_), endYaw(endYaw_), cost(cost_) {}


    unsigned char aind;     // index of the action 
    //double dX;
    //double dY;
    //double dZ;
    double startYaw;
    double endYaw;
    
    unsigned int cost;

    std::vector<sbpl_3Dcell_t> intersectingcellsV;

    std::vector<sbpl_xyz_rpy_pt_t> intermptV;

    //std::vector<sbpl_xyz_rpy_cell_t> interm3DcellsV;
};

// for RRT USE
struct EnvCONTXYZYAWBIAS{
    
    // bias sampling region
    // half the total length
    double rangex;
    double rangey;
    double rangez;
    double rangeyaw;

    // start of bounding box
    double x_;
    double y_;
    double z_;
    double yaw_;

    const double pct = 0.05;

};

struct EnvCONTXYZYAWHashEntry_t{
    int stateID;
    int parentStateID;

    double X;
    double Y;
    double Z;
    double Yaw;
    double VXY;
    double VZ;
    double W;        // Yaw angular speed;
    
    //double VX;
    //double VY;
    //int iteration;

    bool operator==(const EnvCONTXYZYAWHashEntry_t &out) const{
        double a = std::abs(X - out.X) + std::abs(Y - out.Y) + std::abs(Z - out.Z) + std::abs(Yaw - out.Yaw)
            + std::abs(VXY - out.VXY) + std::abs(VZ - out.VZ) + std::abs(W - out.W);
        if(a < 0.00001) return true;
        return false;
    }
};

struct KDTreeContainer{
    std::vector<EnvCONTXYZYAWHashEntry_t*> nodes;

    // container
    inline size_t kdtree_get_point_count() const{return  nodes.size();}
   
    // get the elements and calculate the distance
    // take angle into consideration?
    inline double kdtree_get_pt(const size_t idx, int dim)const{
        /*
         *if (0 == dim) return StateID2CoordTable->at(idx)->X;
         *else if (1 == dim) return StateID2CoordTable->at(idx)->Y;
         *else return StateID2CoordTable->at(idx)->Z;
         */

        switch (dim) {
            case 0:
                return nodes.at(idx)->X;
            case 1:
                return nodes.at(idx)->Y;
            case 2:
                return nodes.at(idx)->Z;
            case 3:
                return nodes.at(idx)->Yaw;
                //return  nodes.at(idx)->VX;
            //case 4:
                //return  nodes.at(idx)->VXY;
                //return  nodes.at(idx)->VY;
        }
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb*/) const {return false;}
};

typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<
        nanoflann::L2_Simple_Adaptor<double, KDTreeContainer>,
        KDTreeContainer,
        4/*dim*/ > kd_tree_t;



struct SBPL_xyzyaw_mprimitive{
    int motprimID;

    // roll and pitch are considered to be 0
    unsigned char startyaw;

    int additionalactioncostmult;
    sbpl_xyz_rpy_cell_t endcell;
    double turning_radius;

    // intermptV start at 0,0,0,startYaw and end at endcell in continuous domain
    std::vector<sbpl_xyz_rpy_pt_t> intermptV;
};


struct EnvironmentCONTXYZYAW_t{
    int startstateid;
    int goalstateid;
    bool bInitialized;

    // any additional variables
};

struct EnvCONTXYZYAWConfig_t{
    int EnvLength;  // number of cells in this dimension
    int EnvWidth;
    int EnvHeight;
    int NumYawDirs;     
    
    double StartX;   // position
    double StartY;
    double StartZ;
    double StartYaw;

    double StartVXY;  // velocity
    double StartVZ;
    double StartW;

    double EndX;
    double EndY;
    double EndZ;
    double EndYaw;

    double EndVXY;
    double EndVZ;
    double EndW;


    //std::vector<double> ThetaDirs;
    //double min_turning_radius_m;
    
    double tol_x;    // goal tolerance
    double tol_y;
    double tol_z;
    double tol_yaw;

     /*
      * the value at which and above which cells are obstacles in the maps 
      * sent from outside the default is defined above
      */
    unsigned char obsthresh;

    /*
     * the value at which and above which until obsthresh (not including it)
     * cells have the nearest obstacle at distance smaller than or equal to
     * the inner circle of the robot. In other words, the robot is definitely
     * colliding with the obstacle, independently of its orientation
     */
    unsigned char cost_inscribed_thresh;

    /*
     * the value at which and above until cost_inscribed_thresh (not including it) cells
     * may have a nearest obstacle within the distance that is in between
     * the robot inner circle and the robot outer circle
     * any cost below this value means that the robot will NOT collide with any
     * obstacle, indenpendently of its orientation.
     */
    unsigned char cost_possibly_circumscribed_thresh;
    
    double nominalvel_mpersecs;

    /* may not in use */
    //double timetoturn45degsinplace_secs;
    
    double cellsize_m;

    // array of actions, Actions[i][j] -jth action for sourceyaw = i
    EnvCONTXYZYAWAction_t** ActionsV;

    // PredActionsV[i] -vector of pointers to the actions that result in a state with theta = i
    std::vector<EnvCONTXYZYAWAction_t*>* PredActionsV;

    int actionwidth;    // number of motion primitives

    std::vector<SBPL_xyzyaw_mprimitive> mprimV;

    /* not in use now */
    //std::vector<sbpl_3Dpt_t> FootprintPolyhedron;
};      // configuration


// Eigen::Vector3d setup
template <class T>
std::size_t hashkey_vector3(const T &o){
    std::size_t seed =0;
    boost::hash_combine(seed, o.x());
    boost::hash_combine(seed, o.y());
    boost::hash_combine(seed, o.z());
    return seed;
}

// hash for Eigen::Vector3d
namespace std{
    template<> struct hash<Eigen::Vector3d>{
        std::size_t operator()(Eigen::Vector3d const &s) const noexcept{
            return hashkey_vector3(s);
        }
    };
}

// hash for Eigen::Vector3i
namespace std{
    template<> struct hash<Eigen::Vector3i>{
        std::size_t operator()(Eigen::Vector3i const &s) const noexcept{
            return hashkey_vector3(s);
        }
    };
}



class EnvironmentCONTXYZYAW : public DiscreteSpaceInformation{
public:

    EnvironmentCONTXYZYAW();
   ~EnvironmentCONTXYZYAW();

//-------------- Initialization of the environment -----------------
public:
    /*
     * Initialization of environment from file. Using .cfg file.
     * It also takes the primeter of the robot with respect to some
     * reference point centered at x=0,y=0,z=0 and yaw=0.
     * The perimeter is defined in meters as a sequence of vertices of
     * a polygon defining the perimeter.
     * No motion primitive files needed.
     */
    //virtual bool InitializeEnv(const char* sEnvFile, const std::vector<sbpl_3Dpt_t>& perimeterptsV);
    
    // a pure virtual function from base class
    virtual bool InitializeEnv(const char* sEnvFile){/* not in use */}
    virtual bool InitializeEnv(const char* cfgEnvFile, const char* stlEnvFile);

    // for RRT use
    // return -1 if not successful, or treeid
    virtual int InitTree(int stateID);

    // for RRT use
    // return the stateID
    virtual int GetRandomSample();

    // for RRT use
    // return the stateID, or -1 if the state is not found
    virtual int GetNN(int treeID, int queryStateID);

    // initialization of MDP data structure
    virtual bool InitializeMDPCfg(MDPConfig *MDPCfg){/* not in use yet */}

    // in meters/radians
    virtual int SetStart(double x_m, double y_m, double z_m, double yaw_r,
            double vxy, double vz,  double w);
    
    virtual int SetGoal(double x_m, double y_m, double z_m, double yaw_r,
            double vxy, double vz,  double w);

    virtual void SetGoalTolerance(double tol_x, double tol_y, double tol_z, double tol_yaw);


    // read in obstacle distance transform data
    virtual void ReadinVoronoi(FILE* vfile) {}
    virtual void ReadinTransform(FILE* vfile) {}

    virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state){/* may not in use */};

protected:

    virtual void InitializeEnvironment();

    //virtual bool InitGeneral(std::vector<SBPL_xyzyaw_mprimitive>* motionprimitiveV);
    virtual bool InitGeneral();
    virtual void InitializeEnvConfig();

    virtual void ReadConfiguration(const char* stlEnvFile);
    virtual void ReadinEnvParam(const char* cfgEnvFile);
    //virtual bool ReadinCell(sbpl_xyz_rpy_cell_t* cell, FILE* fIn);
    //virtual bool ReadinPose(sbpl_xyz_rpy_pt_t* pose, FILE* fIn);
    
    //virtual void DeprecatedPrecomputeActions();

    virtual void ComputeReplanningData();
    //virtual void ComputeReplanningDataforAction(EnvCONTXYZYAWAction_t* action);

    // pure virtual function from base class.
    virtual void SetAllPreds(CMDPSTATE* state){/* not defined */}


//--------------------- Planning operations ------------------------
public:

    // heuristic estimate from stat FromStateID to state ToStateID
    virtual int GetFromToHeuristic(int FromStateID, int ToStateID);

    // retuns state coordinates of state with ID=stateID
    virtual void GetCoordFromState(int stateID, double& x, double& y, double& z, double& yaw,
                    double &vxy, double &vz, double&w ) const;

    // returns stateID for a state with coords x,y,z,yaw
    virtual int GetStateFromCoord(double x, double y, double z, double yaw,
                                  double vxy, double vz, double w);

    // heuristic estimate from start state to state with stateID
    virtual int GetStartHeuristic(int stateID){/* not in use */};

    // heuristic estimate from state with stateID to goal state
    virtual int GetGoalHeuristic(int stateID);      // Dijkstra Heuristic

    // virtual int GetPenaltyGoalHeuristic(int stateID);        // Dijkstra Heuristic + Penalty Heuristic
    

    // for RRT USE
    virtual int GetGoalStateID();
    virtual int GetStartStateID();
    
    /*
     * depending on the search used, it may call GetSuccs function or GetPreds function
     * (forward search/backward search). At least one of these functions should be implemented.
     * pure functions from base class.
     */
    virtual void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV);

    // for RRT use
    virtual int GetSuccs(int parentID, int sampleStateID);

    // not in use yet
    virtual void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV){}

    virtual void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV,
                          std::vector<EnvCONTXYZYAWAction_t*>* actionV );
    /*
     * converts a path given by stateIDs into a sequence of coordinates. Note that since motion primitives
     * are short actions represented as a sequence of points, the path returned by this function contains
     * mush more points than the number of points in the input path. The returned coordinates are in 
     * meters, meters, meters, radians
     */
    virtual void ConvertStateIDPathintoXYZYawPath(std::vector<int>* stateIDPath,
                                                  std::vector<sbpl_xyz_rpy_pt_t>* xyzyawPath);
    
    // returns the number of states (hasentries) created
    virtual int SizeofCreatedEnv();

    // prints environment fonfig file
    virtual void PrintEnv_Config(FILE* fOut);

    // prints time stattistics
    virtual void PrintTimeStat(FILE* fOut);


protected:

    virtual int GetActionCost(double SourceX, double SourceY, double SourceZ, double SourceYaw, EnvCONTXYZYAWAction_t* action);
    
    //virtual unsigned int GETHASHBIN(unsigned int X, unsigned int Y, unsigned int Z, unsigned Yaw){[>not in use<] return 0;}

    virtual EnvCONTXYZYAWHashEntry_t* GetHashEntry(double x, double y, double z, double yaw,
                                                        double vxy, double vz, double w);
    
    virtual EnvCONTXYZYAWHashEntry_t* CreateNewHashEntry(double x, double y, double z, double yaw,
                                                         double vxy, double vz, double w);

    virtual void ComputeHeuristicValues();    
    
    virtual int GetTransformValue(double x, double y, double z){return 0;}
    virtual int GetVoronoiValue(double x, double y, double z){return 0;}
    virtual double EuclideanDistance_m(double x1, double y1, double z1, double x2, double y2, double z2);


//------------------------ validation ------------------------------
public:
    virtual void PrintState(int stateID, bool bVerbose, FILE* fOut);

    virtual bool IsWithinGoalRegion(int stateID);

    virtual bool IsWithinMapCell(double x, double y, double z);

    //virtual bool IsValidConfiguration(double x, double y, double z, double yaw);
    
    /*
     *by default the heuristics are up-to-date, but in some cases, the
     *heuristics are computed only when really needed. For example,
     *xytheta environment uses 2D gridsearch as heuristics, and then
     *the number of times heuristics are re-computed which is an expensive operation.
     *if bGoalHeuristics == true, then it updates goal heuristics, otherwise it updates start heuristics.
     */
    virtual void EnsureHeuristicsUpdated(bool bGoalHeuristics);

protected:

    virtual void PrintHashTableHist(FILE* Out);
    virtual bool IsWall(int x_, int y_, int z_);
    virtual bool IsValidCell(double x, double y, double z);
    virtual bool IsValidCell(int discx, int discy, int discz);
    virtual bool IsWithinGoalRegion(double x, double y, double z, double yaw);
    virtual double GetDistance(int stateID1, int stateID2);

//-------------------------- Data storage --------------------------
public:
    EnvCONTXYZYAWConfig_t EnvCONTXYZYAWCfg;

protected:

    AirplaneAction* EnvAction;

    // kt tree
    KDTreeContainer treeNodes;
    kd_tree_t *kdtree;

    // fstream
    std::ofstream debug;

    // obstacles
    std::unordered_set<Eigen::Vector3i> walls;

    EnvironmentCONTXYZYAW_t EnvCONTXYZYAW;
    EnvCONTXYZYAWBIAS bias;

    std::vector<sbpl_xyz_rpy_cell_t> affectedsuccstatesV;    // arrays of states whose outgoing actions cross cell 0,0,0
    std::vector<sbpl_xyz_rpy_cell_t> affectedpredstatesV;    // arrays of states whose outgoing actions cross cell 0,0,0
    int iteration;
    int blocksize;      // 3D block size;
    int bucketsize;     // 3D bucket size;


    // 3D search for heuristic computations
    bool bNeedtoRecomputeStartHeuristics;       // set whenever grid3Dsearchfromstart needs to be re-executed
    bool bNeedtoRecomputeGoalHeuristics;        // set whenever grid3Dsearchfromgoal needs to be re-excuted

    std::unordered_map<std::size_t, EnvCONTXYZYAWHashEntry_t*> Coord2StateIDHashTable;

    
    // vector that maps from stateID to coords
    std::vector<EnvCONTXYZYAWHashEntry_t*> StateID2CoordTable;

};

#endif
