/*
 *  A motion primitive class of an airplane.
 *  Given the start position, speed, direction of the airplane then the possible
 *  trajectories are generated with corresponding actioncost.
 *
 *  States should be compatible with environment: EnvCONTXYZYAW
 *  
 *  Author: Wei Du
 *  Version: 1.0
 *  Date: June/11/2018
 *
 */

/***********************************************************************************
 The source state contains : x,y,z,yaw,vx,vy,vz,w
 
 Possible discretized trajectories and speeds should be generated.

 maxmium and minimum speed constraints are checked here.
 the validation of the actual trajectory are verified in the environment.

 angular speeds are in a set as constants {-1, 0 , 1}, as well as speed in z-axis 

 speed in x,y are controlled by a set of acceleration constants{-1, 0, 1}

 no turn in place and climb in place allowed.

 speed is defined in km/s   rad/s

************************************************************************************/


#include <vector>
#include <cmath>
#include <sbpl/config.h>
#include <sbpl/utils/utils.h>

//#define ANGLECHANGE M_PI/8.1
#define ANGLECHANGE M_PI/20.1
struct action{
    double xyacc;
    double vz;
    double w;

    int cost;
};


// set yaw value between 0 and 2*pi
double NormalizeYaw(double yaw){

    if (yaw > M_PI *2)   return ( yaw -= M_PI *2 );

    if(yaw < 0)     return ( yaw += M_PI *2 );

    return yaw;
}



class AirplaneAction{
public:
    int numofactions;

    AirplaneAction();
    void InitializeAction();

    // return cost value -1 if action is illegal
    void GetTrajAndCost(int actionIdx,
                        const std::vector<double> &sourceState,
                              std::vector<double> &endState,
                              std::vector<sbpl_xyz_rpy_pt_t>& interpts,
                              int &cost);

    double GetMaxSpeed(){return maxspeed;}

private:

    double maxspeed;
    double minspeed;
    double xyacc;
    double vz;
    double w_r;
    double deltaT;           // time for one action 
    int numofsamples;       // num of discretized samples

    std::vector<action> act;

    // combined speed value
    inline double CmbSpeed(double vxy, double vz);
    inline bool ExceedSpeedBound(double vxy, double vz);   
    inline bool ActionInPlace(int actionIdx, double vxy);
    inline double Euclidist(double x1, double y1, double z1,
                            double x2, double y2, double z2);
};



AirplaneAction::AirplaneAction(){
    deltaT = 0.5;
    numofsamples = 20;

    numofactions = 27;

    // have problem turning around
    maxspeed = 0.060;     // km/s
    minspeed = 0.0280;
    xyacc = 0.015;         // km/s^2
    vz = 0.010;

    //w_r = ANGLECHANGE/deltaT;
    w_r = ANGLECHANGE * 2;

    act.reserve(numofactions);
    InitializeAction();
}

// cost represents the penalty for one action
void AirplaneAction::InitializeAction(){
    int counter = 0;

    for(int i = -1; i<2; ++i){              // xyacc 
        for(int j = -1; j<2; ++j){          // vz
            for(int k= -1; k<2; ++k){       // w

                act[counter].xyacc = i*xyacc;    
                act[counter].vz = j*vz;
                act[counter].w = k*w_r;

                act[counter].cost = (std::abs(i) + std::abs(j) + 2*std::abs(k) ) + 3;
                //act[counter].cost = (std::abs(i) + 2*std::abs(k) ) + 1;

                counter++;
            }
        }
    }
    
}

double AirplaneAction::CmbSpeed(double vxy, double vz){
    return std::sqrt(vxy*vxy + vz*vz);
}

bool AirplaneAction::ExceedSpeedBound(double vxy, double vz){
    //double speed = CmbSpeed(vxy,vz);
    double speed = std::abs(vxy);

    return ( speed > maxspeed || speed < minspeed);
}


// condition: the projection on xoy plane is not moving
bool AirplaneAction::ActionInPlace(int actionIdx, double vxy){
   
    //action currentAction = act[actionIdx];

    if(vxy <= 0.000001 && std::abs(act[actionIdx].xyacc) <= 0.000001) return true;

    return false;
}


// source and end state should contain x, y, z, yaw, vxy, vz, w;
void AirplaneAction::GetTrajAndCost(int actionIdx,
        const std::vector<double> &sourceState,
        std::vector<double> &endState,
        std::vector<sbpl_xyz_rpy_pt_t> &interpts,
        int &cost){

#if DEBUG
    if (7 != sourceState.size()){
        SBPL_ERROR("ERROR: Wrong number of source state elements for action.");
    }
#endif

    endState.clear();
    interpts.clear();
    endState.reserve(numofsamples);
    interpts.reserve(numofsamples);

    action currentAction = act[actionIdx];
    double tempx = sourceState[0];
    double tempy = sourceState[1];
    double tempz = sourceState[2];
    double tempyaw = sourceState[3];
    double tempvxy = sourceState[4];
    
    // check if the action is turn in place or climb in place
    if( ActionInPlace(actionIdx, tempvxy) ){
        cost = -1;
        return;
    }

    double dt = deltaT/numofsamples;

    // insert source point
    // x, y, z, roll, pitch, yaw   // roll = pitch = 0
    sbpl_xyz_rpy_pt_t currentpt(tempx, tempy, tempz,0,0, tempyaw );
    interpts.push_back(currentpt);

    double diff_acc = currentAction.xyacc * dt;
    double trans_acc = 0.5*dt*diff_acc;
    double trans_z = currentAction.vz * dt;
    double rota_yaw = currentAction.w * dt;
    double tempcost = 0;


    // calculation 
    for (int i = 0; i<numofsamples; ++i){
        // update the position
        double cos = std::cos(tempyaw);
        double sin = std::sin(tempyaw);

        currentpt.x = tempx + tempvxy*cos * dt + trans_acc*cos;
        currentpt.y = tempy + tempvxy*sin * dt + trans_acc*sin;
        currentpt.z = tempz + trans_z;
        currentpt.yaw = NormalizeYaw( tempyaw + rota_yaw );

        tempvxy += diff_acc;

        if( ExceedSpeedBound(tempvxy, currentAction.vz) ){
            cost = -1;
            return;
        }

        tempcost += Euclidist(tempx, tempy, tempz, currentpt.x, currentpt.y, currentpt.z);
        // update and goto next round
        tempx = currentpt.x;
        tempy = currentpt.y;
        tempz = currentpt.z;
        tempyaw = currentpt.yaw;
        interpts.push_back(currentpt);
    }

    //cost = currentAction.cost;
    cost =(int)(tempcost *1000);

    endState[0] = tempx;
    endState[1] = tempy;
    endState[2] = tempz;
    endState[3] = tempyaw;
    endState[4] = tempvxy;
    endState[5] = currentAction.vz;
    endState[6] = currentAction.w;


}   // GetTrajAndCost

double AirplaneAction:: Euclidist(double x1, double y1, double z1,
        double x2, double y2, double z2){
    double diff1 = x1-x2;
    double diff2 = y1-y2;
    double diff3 = z1-z2;
    return std::sqrt(diff1*diff1 + diff2*diff2 + diff3*diff3);
}

