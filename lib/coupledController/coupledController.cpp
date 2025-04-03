#include"coupledController.h"

coupledController::coupledController(){
    memset(Kdelta, 0, sizeof(Kdelta));
    memset(stateTheta, 0, sizeof(stateTheta));
    memset(stateDelta, 0, sizeof(stateDelta));
}

void coupledController::setFuzzyGains(const float K1[4], const float K2[4], const float K3[4], const float K4[4], const float K5[4]){
    fuzzyController.setGains(K1, K2, K3, K4, K5);    
}

void coupledController::setDeltaGains(const float Kd[2]){
    memcpy(Kdelta, Kd, sizeof(Kdelta));
}

void coupledController::updateStates(float theta, float thetaRate, float pos, float vel, float delta, float deltaRate){
    stateTheta[0] = theta;
    stateTheta[1] = thetaRate;
    stateTheta[2] = pos;
    stateTheta[3] = vel;

    stateDelta[0] = delta;
    stateDelta[1] = deltaRate;

}

void coupledController::computeTorques(float &Ttheta, float &Tdelta){
    Ttheta = fuzzyController.computeControl(stateTheta[0], stateTheta[1], stateTheta[2], stateTheta[3]);

    Tdelta = 0.0f;
    for (int i = 0; i < 2; ++i)
        Tdelta += -Kdelta[i] * stateDelta[i];
}