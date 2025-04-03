#ifndef COUPLEDCONTROLLER_H
#define COUPLEDCONTROLLER_H

#include"TS_Fuzzy.h"

class coupledController{
public:
    coupledController();

    void setFuzzyGains(const float K1[4], const float K2[4], const float K3[4], const float K4[4], const float K5[4]);
    void setDeltaGains(const float Kdelta[2]);
    void updateStates(float theta, float thetaRate, float pos, float vel,
        float delta, float deltaRate);

    void computeTorques(float &Ttheta, float &Tdelta);

private:
    TS_Fuzzy fuzzyController;
    float Kdelta[2];
    float stateTheta[4];
    float stateDelta[2];
};

#endif