#ifndef TS_FUZZY_H
#define TS_FUZZY_H

#include<Arduino.h>

class TS_Fuzzy{
public:
    TS_Fuzzy();

    void setGains(const float* k1, const float* k2, const float* k3,
        const float* k4, const float* k5);

    float computeControl(float theta, float thetaRate, float pos, float vel);

private:
    float K[5][4];      // Matriz de ganhos para 5 modelos (5 regras), cada um com 4 variáveis de estado
    float sigma;

    void computeMemberships(float theta, float& h1, float& h2, float& h3, float& h4, float& h5);
};

#endif