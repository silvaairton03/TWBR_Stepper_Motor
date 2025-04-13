#ifndef TS_FUZZY_H
#define TS_FUZZY_H

#include<Arduino.h>
#include <math.h>

class TS_Fuzzy{
public:
    TS_Fuzzy();

    void set3Gains(const float* k1, const float* k2, const float* k3);

    void set5Gains(const float* k1, const float* k2, const float* k3,
        const float* k4, const float* k5);
        
    void set7Gains(const float* k1, const float* k2, const float* k3,
        const float* k4, const float* k5, const float* k6, const float* k7);    

    float computeControl3mf(float theta, float thetaRate, float pos, float vel);
    float computeControl5mf(float theta, float thetaRate, float pos, float vel);
    float computeControl7mf(float theta, float thetaRate, float pos, float vel);

private:
    float K[7][4];       // Matriz de ganhos para 5 modelos (5 regras), cada um com 4 variáveis de estado
    float sigma;
    
    void compute3Memberships(float theta, float& h1, float& h2, float& h3);
    void compute5Memberships(float theta, float& h1, float& h2, float& h3, float& h4, float& h5);
    void compute7Memberships(float theta, float& h1, float& h2, float& h3, float& h4, float& h5, float& h6, float& h7);

    const  float DEG_30 = radians(30.0f);
    const float DEG_45 = radians(45.0f);
    const float DEG_60 = radians(60.0f);
};

#endif