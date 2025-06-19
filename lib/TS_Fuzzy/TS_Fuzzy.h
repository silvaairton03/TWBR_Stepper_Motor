#ifndef TS_FUZZY_H
#define TS_FUZZY_H

#include <Arduino.h>
#include <math.h>


class TS_Fuzzy {
public:
    enum Mode {MF3 = 3, MF5 = 5};
    // Constructor
    TS_Fuzzy();

    void setSigmaDegrees(float deg);
    void setMode(Mode m);

    void set3Gains(const float K1[4], const float K2[4], const float K3[4]);

    void set5Gains(const float K1[4], const float K2[4], const float K3[4],
                   const float K4[4], const float K5[4]);

    float computeControl(float theta, float thetaRate, float pos, float vel);

    
private:
    void compute3Memberships(float theta, float &h1, float &h2, float &h3);
    
    void compute5Memberships(float theta, float &h1, float &h2, float &h3, float &h4, float &h5);
    float K[5][4]; 
    float sigma;
    float invSigmaSq;
    Mode mode;
    
    static constexpr float DEG_2 = 0.034906585f;
    static constexpr float DEG_5 = 0.087266462f;
};

#endif // TS_FUZZY_H
