#ifndef TS_FUZZY_H
#define TS_FUZZY_H

#include <Arduino.h>
#include <math.h>

// Angle constants in radians
#define DEG_2 radians(2.0f)
#define DEG_5 radians(5.0f)

class TS_Fuzzy {
public:
    // Constructor
    TS_Fuzzy();

    // Set the width (sigma) of the Gaussian MFs in degrees
    void setSigmaDegrees(float deg);

    // Set controller gains for 3-rule fuzzy system
    void set3Gains(const float K1[4], const float K2[4], const float K3[4]);

    // Set controller gains for 5-rule fuzzy system
    void set5Gains(const float K1[4], const float K2[4], const float K3[4],
                   const float K4[4], const float K5[4]);

    // Compute control input using 3 Gaussian MFs
    float computeControl3mf(float theta, float thetaRate, float pos, float vel);

    // Compute control input using 5 Gaussian MFs
    float computeControl5mf(float theta, float thetaRate, float pos, float vel);

    void compute3Memberships(float theta, float &h1, float &h2, float &h3);

private:
    // Compute 3 normalized membership values

    // Compute 5 normalized membership values
    void compute5Memberships(float theta, float &h1, float &h2, float &h3, float &h4, float &h5);

    float K[5][4];   // Feedback gains: [rule][state]
    float sigma;     // Width of Gaussian membership functions (in radians)
};

#endif // TS_FUZZY_H
