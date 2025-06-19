#include"TS_Fuzzy.h"

TS_Fuzzy::TS_Fuzzy(){
    sigma = radians(20.0f);
    memset(K, 0, sizeof(K));
}

void TS_Fuzzy::setGains(const float K1[4], const float K2[4], const float K3[4], const float K4[4], const float K5[4]){
    memcpy(K[0], K1, sizeof(float) * 4);
    memcpy(K[1], K2, sizeof(float) * 4);
    memcpy(K[2], K3, sizeof(float) * 4);
    memcpy(K[3], K4, sizeof(float) * 4);
    memcpy(K[4], K5, sizeof(float) * 4);
}

void TS_Fuzzy::computeMemberships(float theta, float &h1, float &h2, float &h3, float &h4, float &h5){
    float mf1 = exp(-pow((theta - radians(0.0f)) / sigma, 2));
    float mf2 = exp(-pow((theta - radians(30.0f)) / sigma, 2));
    float mf3 = exp(-pow((theta + radians(30.0f)) / sigma, 2));
    float mf4 = exp(-pow((theta - radians(45.0f)) / sigma, 2));
    float mf5 = exp(-pow((theta + radians(45.0f)) / sigma, 2));

    float total = mf1 + mf2 + mf3 + mf4 + mf5;

    h1 = mf1 / total;
    h2 = mf2 / total;
    h3 = mf3 / total;
    h4 = mf4 / total;
    h5 = mf5 / total;
}

float TS_Fuzzy::computeControl(float theta, float thetaRate, float pos, float vel){
    float h1, h2, h3, h4, h5;
    computeMemberships(theta, h1, h2, h3, h4, h5);

    float x[4] = {theta, thetaRate, pos, vel};
    float u = 0.0f;

    for (int i = 0; i < 4; i++) {
        float Ki = h1 * K[0][i] + h2 * K[1][i] + h3 * K[2][i] + h4 * K[3][i] + h5 * K[4][i];
        u += -Ki * x[i];
    }

    return u;
}