#include"TS_Fuzzy.h"

TS_Fuzzy::TS_Fuzzy(){
    sigma = radians(20.0f);
    memset(K, 0, sizeof(K));
}


void TS_Fuzzy::set3Gains(const float K1[4], const float K2[4], const float K3[4])
{
    memcpy(K[0], K1, sizeof(float) * 4);
    memcpy(K[1], K2, sizeof(float) * 4);
    memcpy(K[2], K3, sizeof(float) * 4);
}

void TS_Fuzzy::set5Gains(const float K1[4], const float K2[4], const float K3[4], const float K4[4], const float K5[4]){
    memcpy(K[0], K1, sizeof(float) * 4);
    memcpy(K[1], K2, sizeof(float) * 4);
    memcpy(K[2], K3, sizeof(float) * 4);
    memcpy(K[3], K4, sizeof(float) * 4);
    memcpy(K[4], K5, sizeof(float) * 4);
}

void TS_Fuzzy::set7Gains(const float K1[4], const float K2[4], const float K3[4], const float K4[4], const float K5[4],
                const float K6[4], const float K7[4])
{
    memcpy(K[0], K1, sizeof(float) * 4);
    memcpy(K[1], K2, sizeof(float) * 4);
    memcpy(K[2], K3, sizeof(float) * 4);
    memcpy(K[3], K4, sizeof(float) * 4);
    memcpy(K[4], K5, sizeof(float) * 4);
    memcpy(K[5], K6, sizeof(float) * 4);
    memcpy(K[6], K7, sizeof(float) * 4);

}

void TS_Fuzzy::compute3Memberships(float theta, float &h1, float &h2, float &h3){
    float mf1 = exp(-pow((theta - radians(0.0f)) / sigma, 2));
    float mf2 = exp(-pow((theta - DEG_30) / sigma, 2));
    float mf3 = exp(-pow((theta + DEG_30) / sigma, 2));

    float total = mf1 + mf2 + mf3;

    h1 = mf1 / total;
    h2 = mf2 / total;
    h3 = mf3 / total;
}

void TS_Fuzzy::compute5Memberships(float theta, float &h1, float &h2, float &h3, float &h4, float &h5){
    float mf1 = exp(-pow((theta - radians(0.0f)) / sigma, 2));
    float mf2 = exp(-pow((theta - DEG_30) / sigma, 2));
    float mf3 = exp(-pow((theta + DEG_30) / sigma, 2));
    float mf4 = exp(-pow((theta - DEG_45) / sigma, 2));
    float mf5 = exp(-pow((theta + DEG_45) / sigma, 2));

    float total = mf1 + mf2 + mf3 + mf4 + mf5;

    h1 = mf1 / total;
    h2 = mf2 / total;
    h3 = mf3 / total;
    h4 = mf4 / total;
    h5 = mf5 / total;
}

void TS_Fuzzy::compute7Memberships(float theta, float &h1, float &h2, float &h3, float &h4, float &h5, float &h6, float &h7){
    float mf1 = exp(-pow((theta - radians(0.0f)) / sigma, 2));
    float mf2 = exp(-pow((theta - DEG_30) / sigma, 2));
    float mf3 = exp(-pow((theta + DEG_30) / sigma, 2));
    float mf4 = exp(-pow((theta - DEG_45) / sigma, 2));
    float mf5 = exp(-pow((theta + DEG_45) / sigma, 2));
    float mf6 = exp(-pow((theta - DEG_60) / sigma, 2));
    float mf7 = exp(-pow((theta + DEG_60) / sigma, 2));

    float total = mf1 + mf2 + mf3 + mf4 + mf5 + mf6 + mf7;

    h1 = mf1 / total;
    h2 = mf2 / total;
    h3 = mf3 / total;
    h4 = mf4 / total;
    h5 = mf5 / total;
    h6 = mf6 / total;
    h7 = mf7 / total;
}

float TS_Fuzzy::computeControl3mf(float theta, float thetaRate, float pos, float vel)
{
    float h1, h2, h3;
    compute3Memberships(theta, h1, h2, h3);
    float x[4] = {theta, thetaRate, pos, vel};
    float u = 0.0f;

    for (int i = 0; i < 4; i++) {
        float Ki = h1 * K[0][i] + h2 * K[1][i] + h3 * K[2][i];
        u += -Ki * x[i];
    }

    return u;
}

float TS_Fuzzy::computeControl5mf(float theta, float thetaRate, float pos, float vel){
    float h1, h2, h3, h4, h5;
    compute5Memberships(theta, h1, h2, h3, h4, h5);

    float x[4] = {theta, thetaRate, pos, vel};
    float u = 0.0f;

    for (int i = 0; i < 4; i++) {
        float Ki = h1 * K[0][i] + h2 * K[1][i] + h3 * K[2][i] + h4 * K[3][i] + h5 * K[4][i];
        u += -Ki * x[i];
    }

    return u;
}

float TS_Fuzzy::computeControl7mf(float theta, float thetaRate, float pos, float vel){
    float h1, h2, h3, h4, h5, h6, h7;
    compute7Memberships(theta, h1, h2, h3, h4, h5, h6, h7);

    float x[4] = {theta, thetaRate, pos, vel};
    float u = 0.0f;

    for (int i = 0; i < 4; i++) {
        float Ki = h1 * K[0][i] + h2 * K[1][i] + h3 * K[2][i] + h4 * K[3][i] + h5 * K[4][i] + h6 * K[5][i] + h7 * K[6][i];
        u += -Ki * x[i];
    }

    return u;

}