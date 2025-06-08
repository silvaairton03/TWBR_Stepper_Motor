#include "TS_Fuzzy.h"

TS_Fuzzy::TS_Fuzzy() {
    sigma = radians(30.0f); // Suitable for 3 or 5 MFs; change via setter if needed
    memset(K, 0, sizeof(K));
}

void TS_Fuzzy::setSigmaDegrees(float deg) {
    sigma = radians(deg);
}

void TS_Fuzzy::set3Gains(const float K1[4], const float K2[4], const float K3[4]) {
    memcpy(K[0], K1, sizeof(float) * 4);
    memcpy(K[1], K2, sizeof(float) * 4);
    memcpy(K[2], K3, sizeof(float) * 4);
}

void TS_Fuzzy::set5Gains(const float K1[4], const float K2[4], const float K3[4],
                         const float K4[4], const float K5[4]) {
    memcpy(K[0], K1, sizeof(float) * 4);
    memcpy(K[1], K2, sizeof(float) * 4);
    memcpy(K[2], K3, sizeof(float) * 4);
    memcpy(K[3], K4, sizeof(float) * 4);
    memcpy(K[4], K5, sizeof(float) * 4);
}

void TS_Fuzzy::compute3Memberships(float theta, float &h1, float &h2, float &h3) {
    float mf1 = expf(-powf((theta + DEG_2) / sigma, 2));
    float mf2 = expf(-powf((theta - radians(0.0f)) / sigma, 2));
    float mf3 = expf(-powf((theta - DEG_2) / sigma, 2));

    float total = mf1 + mf2 + mf3;
    if (total < 1e-6f) {
        h1 = h2 = h3 = 1.0f / 3.0f;
    } else {
        h1 = mf1 / total;
        h2 = mf2 / total;
        h3 = mf3 / total;
    }
}

void TS_Fuzzy::compute5Memberships(float theta, float &h1, float &h2, float &h3, float &h4, float &h5) {
    float mf1 = expf(-powf((theta + DEG_2) / sigma, 2));
    float mf2 = expf(-powf((theta + DEG_5) / sigma, 2));
    float mf3 = expf(-powf((theta - radians(0.0f)) / sigma, 2));
    float mf4 = expf(-powf((theta - DEG_2) / sigma, 2));
    float mf5 = expf(-powf((theta - DEG_5 ) / sigma, 2));

    float total = mf1 + mf2 + mf3 + mf4 + mf5;
    if (total < 1e-6f) {
        h1 = h2 = h3 = h4 = h5 = 1.0f / 5.0f;
    } else {
        h1 = mf1 / total;
        h2 = mf2 / total;
        h3 = mf3 / total;
        h4 = mf4 / total;
        h5 = mf5 / total;
    }
}

float TS_Fuzzy::computeControl3mf(float theta, float thetaRate, float pos, float vel) {
    float h1, h2, h3;
    compute3Memberships(theta, h1, h2, h3);
    float x[4] = {theta, thetaRate, pos, vel};
    float u = 0.0f;

    for (int i = 0; i < 4; i++) {
        float Ki = h1 * K[0][i] + h2 * K[1][i] + h3 * K[2][i];
        u -= Ki * x[i];
    }

    return u;
}

float TS_Fuzzy::computeControl5mf(float theta, float thetaRate, float pos, float vel) {
    float h1, h2, h3, h4, h5;
    compute5Memberships(theta, h1, h2, h3, h4, h5);
    float x[4] = {theta, thetaRate, pos, vel};
    float u = 0.0f;

    for (int i = 0; i < 4; i++) {
        float Ki = h1 * K[0][i] + h2 * K[1][i] + h3 * K[2][i] + h4 * K[3][i] + h5 * K[4][i];
        u -= Ki * x[i];
    }

    return u;
}
