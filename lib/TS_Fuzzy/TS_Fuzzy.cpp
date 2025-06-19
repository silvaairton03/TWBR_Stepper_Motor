#include "TS_Fuzzy.h"

TS_Fuzzy::TS_Fuzzy() {
    setSigmaDegrees(1.0f);
    mode = MF3;

    for (int i = 0; i <5; i++)
        for (int j = 0; j < 4; j++)
            K[i][j] = 0.0f;
}

void TS_Fuzzy::setSigmaDegrees(float deg) {
    sigma = radians(deg);
    invSigmaSq = 1.0f / (sigma * sigma);
}

void TS_Fuzzy::setMode(Mode m) {
    mode = m;
}

void TS_Fuzzy::set3Gains(const float K1[4], const float K2[4], const float K3[4]) {
    for (int i = 0; i < 4; i++) {
        K[0][i] = K1[i];
        K[1][i] = K2[i];
        K[2][i] = K3[i];
    }
}

void TS_Fuzzy::set5Gains(const float K1[4], const float K2[4], const float K3[4],
                         const float K4[4], const float K5[4]) {
    for (int i = 0; i < 4; i++) {
        K[0][i] = K1[i];
        K[1][i] = K2[i];
        K[2][i] = K3[i];
        K[3][i] = K4[i];
        K[4][i] = K5[i];
    }
}

void TS_Fuzzy::compute3Memberships(float theta, float &h1, float &h2, float &h3) {
    float mf1 = expf(-((theta + DEG_2) * (theta + DEG_2)) * invSigmaSq);
    float mf2 = expf(-(theta * theta) * invSigmaSq);
    float mf3 = expf(-((theta - DEG_2) * (theta - DEG_2)) * invSigmaSq);

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
    float mf1 = expf(-((theta + DEG_5) * (theta + DEG_5)) * invSigmaSq);
    float mf2 = expf(-((theta + DEG_2) * (theta + DEG_2)) * invSigmaSq);
    float mf3 = expf(-(theta * theta) * invSigmaSq);
    float mf4 = expf(-((theta - DEG_2) * (theta - DEG_2)) * invSigmaSq);
    float mf5 = expf(-((theta - DEG_5) * (theta - DEG_5)) * invSigmaSq);

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

float TS_Fuzzy::computeControl(float theta, float thetaRate, float pos, float vel) {
    float h[5] = {0};  // Máximo 5 MF
    float x[4] = {theta, thetaRate, pos, vel};

    // Cálculo das pertinências
    if (mode == MF3) {
        compute3Memberships(theta, h[0], h[1], h[2]);
    } else {
        compute5Memberships(theta, h[0], h[1], h[2], h[3], h[4]);
    }

    // Cálculo do controle
    float u = 0.0f;
    for (int i = 0; i < 4; i++) {
        float Ki = 0.0f;
        for (int j = 0; j < (int)mode; j++) {
            Ki += h[j] * K[j][i];
        }
        u -= Ki * x[i];
    }

    return u;
}


