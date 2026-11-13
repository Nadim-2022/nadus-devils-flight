/*!
 * @file AltKalman.cpp
 *
 * 3-state altitude Kalman filter. See AltKalman.h.
 */

#include "AltKalman.h"

#include <math.h>

AltKalman::AltKalman() { reset(0.0f); }

void AltKalman::reset(float h0) {
  _h = h0;
  _v = 0.0f;
  _b = 0.0f;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) P_[i][j] = 0.0f;
  P_[0][0] = 1.0f; // 1 m initial altitude uncertainty
  P_[1][1] = 1.0f; // 1 m/s
  P_[2][2] = 0.1f; // ~0.3 m/s^2 initial bias uncertainty
  _rejected = 0;
}

void AltKalman::predict(float accel_up, float dt) {
  if (dt <= 0.0f || dt > 0.1f) {
    return; // ignore bad time steps
  }

  // State: constant-acceleration model, bias-corrected
  const float a = accel_up - _b;
  _h += _v * dt + 0.5f * a * dt * dt;
  _v += a * dt;

  // Covariance: P = F P F^T + Q
  //   F = [[1, dt, -dt^2/2], [0, 1, -dt], [0, 0, 1]]
  const float dt2 = dt * dt;
  const float f02 = -0.5f * dt2;
  const float f12 = -dt;

  float FP[3][3];
  for (int j = 0; j < 3; j++) {
    FP[0][j] = P_[0][j] + dt * P_[1][j] + f02 * P_[2][j];
    FP[1][j] = P_[1][j] + f12 * P_[2][j];
    FP[2][j] = P_[2][j];
  }
  for (int i = 0; i < 3; i++) {
    P_[i][0] = FP[i][0] + dt * FP[i][1] + f02 * FP[i][2];
    P_[i][1] = FP[i][1] + f12 * FP[i][2];
    P_[i][2] = FP[i][2];
  }

  // Process noise: acceleration noise drives h and v, bias random walk
  const float qa = _accelNoise * _accelNoise;
  P_[0][0] += qa * dt2 * dt2 * 0.25f;
  P_[0][1] += qa * dt2 * dt * 0.5f;
  P_[1][0] += qa * dt2 * dt * 0.5f;
  P_[1][1] += qa * dt2;
  P_[2][2] += _biasWalk * _biasWalk * dt;
}

bool AltKalman::updateBaro(float baro_h) {
  const float R = _baroNoise * _baroNoise;
  const float y = baro_h - _h; // innovation
  const float S = P_[0][0] + R;

  // Outlier check. After kMaxRejectInRow rejections in a row the sample is
  // accepted anyway, so a real sudden change can't lock the filter out.
  if (_gateSigma > 0.0f && y * y > _gateSigma * _gateSigma * S &&
      _rejected < kMaxRejectInRow) {
    _rejected++;
    return false;
  }
  _rejected = 0;

  // Kalman gain, H = [1 0 0]
  const float K0 = P_[0][0] / S;
  const float K1 = P_[1][0] / S;
  const float K2 = P_[2][0] / S;

  _h += K0 * y;
  _v += K1 * y;
  _b += K2 * y;

  // P = (I - K H) P
  const float p0[3] = {P_[0][0], P_[0][1], P_[0][2]};
  for (int j = 0; j < 3; j++) {
    P_[0][j] -= K0 * p0[j];
    P_[1][j] -= K1 * p0[j];
    P_[2][j] -= K2 * p0[j];
  }
  symmetrize();
  return true;
}

float AltKalman::altitudeSigma() const {
  return sqrtf(P_[0][0] > 0.0f ? P_[0][0] : 0.0f);
}

void AltKalman::symmetrize() {
  // keep P symmetric against float rounding
  P_[0][1] = P_[1][0] = 0.5f * (P_[0][1] + P_[1][0]);
  P_[0][2] = P_[2][0] = 0.5f * (P_[0][2] + P_[2][0]);
  P_[1][2] = P_[2][1] = 0.5f * (P_[1][2] + P_[2][1]);
}
