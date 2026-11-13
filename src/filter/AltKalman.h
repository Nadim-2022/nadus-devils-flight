/*!
 * @file AltKalman.h
 *
 * 3-state Kalman filter for drone altitude: fuses the world-frame vertical
 * acceleration (BNO085 linear acceleration) with barometric altitude
 * (BMP581).
 *
 * State:  altitude above ground (m)
 *         vertical speed, up = + (m/s)
 *         accelerometer bias in the up direction (m/s^2)
 *
 * The model is linear, so this is an exact (not extended) Kalman filter.
 *
 * Usage:
 *   static AltKalman kf;
 *   kf.reset(0.0f);                    // after measuring ground pressure
 *   every IMU sample:       kf.predict(accel_up, dt);
 *   every NEW baro sample:  kf.updateBaro(baro_h);
 *   read kf.altitude(), kf.climbRate()
 */

#ifndef ALT_KALMAN_H
#define ALT_KALMAN_H

class AltKalman {
public:
  AltKalman();

  /*! @brief Start again at altitude h0 (m), speed 0, bias 0. */
  void reset(float h0 = 0.0f);

  /*!
   * @brief Prediction step. Call at the IMU rate.
   * @param accel_up world-frame vertical acceleration, gravity removed (m/s^2)
   * @param dt       time since the previous call (s); steps outside
   *                 (0, 0.1] s are ignored
   */
  void predict(float accel_up, float dt);

  /*!
   * @brief Correction step. Call ONLY when the barometer has a NEW sample.
   * @param baro_h barometric altitude (m)
   * @return false if the sample was rejected as an outlier
   */
  bool updateBaro(float baro_h);

  // ---- Results ----------------------------------------------------------
  float altitude() const { return _h; }        ///< m
  float climbRate() const { return _v; }       ///< m/s, up = +
  float accelBias() const { return _b; }       ///< m/s^2
  float altitudeSigma() const;                 ///< 1-sigma uncertainty, m
  unsigned rejectedInRow() const { return _rejected; }

  // ---- Tuning (physical units) -------------------------------------------
  /*! Vertical acceleration noise, m/s^2 (1 sigma). Bigger = follow baro more. */
  void setAccelNoise(float sigma) { _accelNoise = sigma; }
  /*! How fast the accel bias may wander, m/s^2 per sqrt(s). */
  void setBiasWalk(float sigma) { _biasWalk = sigma; }
  /*! Barometer noise, m (1 sigma). Bigger = smoother but slower. */
  void setBaroNoise(float sigma) { _baroNoise = sigma; }
  /*! Reject baro samples further than this many sigmas away. 0 = off. */
  void setGateSigma(float sigmas) { _gateSigma = sigmas; }

private:
  void symmetrize();

  // tuning
  float _accelNoise = 0.5f;
  float _biasWalk = 0.01f;
  float _baroNoise = 0.2f;
  float _gateSigma = 4.0f;

  // state and covariance
  float _h = 0.0f, _v = 0.0f, _b = 0.0f;
  float P_[3][3];
  unsigned _rejected = 0;

  static constexpr unsigned kMaxRejectInRow = 10;
};

#endif
