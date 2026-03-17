#pragma once

#include <cmath>
#include <limits>
#include <vector>
#include <optional>

#include <Eigen/Core>
#include <GeographicLib/LocalCartesian.hpp>

namespace glim {

/**
 * @brief Raw GNSS values extracted from a cereal/capnp message (no external types).
 */
struct GNSSRawInput {
  double stamp;        ///< Timestamp (seconds)
  double latitude;     ///< Degrees
  double longitude;    ///< Degrees
  double altitude_hae; ///< Height above ellipsoid (meters)
  bool is_rtk_fixed;   ///< true = RTK Fix, false = RTK Float

  // Position covariance in NED frame (East-East, North-North, Down-Down)
  bool has_pos_cov{false};
  double cov_ee{0.0}, cov_nn{0.0}, cov_dd{0.0};

  // NED velocity (highest priority)
  bool has_ned{false};
  double vel_e{0.0}, vel_n{0.0}, vel_d{0.0};

  // Track + speed fallback (used when !has_ned)
  double speed{0.0}; ///< Ground speed (m/s)
  double track{0.0}; ///< True heading (rad)
  double climb{0.0}; ///< Vertical speed (m/s)

  // ImuFused heading attached by caller before passing to process()
  double fused_heading_enu{std::numeric_limits<double>::quiet_NaN()};
  double fused_heading_sigma{0.01};
};

/**
 * @brief Processed GNSS measurement in ENU frame, ready for OdometryEstimation::insert_gnss().
 */
struct GNSSProcessedMeasurement {
  double stamp;
  Eigen::Vector3d pos_enu;
  Eigen::Vector3d vel_enu;
  Eigen::Vector3d variance; ///< [East, North, Down] in m²
  bool is_rtk_fixed;
  double fused_heading_enu{std::numeric_limits<double>::quiet_NaN()};
  double fused_heading_sigma{0.01};
};

/**
 * @brief Preprocesses raw GNSS measurements into ENU-frame measurements.
 *
 * In deferred datum mode (default, min_travel_dist > 0):
 *  - Buffers all raw measurements until the vehicle has traveled min_travel_dist
 *    metres AND a measurement arrives with RTK fix and variance < datum_var_thresh.
 *  - Sets LocalCartesian origin at that "good" point, converts all buffered data.
 *  - Caller retrieves the batch via take_ready_batch() and uses it for Umeyama
 *    trajectory alignment to calibrate ENU→world yaw+translation.
 *
 * Responsibilities:
 *  - Rejects measurements with lat=0/lon=0 (Gulf of Guinea) or zero timestamp
 *  - Converts LLA → ENU
 *  - Extracts velocity from NED struct (priority 1) or track+speed fallback (priority 2)
 *  - Extracts position variance from posCov, applies minimum floor of 1e-4 m²
 *
 * The class is cereal/capnp-agnostic; the caller fills GNSSRawInput from whatever
 * IPC mechanism is in use.
 */
class GNSSPreprocessor {
public:
  /**
   * @param init_var_thresh    Max horizontal variance (m²) to accept during initial buffering.
   * @param min_travel_dist    Minimum travel distance (m) before datum can be set. Set to 0
   *                           to disable deferred mode (original behaviour).
   * @param datum_var_thresh   Max horizontal variance (m²) required for the datum point.
   *                           Must be <= init_var_thresh. Typically 0.05 m² (RTK fix).
   */
  explicit GNSSPreprocessor(double init_var_thresh = 0.1,
                             double min_travel_dist = 5.0,
                             double datum_var_thresh = 0.05);

  /**
   * @brief Process one raw measurement.
   *
   * In deferred mode: returns std::nullopt while buffering. Once the datum is set,
   * returns std::nullopt for the trigger measurement (it is placed in the ready batch);
   * subsequent calls return the processed measurement normally.
   *
   * @return std::nullopt if buffering or measurement rejected; otherwise ready-to-use measurement.
   */
  std::optional<GNSSProcessedMeasurement> process(const GNSSRawInput& in);

  /**
   * @brief Return (and clear) the batch of buffered measurements converted after datum was set.
   *        Non-empty exactly once, immediately after datum becomes ready.
   */
  std::vector<GNSSProcessedMeasurement> take_ready_batch();

  bool has_origin() const { return origin_set_; }
  void reset_origin() { origin_set_ = false; }

private:
  GNSSProcessedMeasurement convert_raw(const GNSSRawInput& in) const;
  static bool should_warn(int count) { return count == 1 || count % 100 == 0; }

  double init_var_thresh_;
  double min_travel_dist_;
  double datum_var_thresh_;

  // Deferred datum buffering
  bool provisional_origin_set_{false};
  GeographicLib::LocalCartesian provisional_enu_; // for distance tracking only
  std::vector<GNSSRawInput> raw_buffer_;           // buffered inputs before datum

  // Actual ENU origin
  bool origin_set_{false};
  GeographicLib::LocalCartesian enu_converter_;

  // Batch populated when datum becomes ready (consumed once by take_ready_batch())
  std::vector<GNSSProcessedMeasurement> ready_batch_;

  int zero_pos_count_{0};
};

/**
 * @brief Convert navigation heading to ENU yaw.
 *
 * Navigation convention: clockwise from true North (radians).
 * ENU convention: counter-clockwise from East (radians).
 *
 * @param heading_nav  True heading CW from North (radians)
 * @return             ENU yaw CCW from East (radians)
 */
inline double heading_nav_to_enu_yaw(double heading_nav) {
  return M_PI / 2.0 - heading_nav;
}

} // namespace glim
