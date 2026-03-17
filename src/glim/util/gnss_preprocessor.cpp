#include <glim/util/gnss_preprocessor.hpp>

#include <spdlog/spdlog.h>

namespace glim {

GNSSPreprocessor::GNSSPreprocessor(double init_var_thresh, double min_travel_dist, double datum_var_thresh)
: init_var_thresh_(init_var_thresh), min_travel_dist_(min_travel_dist), datum_var_thresh_(datum_var_thresh) {}

std::optional<GNSSProcessedMeasurement> GNSSPreprocessor::process(const GNSSRawInput& in) {
  // Reject clearly invalid position (Gulf of Guinea / uninitialized receiver)
  if (in.latitude == 0.0 && in.longitude == 0.0) {
    ++zero_pos_count_;
    if (should_warn(zero_pos_count_)) {
      spdlog::warn("GNSS: lat=0 lon=0 (invalid fix, Gulf of Guinea?) — dropping (count={})", zero_pos_count_);
    }
    return std::nullopt;
  }
  zero_pos_count_ = 0;

  if (in.stamp == 0.0) {
    spdlog::warn("GNSS: timestamp is zero — dropping measurement");
    return std::nullopt;
  }

  if (!origin_set_) {
    // === Deferred datum mode ===
    // Step 1: set provisional origin on first valid measurement for distance tracking
    if (!provisional_origin_set_) {
      provisional_enu_.Reset(in.latitude, in.longitude, in.altitude_hae);
      provisional_origin_set_ = true;
      spdlog::info("GNSS: provisional origin set for distance tracking: lat={:.6f} lon={:.6f}", in.latitude, in.longitude);
    }

    // Compute provisional ENU position (only for travel distance check)
    double pe, pn, pu;
    provisional_enu_.Forward(in.latitude, in.longitude, in.altitude_hae, pe, pn, pu);
    const double travel_dist = std::sqrt(pe * pe + pn * pn);

    // Horizontal variance of this measurement
    double var_horiz = 1.0;
    if (in.has_pos_cov) {
      var_horiz = std::max(in.cov_ee, in.cov_nn);
    }

    // Check datum conditions: sufficient travel AND good RTK fix
    const bool sufficient_travel = (travel_dist >= min_travel_dist_);
    const bool good_fix = in.is_rtk_fixed && (var_horiz < datum_var_thresh_);

    if (!sufficient_travel || !good_fix) {
      // Still buffering
      raw_buffer_.push_back(in);
      if (raw_buffer_.size() % 50 == 0) {
        spdlog::debug(
          "GNSS datum pending: travel={:.1f}m (need {:.1f}m), var={:.4f} (need <{:.4f}), rtk={}, buffered={}",
          travel_dist, min_travel_dist_, var_horiz, datum_var_thresh_, in.is_rtk_fixed, raw_buffer_.size());
      }
      return std::nullopt;
    }

    // === Datum conditions met: set actual origin at this measurement ===
    enu_converter_.Reset(in.latitude, in.longitude, in.altitude_hae);
    origin_set_ = true;
    spdlog::info(
      "GNSS datum set after {:.1f}m travel (var={:.5f}m², {} buffered): lat={:.6f} lon={:.6f} alt={:.3f}",
      travel_dist, var_horiz, raw_buffer_.size(), in.latitude, in.longitude, in.altitude_hae);

    // Convert all buffered measurements to ENU using the actual datum
    for (const auto& buffered : raw_buffer_) {
      ready_batch_.push_back(convert_raw(buffered));
    }
    raw_buffer_.clear();

    // Also include the trigger measurement itself in the batch
    ready_batch_.push_back(convert_raw(in));

    // Return nullopt — caller uses take_ready_batch() to retrieve the batch
    return std::nullopt;
  }

  // === Normal processing: datum already set ===
  return convert_raw(in);
}

GNSSProcessedMeasurement GNSSPreprocessor::convert_raw(const GNSSRawInput& in) const {
  GNSSProcessedMeasurement out;
  out.stamp = in.stamp;
  out.is_rtk_fixed = in.is_rtk_fixed;
  out.fused_heading_enu = in.fused_heading_enu;
  out.fused_heading_sigma = in.fused_heading_sigma;

  // LLA → ENU
  enu_converter_.Forward(in.latitude, in.longitude, in.altitude_hae, out.pos_enu[0], out.pos_enu[1], out.pos_enu[2]);

  // Velocity: NED struct (priority 1) or track+speed fallback (priority 2)
  out.vel_enu = Eigen::Vector3d::Zero();
  if (in.has_ned) {
    out.vel_enu << in.vel_e, in.vel_n, -in.vel_d;
  } else if (in.speed > 0.1) {
    out.vel_enu << in.speed * std::sin(in.track), in.speed * std::cos(in.track), in.climb;
  }

  // Variance from posCov with a minimum floor
  out.variance = Eigen::Vector3d(1.0, 1.0, 4.0);
  if (in.has_pos_cov) {
    out.variance << in.cov_ee, in.cov_nn, in.cov_dd;
    if (out.variance.isZero() && !in.is_rtk_fixed) {
      spdlog::warn("GNSS: posCov is all-zero — using default variance [1,1,4] m²");
    }
  } else {
    spdlog::debug("GNSS: no posCov field — using default variance [1,1,4] m²");
  }
  out.variance = out.variance.cwiseMax(1e-4);

  return out;
}

std::vector<GNSSProcessedMeasurement> GNSSPreprocessor::take_ready_batch() {
  std::vector<GNSSProcessedMeasurement> batch;
  std::swap(batch, ready_batch_);
  return batch;
}

} // namespace glim
