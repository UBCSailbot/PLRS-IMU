/**
 * Bridge from the GNSS dual-antenna attitude block to a filter heading
 * measurement.
 *
 * The mosaic-go-H reports the true heading of the antenna baseline
 * (main -> aux) in [0, 360); the filter wants boat-forward heading in
 * +-180. A static mount offset reconciles the two, and the block's
 * covariance feeds the measurement variance.
 *
 * This is the one place the fusion layer depends on the Septentrio block
 * types; the GNSS decode layer stays unaware of fusion.
 */

#pragma once

#include "attitude.h"
#include "fusion.h"
#include "sbf_blocks.h"

namespace fusion {

/*
 * AttEuler mode code 0 means the receiver has no attitude solution; codes 1-4
 * all carry a heading (sbf_blocks.h AttEuler::mode). Mode is the authoritative
 * heading-availability gate, and code 0 already covers the "attitude not
 * requested" error case.
 */
constexpr uint16_t ATT_MODE_NO_ATTITUDE = 0;

/**
 * Static calibration relating the GNSS antenna baseline to boat-forward.
 *
 * baseline_offset_deg is the heading of the baseline measured clockwise
 * from boat-forward, subtracted off so the filter sees boat heading. The
 * receiver's own setAttitudeOffset is left at zero so the offset is applied
 * exactly once, here. fallback_heading_variance_deg2 is used when the
 * receiver emits a Do-Not-Use covariance.
 */
struct GnssAttitudeMount {
  float baseline_offset_deg = 0.0f;
  float fallback_heading_variance_deg2 = 0.0f;
};

/**
 * @brief Convert a dual-antenna attitude block into a GnssSample.
 *
 * @param att   Decoded AttEuler (block 5938).
 * @param cov   Decoded AttCovEuler (block 5939) for the same epoch.
 * @param mount Static baseline-to-boat calibration.
 *
 * @return A heading measurement for update(). valid is false when the
 *         receiver has no attitude solution or a Do-Not-Use heading, in
 *         which case the filter skips it. Heading is converted to boat
 *         frame and wrapped to +-180; variance falls back to the mount
 *         default when the covariance is Do-Not-Use.
 */
inline GnssSample att_euler_to_gnss_sample(const sbf::AttEuler &att,
                                           const sbf::AttCovEuler &cov,
                                           const GnssAttitudeMount &mount) {
  const bool has_heading =
      att.mode != ATT_MODE_NO_ATTITUDE && att.heading != sbf::DNU_F4;
  const float variance = cov.cov_headhead != sbf::DNU_F4
                             ? cov.cov_headhead
                             : mount.fallback_heading_variance_deg2;
  return GnssSample{
      .heading_deg = wrap180(att.heading - mount.baseline_offset_deg),
      .heading_variance_deg2 = variance,
      .timestamp = Ms{att.tow},
      .valid = has_heading,
  };
}

} // namespace fusion
