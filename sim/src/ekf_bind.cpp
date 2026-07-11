/**
 * nanobind module exposing fusion::TinyEkfFilter to Python.
 *
 * Thin passthrough: bind the same C++ types the firmware uses, expose
 * std::chrono::milliseconds timestamps as int milliseconds across the FFI.
 * All Rust-shaped ergonomics (frozen dataclasses, kw-only construction)
 * live in the Python wrapper, not here.
 */

#include "common.h"
#include "ekf_filter.h"
#include "gnss_bridge.h"
#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>

namespace nb = nanobind;
using namespace fusion;
using plrs::Quaternion;

NB_MODULE(_native, m) {
  m.doc() = "Native bindings for the Polaris fusion EKF.";

  nb::class_<plrs::Vec3>(m, "Vec3")
      .def(nb::init<>())
      .def(nb::init<float, float, float>(),
           nb::arg("x"),
           nb::arg("y"),
           nb::arg("z"))
      .def_rw("x", &plrs::Vec3::x)
      .def_rw("y", &plrs::Vec3::y)
      .def_rw("z", &plrs::Vec3::z);

  nb::class_<Quaternion>(m, "Quaternion")
      .def(nb::init<>())
      .def(nb::init<float, float, float, float>(),
           nb::arg("w"),
           nb::arg("x"),
           nb::arg("y"),
           nb::arg("z"))
      .def_rw("w", &Quaternion::w)
      .def_rw("x", &Quaternion::x)
      .def_rw("y", &Quaternion::y)
      .def_rw("z", &Quaternion::z);

  // UnitQuaternion is private-constructor + factory; expose from_raw so
  // Python can hand the filter a validated rotation. The sim's truth
  // path always produces a unit quaternion, so the error branch turns
  // into a Python exception rather than a silent identity.
  nb::class_<UnitQuaternion>(m, "UnitQuaternion")
      .def_static(
          "from_raw",
          [](Quaternion q) {
            auto result = UnitQuaternion::from_raw(q);
            if (!result) {
              throw nb::value_error(result.error());
            }
            return *result;
          },
          nb::arg("q"))
      .def_static("identity", &UnitQuaternion::identity);

  nb::class_<MountRotation>(m, "MountRotation")
      .def(nb::init<>())
      .def_rw("boat_to_imu", &MountRotation::boat_to_imu);

  nb::class_<ImuSample>(m, "ImuSample")
      .def(nb::init<>())
      .def_rw("angular_velocity_rad_s", &ImuSample::angular_velocity_rad_s)
      .def_rw("accel_ms2", &ImuSample::accel_ms2)
      .def_rw("orientation", &ImuSample::orientation)
      .def_prop_rw(
          "timestamp_ms",
          [](const ImuSample &s) { return s.timestamp.count(); },
          [](ImuSample &s, Ms::rep v) { s.timestamp = Ms {v}; });

  nb::class_<GnssSample>(m, "GnssSample")
      .def(nb::init<>())
      .def_rw("heading_deg", &GnssSample::heading_deg)
      .def_rw("heading_variance_deg2", &GnssSample::heading_variance_deg2)
      .def_rw("valid", &GnssSample::valid)
      .def_prop_rw(
          "timestamp_ms",
          [](const GnssSample &s) { return s.timestamp.count(); },
          [](GnssSample &s, Ms::rep v) { s.timestamp = Ms {v}; });

  nb::class_<FusionOutput>(m, "FusionOutput")
      .def_ro("heading_deg", &FusionOutput::heading_deg)
      .def_ro("heading_variance_deg2", &FusionOutput::heading_variance_deg2)
      .def_ro("roll_deg", &FusionOutput::roll_deg)
      .def_ro("roll_variance_deg2", &FusionOutput::roll_variance_deg2)
      .def_ro("pitch_deg", &FusionOutput::pitch_deg)
      .def_ro("pitch_variance_deg2", &FusionOutput::pitch_variance_deg2)
      .def_prop_ro("timestamp_ms",
                   [](const FusionOutput &s) { return s.timestamp.count(); })
      .def_ro("yaw_rate_dps", &FusionOutput::yaw_rate_dps);

  nb::class_<TinyEkfFilter::MtiYawConfig>(m, "MtiYawConfig")
      .def(nb::init<>())
      .def_rw("variance_deg2", &TinyEkfFilter::MtiYawConfig::variance_deg2)
      .def_rw("q_offset_deg2", &TinyEkfFilter::MtiYawConfig::q_offset_deg2)
      .def_rw("p0_offset_deg2", &TinyEkfFilter::MtiYawConfig::p0_offset_deg2);

  nb::class_<TinyEkfFilter::Config>(m, "Config")
      .def(nb::init<>())
      .def_rw("q_heading_deg2", &TinyEkfFilter::Config::q_heading_deg2)
      .def_rw("q_roll_deg2", &TinyEkfFilter::Config::q_roll_deg2)
      .def_rw("q_pitch_deg2", &TinyEkfFilter::Config::q_pitch_deg2)
      .def_rw("q_bias_deg2_s2", &TinyEkfFilter::Config::q_bias_deg2_s2)
      .def_rw("p0_heading_deg2", &TinyEkfFilter::Config::p0_heading_deg2)
      .def_rw("p0_roll_deg2", &TinyEkfFilter::Config::p0_roll_deg2)
      .def_rw("p0_pitch_deg2", &TinyEkfFilter::Config::p0_pitch_deg2)
      .def_rw("p0_bias_deg2_s2", &TinyEkfFilter::Config::p0_bias_deg2_s2)
      .def_rw("mti_roll_variance_deg2",
              &TinyEkfFilter::Config::mti_roll_variance_deg2)
      .def_rw("mti_pitch_variance_deg2",
              &TinyEkfFilter::Config::mti_pitch_variance_deg2)
      .def_rw("mti_yaw", &TinyEkfFilter::Config::mti_yaw)
      .def_rw("mount", &TinyEkfFilter::Config::mount);

  nb::class_<TinyEkfFilter::Debug>(m, "Debug")
      .def_ro("gyro_bias_dps", &TinyEkfFilter::Debug::gyro_bias_dps)
      .def_ro("gyro_bias_variance_deg2_s2",
              &TinyEkfFilter::Debug::gyro_bias_variance_deg2_s2)
      .def_ro("mag_offset_deg", &TinyEkfFilter::Debug::mag_offset_deg)
      .def_ro("mag_offset_variance_deg2",
              &TinyEkfFilter::Debug::mag_offset_variance_deg2)
      .def_ro("gate_rejects", &TinyEkfFilter::Debug::gate_rejects)
      .def_ro("mag_gate_rejects", &TinyEkfFilter::Debug::mag_gate_rejects);

  nb::class_<TinyEkfFilter>(m, "TinyEkfFilter")
      .def(nb::init<TinyEkfFilter::Config>(), nb::arg("cfg"))
      .def("predict", &TinyEkfFilter::predict, nb::arg("imu"))
      .def("update", &TinyEkfFilter::update, nb::arg("gnss"))
      .def("output", &TinyEkfFilter::output)
      .def("debug", &TinyEkfFilter::debug);

  // GNSS dual-antenna attitude bridge. The sim builds these from truth,
  // perturbs them, and runs them through the real bridge so the path it
  // exercises is the one the firmware ships.
  nb::class_<sbf::AttEuler>(m, "AttEuler")
      .def(nb::init<>())
      .def_rw("tow", &sbf::AttEuler::tow)
      .def_rw("error", &sbf::AttEuler::error)
      .def_rw("mode", &sbf::AttEuler::mode)
      .def_rw("heading", &sbf::AttEuler::heading)
      .def_rw("pitch", &sbf::AttEuler::pitch)
      .def_rw("roll", &sbf::AttEuler::roll)
      .def_rw("heading_dot", &sbf::AttEuler::heading_dot);

  nb::class_<sbf::AttCovEuler>(m, "AttCovEuler")
      .def(nb::init<>())
      .def_rw("cov_headhead", &sbf::AttCovEuler::cov_headhead)
      .def_rw("cov_pitchpitch", &sbf::AttCovEuler::cov_pitchpitch)
      .def_rw("cov_rollroll", &sbf::AttCovEuler::cov_rollroll);

  nb::class_<GnssAttitudeMount>(m, "GnssAttitudeMount")
      .def(nb::init<>())
      .def_rw("baseline_offset_deg", &GnssAttitudeMount::baseline_offset_deg)
      .def_rw("fallback_heading_variance_deg2",
              &GnssAttitudeMount::fallback_heading_variance_deg2);

  m.def("att_euler_to_gnss_sample",
        &att_euler_to_gnss_sample,
        nb::arg("att"),
        nb::arg("cov"),
        nb::arg("mount"));
}
