/**
 * nanobind module exposing fusion::TinyEkfFilter to Python.
 *
 * Thin passthrough: bind the same C++ types the firmware uses, expose
 * std::chrono::milliseconds timestamps as int milliseconds across the FFI.
 * All Rust-shaped ergonomics (frozen dataclasses, kw-only construction)
 * live in the Python wrapper, not here.
 */

#include "ekf_filter.h"
#include "common.h"
#include <nanobind/nanobind.h>

namespace nb = nanobind;
using namespace fusion;
using plrs::Quaternion;

NB_MODULE(_native, m) {
  m.doc() = "Native bindings for the Polaris fusion EKF.";

  nb::class_<plrs::Vec3>(m, "Vec3")
      .def(nb::init<>())
      .def(nb::init<float, float, float>(), nb::arg("x"), nb::arg("y"),
           nb::arg("z"))
      .def_rw("x", &plrs::Vec3::x)
      .def_rw("y", &plrs::Vec3::y)
      .def_rw("z", &plrs::Vec3::z);

  nb::class_<Quaternion>(m, "Quaternion")
      .def(nb::init<>())
      .def(nb::init<float, float, float, float>(), nb::arg("w"), nb::arg("x"),
           nb::arg("y"), nb::arg("z"))
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
          [](ImuSample &s, Ms::rep v) { s.timestamp = Ms{v}; });

  nb::class_<GnssSample>(m, "GnssSample")
      .def(nb::init<>())
      .def_rw("heading_deg", &GnssSample::heading_deg)
      .def_rw("heading_variance_deg2", &GnssSample::heading_variance_deg2)
      .def_rw("valid", &GnssSample::valid)
      .def_prop_rw(
          "timestamp_ms",
          [](const GnssSample &s) { return s.timestamp.count(); },
          [](GnssSample &s, Ms::rep v) { s.timestamp = Ms{v}; });

  nb::class_<FusionOutput>(m, "FusionOutput")
      .def_ro("heading_deg", &FusionOutput::heading_deg)
      .def_ro("heading_variance_deg2", &FusionOutput::heading_variance_deg2)
      .def_ro("roll_deg", &FusionOutput::roll_deg)
      .def_ro("roll_variance_deg2", &FusionOutput::roll_variance_deg2)
      .def_ro("pitch_deg", &FusionOutput::pitch_deg)
      .def_ro("pitch_variance_deg2", &FusionOutput::pitch_variance_deg2)
      .def_prop_ro("timestamp_ms",
                   [](const FusionOutput &s) { return s.timestamp.count(); });

  nb::class_<TinyEkfFilter::Config>(m, "Config")
      .def(nb::init<>())
      .def_rw("q_heading_deg2", &TinyEkfFilter::Config::q_heading_deg2)
      .def_rw("q_bias_deg2_s2", &TinyEkfFilter::Config::q_bias_deg2_s2)
      .def_rw("p0_heading_deg2", &TinyEkfFilter::Config::p0_heading_deg2)
      .def_rw("p0_bias_deg2_s2", &TinyEkfFilter::Config::p0_bias_deg2_s2)
      .def_rw("mount", &TinyEkfFilter::Config::mount);

  nb::class_<TinyEkfFilter>(m, "TinyEkfFilter")
      .def(nb::init<TinyEkfFilter::Config>(), nb::arg("cfg"))
      .def("predict", &TinyEkfFilter::predict, nb::arg("imu"))
      .def("update", &TinyEkfFilter::update, nb::arg("gnss"))
      .def("output", &TinyEkfFilter::output);
}
