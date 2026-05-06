#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "gimbal_pipeline/pybind/offline_tracker_replay.hpp"

namespace py = pybind11;
using fyt::auto_aim::ObservationData;
using fyt::auto_aim::pybind::OfflineTrackerReplay;
using fyt::auto_aim::pybind::ReplaySnapshot;

PYBIND11_MODULE(gimbal_pipeline_kf_opt, m) {
  m.doc() = "Offline replay wrapper for gimbal_pipeline tracker parameter optimization";

  py::class_<ObservationData>(m, "ObservationData")
      .def(py::init<>())
      .def_readwrite("x", &ObservationData::x)
      .def_readwrite("y", &ObservationData::y)
      .def_readwrite("z", &ObservationData::z)
      .def_readwrite("yaw", &ObservationData::yaw)
      .def_readwrite("panel_id", &ObservationData::panel_id)
      .def_readwrite("layer", &ObservationData::layer)
      .def_readwrite("confidence", &ObservationData::confidence)
      .def_readwrite("timestamp", &ObservationData::timestamp);

  py::class_<ReplaySnapshot>(m, "ReplaySnapshot")
      .def(py::init<>())
      .def_readwrite("valid", &ReplaySnapshot::valid)
      .def_readwrite("robot_id", &ReplaySnapshot::robot_id)
      .def_readwrite("timestamp_sec", &ReplaySnapshot::timestamp_sec)
      .def_readwrite("tracker_count", &ReplaySnapshot::tracker_count)
      .def_readwrite("track_state", &ReplaySnapshot::track_state)
      .def_readwrite("frame_count", &ReplaySnapshot::frame_count)
      .def_readwrite("lost_count", &ReplaySnapshot::lost_count)
      .def_readwrite("center_x", &ReplaySnapshot::center_x)
      .def_readwrite("center_y", &ReplaySnapshot::center_y)
      .def_readwrite("center_z", &ReplaySnapshot::center_z)
      .def_readwrite("vel_x", &ReplaySnapshot::vel_x)
      .def_readwrite("vel_y", &ReplaySnapshot::vel_y)
      .def_readwrite("vel_z", &ReplaySnapshot::vel_z)
      .def_readwrite("yaw", &ReplaySnapshot::yaw)
      .def_readwrite("radius_1", &ReplaySnapshot::radius_1)
      .def_readwrite("radius_2", &ReplaySnapshot::radius_2)
      .def_readwrite("dza", &ReplaySnapshot::dza)
      .def_readwrite("innov_x", &ReplaySnapshot::innov_x)
      .def_readwrite("innov_y", &ReplaySnapshot::innov_y)
      .def_readwrite("innov_z", &ReplaySnapshot::innov_z)
      .def_readwrite("innov_yaw", &ReplaySnapshot::innov_yaw)
      .def_readwrite("nis", &ReplaySnapshot::nis)
      .def_readwrite("update_type", &ReplaySnapshot::update_type)
      .def_readwrite("p_var_x", &ReplaySnapshot::p_var_x)
      .def_readwrite("p_var_y", &ReplaySnapshot::p_var_y)
      .def_readwrite("p_var_z", &ReplaySnapshot::p_var_z)
      .def_readwrite("obs_count", &ReplaySnapshot::obs_count)
      .def_readwrite("dual_obs", &ReplaySnapshot::dual_obs)
      .def_readwrite("mode", &ReplaySnapshot::mode)
      .def_readwrite("candidate_panel_id", &ReplaySnapshot::candidate_panel_id)
      .def_readwrite("candidate_prob", &ReplaySnapshot::candidate_prob)
      .def_readwrite("candidate_margin", &ReplaySnapshot::candidate_margin)
      .def_readwrite("entropy_norm", &ReplaySnapshot::entropy_norm)
      .def_readwrite("max_prob", &ReplaySnapshot::max_prob)
      .def_readwrite("switch_event", &ReplaySnapshot::switch_event)
      .def_readwrite("switch_reason", &ReplaySnapshot::switch_reason)
      .def_readwrite("binding_confidence", &ReplaySnapshot::binding_confidence)
      .def_readwrite("z_audit_conflict_count", &ReplaySnapshot::z_audit_conflict_count)
      .def_readwrite("degraded_single_obs_mode", &ReplaySnapshot::degraded_single_obs_mode)
      .def_readwrite("single_obs_streak", &ReplaySnapshot::single_obs_streak);

  py::class_<OfflineTrackerReplay>(m, "OfflineTrackerReplay")
      .def(py::init<>())
      .def("reset", &OfflineTrackerReplay::reset,
           py::arg("dt") = 0.05,
           py::arg("default_r1") = 0.15,
           py::arg("default_r2") = 0.20,
           py::arg("default_dza") = 0.0,
           py::arg("timeout_seconds") = 0.5,
           py::arg("enable_oscillation") = false)
      .def("clear", &OfflineTrackerReplay::clear)
      .def("set_fixed_robot_id", &OfflineTrackerReplay::set_fixed_robot_id)
      .def("fixed_robot_id", &OfflineTrackerReplay::fixed_robot_id,
           py::return_value_policy::reference_internal)
      .def("apply_numeric_overrides", &OfflineTrackerReplay::apply_numeric_overrides)
      .def("feed", &OfflineTrackerReplay::feed)
      .def("feed_empty", &OfflineTrackerReplay::feed_empty)
      .def("snapshot", &OfflineTrackerReplay::snapshot);
}
