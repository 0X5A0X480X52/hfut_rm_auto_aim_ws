// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_CORE_IDS_HPP_
#define MAX_ENTROPY_TRACKER_CORE_IDS_HPP_

namespace fyt::auto_aim {

/// Per-frame detection index; valid only within the current frame.
using DetectionId = int;

/// Image-domain short-term continuous instance identity.
using Track2DId = int;

/// 3D single-armor track identity (managed by SingleArmorProxyManager).
using Track3DId = int;

/// Full robot tracker asset identity (managed by TrackerManager).
using RobotTrackId = int;

/// Candidate panel index in the robot structural model (0..3 for Norm4).
using PanelId = int;

/// Business-layer armor number.
using ArmorId = int;

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_CORE_IDS_HPP_
