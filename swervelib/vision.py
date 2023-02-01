from dataclasses import dataclass

import photonvision
import wpimath.geometry
import robotpy_apriltag as apriltag


@dataclass
class CameraDefinition:
    name: str
    translation: wpimath.geometry.Transform3d


class AprilTagCameraCollection:
    def __init__(self, camera_definitions: list[CameraDefinition], field_layout: apriltag.AprilTagFieldLayout):
        cameras = [
            (photonvision.PhotonCamera(definition.name), definition.translation) for definition in camera_definitions
        ]
        self.pose_estimator = photonvision.RobotPoseEstimator(
            field_layout, photonvision.PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cameras
        )

    def estimate_pose(self, previous_estimated_pose: wpimath.geometry.Pose2d) -> wpimath.geometry.Pose2d:
        self.pose_estimator.setReferencePose(wpimath.geometry.Pose3d(previous_estimated_pose))
        # The update() method returns a tuple with an estimated pose as its 0th element and a timestamp as its 1st.
        new_pose = self.pose_estimator.update()[0]
        return new_pose.toPose2d()
