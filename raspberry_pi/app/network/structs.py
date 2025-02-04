""" Interface for Network Tables-like things. """

# pylint: disable=C0301,R0902,R0903,W0212,W2301

import dataclasses

import numpy as np
from wpimath.geometry import Pose3d, Transform3d
from wpiutil import wpistruct


F = ".3f"


@wpistruct.make_wpistruct  # type:ignore
@dataclasses.dataclass
class Blip24:
    """AprilTag target pose used in 2024"""

    id: int
    pose: Transform3d




@wpistruct.make_wpistruct
@dataclasses.dataclass
class PoseEstimate25:
    """Result of the pose estimator."""

    # most-recent state (corresponding to the NT timestamp)
    # TODO: make this a pose2d
    x: float
    y: float
    theta: float
    # std dev of most-recent state (sqrt of diagonal of marginal covariance)
    # TODO: make this a twist2d
    x_sigma: float
    y_sigma: float
    theta_sigma: float
    # twist of most-recent odometry
    # TODO: make this a twist2d
    dx: float
    dy: float
    dtheta: float
    # time between next-most-recent and most-recent
    dt: float

    def __str__(self) -> str:
        return (
            f"(x {self.x:{F}} y {self.y:{F}} Θ {self.theta:{F}} "
            f"sx {self.x_sigma:{F}} sy {self.y_sigma:{F}} sΘ {self.theta_sigma:{F}} "
            f"dx {self.dx:{F}} dy {self.dy:{F}} dΘ {self.dtheta:{F}} "
            f"dt {self.dt:{F}})"
        )






