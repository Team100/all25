""" This is a wrapper for network tables. """

# pylint: disable=R0902,R0903,W0212

# github workflow crashes in ntcore, bleah
from typing import cast

import ntcore
from wpimath.geometry import Pose2d, Rotation2d, Rotation3d
from wpiutil import wpistruct

from app.config.identity import Identity
from app.network.structs import (
    Blip24,
    PoseEstimate25,
)

# global singleton
start_time_us = ntcore._now()


class DoubleSender:
    def __init__(self, pub: ntcore.DoublePublisher) -> None:
        self.pub = pub

    def send(self, val: float, delay_us: int) -> None:
        self.pub.set(val, int(ntcore._now() - delay_us))


class BlipSender:
    def __init__(self, pub: ntcore.StructArrayPublisher) -> None:
        self.pub = pub

    def send(self, val: list[Blip24], delay_us: int) -> None:
        self.pub.set(val, int(ntcore._now() - delay_us))


class NoteSender:
    def __init__(self, pub: ntcore.StructArrayPublisher) -> None:
        self.pub = pub

    def send(self, val: list[Rotation3d], delay_us: int) -> None:
        self.pub.set(val, int(ntcore._now() - delay_us))

class PoseSender:
    def __init__(self, pub: ntcore.StructPublisher) -> None:
        self.pub = pub

    def send(self, val: PoseEstimate25, delay_us: int) -> None:
        self.pub.set(val, int(ntcore._now() - delay_us))

class Network:
    def __init__(self, identity: Identity) -> None:
        # TODO: use identity.name instead
        self._serial: str = identity.value
        self._inst: ntcore.NetworkTableInstance = (
            ntcore.NetworkTableInstance.getDefault()
        )
        self._inst.startClient4("tag_finder24")

        ntcore._now()

        # roboRio address. windows machines can impersonate this for simulation.
        # also localhost for testing
        if identity == Identity.UNKNOWN:
            # vasili says this doesn't work, but i need it for testing.
            self._inst.setServer("localhost")
        else:
            # this works
            self._inst.setServer("10.1.0.2")

    def now(self) -> int:
        return ntcore._now() - start_time_us

    def get_double_sender(self, name: str) -> DoubleSender:
        return DoubleSender(self._inst.getDoubleTopic(name).publish())

    def get_blip_sender(self, name: str) -> BlipSender:
        return BlipSender(self._inst.getStructArrayTopic(name, Blip24).publish())

    def get_note_sender(self, name: str) -> NoteSender:
        return NoteSender(
            self._inst.getStructArrayTopic(name, Rotation3d).publish()
        )

    def get_pose_sender(self, name: str) -> PoseSender:
        return PoseSender(self._inst.getStructTopic(name, PoseEstimate25).publish())

    def flush(self) -> None:
        self._inst.flush()
