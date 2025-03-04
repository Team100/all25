""" Choose a camera implementation based on identity."""

# pylint: disable=C0415

from app.camera.camera_protocol import Camera
from app.config.identity import Identity


class CameraFactory:
    @staticmethod
    def get(identity: Identity, camera_num: int) -> Camera:
        try:
            # this will fail if we're not running on a Raspberry Pi.
            from app.camera.real_camera import RealCamera

            return RealCamera(identity, camera_num)

        except ImportError:
            from app.camera.fake_camera import FakeCamera

            if camera_num == 0:
                # 1/4 scale
                # return FakeCamera("tag_and_board.jpg", (1100, 620), -5)
                # full-size (huge)
                return FakeCamera("tag_and_board.jpg", (5504, 3096), -0.1)
            return FakeCamera("blob.jpg")

    @staticmethod
    def get_num_cameras(identity: Identity) -> int:
        match identity:
            case Identity.UNKNOWN:
                return 2
            case (
                Identity.RIGHTAMP
                | Identity.LEFTAMP
                | Identity.SHOOTER
                | Identity.GAME_PIECE
                | Identity.GLOBAL_GAME_PIECE
                | Identity.CORAL_RIGHT
                | Identity.CORAL_LEFT
                | Identity.SWERVE_RIGHT
                | Identity.SWERVE_LEFT
                | Identity.FUNNEL
                | Identity.DEV # has one v2 camera at the moment
                | Identity.DIST_TEST
            ):
                return 1
            case Identity.FLIPPED:
                return 0
            case _:
                raise ValueError(f"Unknown identity: {identity}")
