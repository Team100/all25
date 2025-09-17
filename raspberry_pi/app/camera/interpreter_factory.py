# pylint: disable=C0301,E0611,E1101,R0903
import numpy as np

from app.camera.camera_protocol import Camera
from app.camera.interpreter_protocol import Interpreter
from app.config.identity import Identity
from app.dashboard.fake_display import FakeDisplay
from app.dashboard.real_display import RealDisplay
from app.localization.combined_detector import CombinedDetector
from app.localization.note_detector import NoteDetector
from app.localization.null_detector import NullDetector
from app.localization.tag_detector import TagDetector
from app.network.network import Network


class InterpreterFactory:
    @staticmethod
    def get(
        identity: Identity, cam: Camera, camera_num: int, network: Network
    ) -> Interpreter:
        # GREEN TARGET VALUES
        # object_lower = np.array((40, 50, 100))
        # object_higher = np.array((70, 255, 255))
        # get these values from changing the range till the object is highlighted
        # WHITE TARGET VALUES
        object_lower = np.array((0, 0, 150))
        object_higher = np.array((255, 100, 255))
        size = cam.get_size()
        if identity == Identity.DIST_TEST:
            scale = 1.0
        elif identity == Identity.DEV:
            scale = 1.0
        elif identity != Identity.UNKNOWN:
            scale = 0.25
        else:
            scale = 1.0
        match identity:
            case Identity.GAME_PIECE:
                display = RealDisplay(
                    int(scale * size.width),
                    int(scale * size.height),
                    "note" + str(camera_num),
                )
                return NoteDetector(
                    identity,
                    cam,
                    camera_num,
                    display,
                    network,
                    object_lower,
                    object_higher,
                )
            case (
                Identity.RIGHTAMP
                | Identity.LEFTAMP
                | Identity.SHOOTER
                | Identity.GLOBAL_GAME_PIECE 
                | Identity.SWERVE_RIGHT
                | Identity.SWERVE_LEFT
                | Identity.FUNNEL
                | Identity.DIST_TEST
                | Identity.JOELS_TEST
            ):
                display = RealDisplay(
                    int(scale * size.width),
                    int(scale * size.height),
                    "tag" + str(camera_num),
                )
                return TagDetector(identity, cam, camera_num, display, network)
            case (Identity.DEV|Identity.DEV2| Identity.CORAL_RIGHT| Identity.CORAL_LEFT):
                display = RealDisplay(
                    int(scale * size.width),
                    int(scale * size.height),
                    "combined" + str(camera_num),
                )
                return CombinedDetector(identity, cam, camera_num, display, network, object_lower, object_higher)
            case _:
                display = FakeDisplay()
                return TagDetector(identity, cam, camera_num, display, network)
