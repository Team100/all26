# pylint: disable=C0301,E0611,E1101,R0903,R0913,R0917
import numpy as np
from numpy.typing import NDArray

from app.camera.camera_protocol import Camera
from app.config.identity import Identity
from app.dashboard.display_protocol import Display
from app.interpreter.interpreter_protocol import Interpreter
from app.interpreter.viewfinder import Viewfinder
from app.analysis.apriltags import AprilTags
from app.analysis.balls import Balls
from app.interpreter.dual_interpreter import DualInterpreter
from app.network.network_protocol import Network
from app.util.timestamps import Timestamps

USE_NULL: bool = False


class InterpreterFactory:
    @staticmethod
    def get(
        identity: Identity,
        cam: Camera,
        display1: Display,
        display2: Display,
        network: Network,
        timestamps: Timestamps,
    ) -> Interpreter:
        print("\n*** InterpreterFactory selecting an interpreter", flush=True)
        if USE_NULL:
            # For testing.
            return Viewfinder(display1, display2, network)

        # GREEN TARGET VALUES
        # object_lower = np.array((40, 50, 100))
        # object_higher = np.array((70, 255, 255))
        # get these values from changing the range till the object is highlighted

        # WHITE TARGET VALUES
        # object_lower: NDArray[np.int32] = np.array((0, 0, 200))
        # object_higher: NDArray[np.int32] = np.array((255, 150, 255))

        # YELLOW TARGET VALUES (outside)
       #  object_lower: NDArray[np.int32] = np.array((15, 100, 133))
       # object_higher: NDArray[np.int32] = np.array((35, 255, 255))

        # YELLOW TARGET VALUES (inside)
        object_lower: NDArray[np.int32] = np.array((15, 45, 45))
        object_higher: NDArray[np.int32] = np.array((35, 265, 260))

        # https://medium.com/@pkusolruangchai/find-hsv-range-interactive-sliders-in-opencv-1571c4c64433

        match identity:
            case Identity.FUNNEL:
                return DualInterpreter(
                    cam,
                    display1,
                    display2,
                    network,
                    timestamps,
                    AprilTags(identity, cam, network),
                    None,
                )
            case Identity.GAME_PIECE | Identity.COLOR_DETECT_1:
                return DualInterpreter(
                    cam,
                    display1,
                    display2,
                    network,
                    timestamps,
                    None,
                    Balls(cam, network, object_lower, object_higher),
                )
            case (
                Identity.DEV
                | Identity.DIST_TEST
                | Identity.LEFTAMP
                | Identity.CAMERA_BACK
                | Identity.JOELS_TEST
                | Identity.RIGHTAMP
                | Identity.SHOOTER
                | Identity.CLIMB_LEFT
                | Identity.CLIMB_RIGHT
                | Identity.SWERVE_RIGHT
                | Identity.SWERVE_LEFT
            ):
                return DualInterpreter(
                    cam,
                    display1,
                    display2,
                    network,
                    timestamps,
                    AprilTags(identity, cam, network),
                    None,
                )
            case Identity.CAMERA_FRONT | Identity.DEV2:
                return DualInterpreter(
                    cam,
                    display1,
                    display2,
                    network,
                    timestamps,
                    AprilTags(identity, cam, network),
                    Balls(cam, network, object_lower, object_higher),
                )
            case _:
                return DualInterpreter(
                    cam,
                    display1,
                    display2,
                    network,
                    timestamps,
                    AprilTags(identity, cam, network),
                    None,
                )
