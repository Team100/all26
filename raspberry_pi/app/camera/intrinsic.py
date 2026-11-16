# pylint: disable=C0114,C0115,C0116,E0401,R0801,R0903,R0912,W0613
from datetime import datetime

import numpy as np
from numpy.typing import NDArray

from app.config.identity import Identity


class Intrinsic:
    """Camera intrinsic matrix.

    This should be measured using mrcal.

    Intrinsic matrix is:

    fx 0 cx
    0 fy cy
    0 0 1

    For the GS camera with the 3.2mm lens, typical values are

    * fx and fy (focal length) around 940
    * cx (center) around 680
    * cy (center) around 530

    Note the wide variance in cx and cy, which corresponds to variation sensor/lens alignment.

    See doc/calibration.md.
    """

    def __init__(self, identity: Identity) -> None:
        print(f"[{datetime.now()}] " + "*** Intrinsic for identity:", identity)
        self._mtx: NDArray[np.float32]
        self._valid: bool = True
        match identity:
            #
            # Camerabot cameras 2026
            #
            case Identity.CAMERA_BACK:
                # Calibrated 2/28/26.
                self._mtx = np.array(
                    [
                        [943.2920845, 0.0, 681.6154074],
                        [0.0, 943.3584029, 529.4306382],
                        [0, 0, 1],
                    ]
                )
            case Identity.CAMERA_FRONT:
                # Calibrated 2/28/26.
                self._mtx = np.array(
                    [
                        [1378.6815, 0.0, 768.4795],
                        [0.0, 1376.3448, 693.7945],
                        [0, 0, 1],
                    ]
                )
            case Identity.CAMERA_2A:
                # Calibrated 10/14.
                self._mtx = np.array(
                    [
                        [1378.681461, 0.0, 768.4795113],
                       [0.0, 1376.344836, 693.79454],
                       [0, 0, 1],

                    ]
                )
            case Identity.BALL_DETECT:
                # Calibrated never.
                # TODO: do it
                self._mtx = np.array(
                    [
                        [874.8730398, 0.0, 875.8991441],
                        [0.0, 954.7454588, 609.6603374],
                        [0, 0, 1],
                    ]
                )
            case Identity.COLOR_DETECT_1:
                # Calibrated 10/12/26.
                # TODO: do it
                self._mtx = np.array(
                    [
                        [872.4186642, 0.0, 871.2316943],
                        [0.0, 967.5641454, 610.2786945],
                        [0, 0, 1],
                    ]
                )
            case Identity.FUNNEL:
                # Calibrated on 07/14/26
                self._mtx = np.array(
                    [
                        [1378.6815, 0.0, 768.4795],
                        [0.0, 1376.3448, 693.7945],
                        [0, 0, 1],
                    ]
                )

            # TODO: clean up the entries below
            #

            #
            #
            #
            case Identity.DEV:
                self._mtx = np.array(
                    [
                        [944.3507484, 0.0, 693.7105365],
                        [0.0, 943.8611003, 498.1103206],
                        [0, 0, 1],
                    ]
                )
            case Identity.DIST_TEST:
                self._mtx = np.array(
                    [
                        [944.3507484, 0.0, 693.7105365],
                        [0.0, 943.8611003, 498.1103206],
                        [0, 0, 1],
                    ]
                )
            case Identity.FLIPPED:
                self._mtx = np.array(
                    [
                        [944.3507484, 0.0, 693.7105365],
                        [0.0, 943.8611003, 498.1103206],
                        [0, 0, 1],
                    ]
                )
            case Identity.SIM0:
                self._mtx = np.array(
                    [
                        [944.3507484, 0.0, 693.7105365],
                        [0.0, 943.8611003, 498.1103206],
                        [0, 0, 1],
                    ]
                )
            case Identity.CLIMB_LEFT:
                self._mtx = np.array(
                    [
                        [937.8076198, 0.0, 675.5811099],
                        [0.0, 938.9070878, 529.2441371],
                        [0, 0, 1],
                    ]
                )
            case Identity.CLIMB_RIGHT:
                self._mtx = np.array(
                    [
                        [937.5666656, 0.0, 735.8798376],
                        [0.0, 939.1987448, 543.5501318],
                        [0, 0, 1],
                    ]
                )
            case Identity.SHOOTER:
                self._mtx = np.array(
                    [
                        [937.5578917, 0.0, 674.2476937],
                        [0.0, 932.0791377, 528.6425504],
                        [0, 0, 1],
                    ]
                )
            case Identity.DESKTOP:
                self._mtx = np.array(
                    [
                        [944.3507484, 0.0, 693.7105365],
                        [0.0, 943.8611003, 498.1103206],
                        [0, 0, 1],
                    ]
                )
            case _:
                self._mtx = np.array(
                    [
                        [944.3507484, 0.0, 693.7105365],
                        [0.0, 943.8611003, 498.1103206],
                        [0, 0, 1],
                    ]
                )
                self._valid = False
                print(
                    f"[{datetime.now()}] "
                    + "##########################################"
                )
                print(
                    f"[{datetime.now()}] "
                    + "#                                        #"
                )
                print(
                    f"[{datetime.now()}] "
                    + "#  CAMERA INTRINSIC IS NOT CALIBRATED!   #"
                )
                print(
                    f"[{datetime.now()}] "
                    + "#                                        #"
                )
                print(
                    f"[{datetime.now()}] "
                    + "#  YOU MUST CALIBRATE IT!!!!             #"
                )
                print(
                    f"[{datetime.now()}] "
                    + "#                                        #"
                )
                print(
                    f"[{datetime.now()}] "
                    + "##########################################"
                )

        print(f"[{datetime.now()}] " + "*** INTRINSIC")
        print(self._mtx)

    def get(self) -> NDArray[np.float32]:
        return self._mtx

    def valid(self) -> bool:
        return self._valid
