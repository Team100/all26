import numpy as np
from numpy.typing import NDArray

from app.config.identity import Identity


class Distortion:
    """Camera distortion matrix.

    This should be measured using mrcal.

    We've had good results using the four-parameter model: [k1, k2, p1, p2].
    

    Using the 3.2mm lens and GS camera:
    * k1 is quadratic radial, should be around -0.3
    * k2 is quartic radial, should be around 0.1
    * p1 and p2 are tangential, should be very small.

    See https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html
    See https://docs.google.com/spreadsheets/d/1x2_58wyVb5e9HJW8WgakgYcOXgPaJe0yTIHew206M-M
    See doc/calibration.md.
    """

    def __init__(self, identity: Identity) -> None:
        print("*** Distortion for identity:", identity)
        self._dist: NDArray[np.float32]
        self._valid: bool = True
        match identity:
            #
            # Camerabot cameras 2026
            #
            case Identity.CAMERA_BACK:
                # Calibrated 2/28/26.
                self._dist = np.array(
                    [-0.291773408, 0.09223526926, 0.0001767030772, -8.758271042e-05]
                )
            case Identity.CAMERA_FRONT:
                # Calibrated 2/28/26.
                self._dist = np.array(
                    [-0.2961790762, 0.09865662084, -0.0001707410996, 0.0002482554604]
                )
            case Identity.FUNNEL:
                self._dist = np.array(
                    [-0.03226, 0.01730, 0.00105, 0.00284]
                )   
            #
            # TODO: clean up the entries below
            #

            #
            #
            #
            case Identity.CLIMB_LEFT:
                self._dist = np.array(
                    [-0.2944709861, 0.09625348382, 0.0002112286215, -0.0001536884092]
                )
            case Identity.CLIMB_RIGHT:
                self._dist = np.array(
                    [-0.2963480251, 0.1022134995, -2.639191492e-05, -0.0002157464781]
                )
            case Identity.SHOOTER:
                self._dist = np.array(
                    [-0.2896588494, 0.08913011574, -0.0003728505114, 0.0001180931071]
                )
            case _:
                self._dist = np.array(
                    [-0.2883685917, 0.08640059653, -0.0003167335742, 0.0004112607248]
                )

                self._valid = False
                print("###########################################")
                print("#                                         #")
                print("#  CAMERA DISTORTION IS NOT CALIBRATED!   #")
                print("#                                         #")
                print("#  YOU MUST CALIBRATE IT!!!!              #")
                print("#                                         #")
                print("###########################################")

        print("*** DISTORTION")
        print(self._dist)

    def get(self) -> NDArray[np.float32]:
        return self._dist

    def valid(self) -> bool:
        return self._valid
