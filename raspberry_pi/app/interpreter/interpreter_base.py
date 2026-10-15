# pylint: disable=E0611,E1101,R0902,R0903,R0913,R0914,R0917,W0212,W0611

import os
import cv2
import ntcore
import numpy as np
from numpy.typing import NDArray
from app.interpreter.interpreter_protocol import Interpreter
from app.network.network_protocol import Network

IMAGE_DIR: str = "images"


class InterpreterBase(Interpreter):
    """Base implementation for interpreter classes, handles logging
    CPU temperature and writing calibration images."""

    def __init__(
        self,
        network: Network,
    ) -> None:
        # Network output for CPU temperature
        self._temp = network.get_double_sender("temp")
        # To keep track of images to write
        self.img_ts_sec: int = 0
        # Make a place to put example images.
        cwd = os.getcwd()
        print("*** CWD:", cwd)
        if not os.path.exists(IMAGE_DIR):
            os.mkdir(IMAGE_DIR)

    def log_temperature(self) -> None:
        """Log the CPU temperature in Celsius.

        The raspberry pi throttles at 80 C.
        """
        try:
            with open(
                "/sys/class/thermal/thermal_zone0/temp", "r", encoding="ascii"
            ) as f:
                raw_temp: int = int(f.read().strip())
                temp_c: float = raw_temp / 1000
                self._temp.send(temp_c)
        except IOError:
            # This file does not exist (e.g. on Windows etc)
            pass

    def write_calibration_image(self, img: NDArray[np.uint8]) -> None:
        """Write an image for later analysis (e.g. calibration).

        WARNING! This is VERY VERY SLOW, like 1 FPS.

        To retrieve these files, use, e.g.:

        scp pi@10.1.0.11:images/* .

        Note the single dot at the end of the line above, which means "current directory"

        These will accumulate forever, so remember to clean it out:

        ssh pi@10.1.0.11 "rm images/img*"
        """
        now_us: int = ntcore._now()
        now_s: int = now_us // 1000000  # once per second
        if now_s > self.img_ts_sec:
            self.img_ts_sec = now_s
            filename: str = IMAGE_DIR + "/img" + str(now_s) + ".png"
            cv2.imwrite(filename, img)
