# pylint: disable=C0103,E0611,E1101,E1121,R0902,R0903,R0913,R0914,R0917,W0212,W0612

from typing import override
from collections.abc import Buffer

import cv2
import numpy as np

from cv2.typing import MatLike
from numpy.typing import NDArray

from app.camera.camera_protocol import Camera, Request
from app.dashboard.display_protocol import Display
from app.dashboard.display_util import DisplayUtil
from app.decoder.decoder_protocol import Decoder
from app.analysis.analysis_protocol import ColorAnalysis, MonoAnalysis
from app.interpreter.interpreter_base import InterpreterBase
from app.network.network_protocol import Network
from app.util.timestamps import Timestamps


class DualInterpreter(InterpreterBase):
    """A detector for both color and monochrome analysis."""

    def __init__(
        self,
        cam: Camera,
        display1: Display,
        display2: Display,
        network: Network,
        timestamps: Timestamps,
        analyzer_mono: MonoAnalysis,
        analyzer_color: ColorAnalysis,
    ) -> None:
        """
        :cam: for intrinsic and distortion parameters
        :display1: for dashboard
        :display2: for debugging, switched by Network Tables
        :network: to read flags
        :timestamps: for timing
        :analyzer_mono: for monochrome, use None if you don't need it
        :analyzer_color: for color, use None if you don't need it
        """
        super().__init__(network)
        print("*** Interpreter: DualInterpreter")

        self._display1 = display1
        self._display2 = display2
        self._network = network
        self._timestamps = timestamps

        self._mtx: NDArray[np.float32] = cam.get_intrinsic()
        self._dist: NDArray[np.float32] = cam.get_dist()

        self._analyzer_mono = analyzer_mono
        self._analyzer_color = analyzer_color

        # network output for camera FPS
        self._fps = network.get_double_sender("fps")

    @override
    def interpret(self, req: Request) -> None:
        """Process both tags and objects from the BGR image."""
        with req.buffer() as buffer:
            #
            # Compute delay and servertime
            #
            # Capture timestamp in boottime.
            timestamp_boottime_us = req.timestamp_boottime_us()

            # Microsecond age of frame.
            delay_us = Timestamps.delta_us(timestamp_boottime_us)

            # Capture timestamp in servertime.
            servertime: int = self._timestamps.boot_time_to_server_time(
                timestamp_boottime_us
            )

            (img_bgr, img_mono, img_display) = self._images(buffer, req.decoder())

            if img_display is None:
                return

            if self._network.calibrate():
                # Save the raw image for calibration if requested
                self.write_calibration_image(img_display)

            if self._network.undistort_view():
                # Show the undistorted image only if requested.
                self._display2.put(cv2.undistort(img_display, self._mtx, self._dist))

            if img_mono is not None:
                self._analyzer_mono.analyze_mono(img_mono, img_display, servertime)

            if img_bgr is not None:
                self._analyzer_color.analyze_color(img_bgr, img_display, servertime, 800)

            # Send camera FPS to network.
            fps: float = req.fps()
            self._fps.send(fps)
            self.log_temperature()

            # Must flush!  Otherwise, 100ms update rate.
            self._network.flush()

            # Do the drawing after the NT payload is written to minimize latency.
            # This is not particularly fast or important for prod.
            DisplayUtil.text(img_display, f"FPS {fps:2.0f}", (5, 5), 0)
            DisplayUtil.text(img_display, f"delay (ms) {delay_us/1000:2.0f}", (5, 5), 1)
            self._display1.put(img_display)

    def _images(
        self, buffer: Buffer, decoder: Decoder
    ) -> tuple[MatLike | None, MatLike | None, MatLike | None]:
        """Create images for analysis and display."""
        # Images for analysis, do not modify.
        img_bgr = None
        img_mono = None
        # Image for display, ok to modify.
        img_display = None

        if self._analyzer_color:
            img_bgr = decoder.color(buffer)
        if self._analyzer_mono:
            if img_bgr is not None:
                # Convert BGR to grayscale.
                img_mono = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
                img_mono = np.ascontiguousarray(img_mono)
            else:
                # Decode to mono.
                img_mono = decoder.mono(buffer)
        if img_bgr is not None:
            img_display = img_bgr.copy()
        elif img_mono is not None:
            img_display = img_mono.copy()

        return (img_bgr, img_mono, img_display)
