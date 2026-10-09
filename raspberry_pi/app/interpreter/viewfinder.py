# pylint: disable=C0103,E1101,R0903
from typing import  override
from collections.abc import Buffer
from cv2.typing import MatLike
from app.camera.camera_protocol import Request
from app.dashboard.display_protocol import Display
from app.dashboard.display_util import DisplayUtil
from app.decoder.decoder_protocol import Decoder
from app.interpreter.interpreter_protocol import Interpreter
from app.network.network_protocol import Network
from app.util.timestamps import Timestamps


class Viewfinder(Interpreter):
    """An interpreter that does nothing but publish its input."""

    def __init__(
        self,
        display1: Display,
        display2: Display,
        network: Network,
    ) -> None:
        print("*** Interpreter: Viewfinder")
        self._display1 = display1
        self._display2 = display2
        # network output for camera FPS
        self._fps = network.get_double_sender("fps")


    @override
    def interpret(self, req: Request) -> None:
        buffer: Buffer
        with req.buffer() as buffer:
            decoder: Decoder = req.decoder()
            img_bgr: MatLike | None = decoder.color(buffer)
            if img_bgr is None:
                return
            fps = req.fps()
            self._fps.send(fps)
            delay_us = Timestamps.delta_us(req.timestamp_boottime_us())
            DisplayUtil.text(img_bgr, f"FPS {fps:2.0f}", (5, 5), 0)
            DisplayUtil.text(img_bgr, f"delay (ms) {delay_us/1000:2.0f}", (5, 5), 1)
            self._display1.put(img_bgr)
            self._display2.put(img_bgr)
