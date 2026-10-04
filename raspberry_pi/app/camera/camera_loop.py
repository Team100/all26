# pylint: disable=R0903

from threading import Event

from typing import override

from app.camera.camera_protocol import Camera
from app.interpreter.interpreter_protocol import Interpreter
from app.framework.looper import Looper


class CameraLoop(Looper):
    """Capture a request, interpret it, release it, repeat."""

    def __init__(
        self,
        camera: Camera,
        interpreter: Interpreter,
        done: Event,
    ) -> None:
        super().__init__(done)
        print("*** Looper: CameraLoop")
        self._interpreter: Interpreter = interpreter
        self._camera: Camera = camera

    @override
    def execute(self) -> None:
        req = self._camera.capture_request()
        try:
            self._interpreter.interpret(req)
        finally:
            req.release()

    @override
    def end(self) -> None:
        self._camera.stop()
