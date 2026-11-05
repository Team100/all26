from datetime import datetime
# pylint: disable=R0903

from typing import Protocol
from app.camera.camera_protocol import Request


class Interpreter(Protocol):
    """Interface for request interpreters."""

    def interpret(self, req: Request) -> None: ...
