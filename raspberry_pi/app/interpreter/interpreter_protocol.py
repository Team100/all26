# pylint: disable=C0114,C0115,C0116,E0401,E1101,R0903,W2301

from typing import Protocol

from app.camera.camera_protocol import Request


class Interpreter(Protocol):
    """Interface for request interpreters."""

    def interpret(self, req: Request) -> None: ...
