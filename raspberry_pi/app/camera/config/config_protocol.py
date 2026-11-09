# from datetime import datetime
# pylint: disable=E0401,R0903,W2301

from typing import Any
from typing import Protocol
from picamera2 import libcamera  # type: ignore

from app.config.identity import Identity
from app.decoder.decoder_protocol import Decoder


class Config(Protocol):
    """Interface for camera config, with some defaults.
    Mostly corresponds to the arguments for
    Picamera2.create_still_configuration()."""

    def decoder(self) -> Decoder:
        """Decoder for the format of this config."""
        ...

    def extra_delay_ms(self) -> float:
        """Delay found using the camera delay tester, in milliseconds."""
        ...

    def buffer_count(self) -> int:
        """Even though we always take the most-recent buffer,
        adding a few more seems to make the pipeline a little smoother."""
        return 5

    def queue(self) -> bool:
        """Without queueing, every capture waits for the current
        frame, which means less FPS and more latency."""
        return True

    def sensor(self) -> dict[str, Any]:
        """RPI cameras only, size and bit depth."""
        return {}

    def raw(self) -> Any:
        """RPI Cameras only, raw Bayer or mono frames.
        We never use these."""
        return None

    def main(self) -> dict[str, Any]:
        """Encoding format and size."""
        ...

    def transform(self, identity: Identity) -> libcamera.Transform:  # type: ignore
        """Flip for upside-down cameras.
        See libcamera/src/libcamera/transform.cpp."""
        match identity:
            case Identity.FLIPPED | Identity.CLIMB_RIGHT:
                return libcamera.Transform(  # type: ignore
                    rotation=0,
                    hflip=True,
                    vflip=True,
                    transpose=False,
                )
            case _:
                return libcamera.Transform()  # type: ignore

    def controls(self) -> dict[str, Any]:
        """Gain, exposure, etc.
        See Picamera2.camera_controls for available controls
        for each camera type."""
        ...

    def ok(self, conf: dict[str, Any], cam: dict[str, Any]) -> bool:
        """Check the camera config against the desired."""
        return True
