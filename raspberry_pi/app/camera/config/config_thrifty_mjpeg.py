# pylint: disable=E0401

from typing import Any
from typing import override

from app.camera.config.config_protocol import Config
from app.camera.size import Size
from app.decoder.mjpeg_decoder import MjpegDecoder


class ConfigThriftyMjpeg(Config):
    """The Thrifty Cam camera, using the MJPEG encoding.
    
    The MJPEG encoding seems better for all uses than the
    YUYV encoding, because YUYV moves much more data.
    """

    def __init__(self, size: Size) -> None:
        print("*** Config: ConfigThriftyMjpeg")
        self._size = size

    @override
    def decoder(self) -> MjpegDecoder:
        return MjpegDecoder(self._size)

    @override
    def extra_delay_ms(self) -> float:
        return 2.5

    @override
    def main(self) -> dict[str, Any]:
        return {"format": "MJPEG", "size": (self._size.width, self._size.height)}

    @override
    def controls(self) -> dict[str, Any]:
        return {
            # ANALOGUE GAIN
            # To minimize blur, set this as high as possible.
            # On the Thiftycam the range is [1.0, 4.0]
            # 4.0 seems to produce lots of noise
            "AnalogueGain": 1.0,
            #
            # AUTO EXPOSURE
            # Must be true for outside or in bright sun.
            # "AeEnable": True,
            "AeEnable": False,
            #
            # EXPOSURE TIME (microseconds)
            "ExposureTime": 500,
        }
