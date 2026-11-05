# pylint: disable=E0401

from typing import Any
from typing import override
from app.camera.config.config_protocol import Config
from app.camera.size import Size
from app.decoder.yuyv_decoder import YuyvDecoder


class ConfigThriftyYuyv(Config):
    """The Thrifty Cam camera, using the YUYV encoding.

    The YUYV encoding seems worse for all uses than the
    MJPEG encoding, because YUYV moves much more data,
    and because YUYV is an interleaved encoding, so it
    is not particuarly fast to decode.
    """

    def __init__(self, size: Size) -> None:
        print("*** Config: ConfigThriftyYuyv")
        self._size = size

    @override
    def decoder(self) -> YuyvDecoder:
        return YuyvDecoder(self._size)

    @override
    def extra_delay_ms(self) -> float:
        return 2.5

    @override
    def main(self) -> dict[str, Any]:
        return {"format": "YUYV", "size": (self._size.width, self._size.height)}

    @override
    def controls(self) -> dict[str, Any]:
        return {
            # ANALOGUE GAIN
            # To minimize blur, set this as high as possible.
            # On the Thiftycam the range is [1.0, 4.0]
            # 4.0 seems to produce lots of noise
            "AnalogueGain": 2.0,
            #
            # AUTO EXPOSURE
            # Must be true for outside or in bright sun.
            # "AeEnable": True,
            "AeEnable": False,
            #
            # EXPOSURE TIME (microseconds)
            # Minimizes blur.  Requires pretty good light.
            # "ExposureTime": 500,
            # Works in less light, slightly more blur.
            "ExposureTime": 1000,
        }
