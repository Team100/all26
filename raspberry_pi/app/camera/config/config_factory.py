# pylint: disable=R0903,W0611

from app.camera.config.config_ardu_mjpeg import ConfigArduMjpeg
from app.camera.config.config_protocol import Config
from app.camera.size import Size
from app.config.identity import Identity
from app.camera.config.config_thrifty_mjpeg import ConfigThriftyMjpeg
from app.camera.config.config_gs_color import ConfigGsColor
from app.camera.config.config_gs_mono import ConfigGsMono


class ConfigFactory:
    """Choose a camera confing based on identity."""

    @staticmethod
    def get(identity: Identity, size: Size) -> Config:
        print("*** CameraFactory for identity:", identity)
        match identity:
            case Identity.FUNNEL:
                # test the RPi GS camera
                # return ConfigGsMono(size)
                # test the ThriftyCam
                return ConfigThriftyMjpeg(size)
                # return ConfigThriftyYuyv(size)
            case Identity.CLIMB_RIGHT:
                return ConfigThriftyMjpeg(size)
            case Identity.CLIMB_LEFT :
                return ConfigGsColor(size)
            case Identity.SHOOTER:
                return ConfigGsMono(size)
            case Identity.CAMERA_FRONT:
                return ConfigArduMjpeg(size)
            case Identity.CAMERA_2A:
                return ConfigArduMjpeg(size)
            case Identity.COLOR_DETECT_1:
                return ConfigArduMjpeg(size)
            case Identity.BALL_DETECT:
                return ConfigArduMjpeg(size)
            case _:
                return ConfigGsMono(size)
