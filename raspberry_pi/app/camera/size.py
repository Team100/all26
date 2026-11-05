from dataclasses import dataclass

from app.camera.model import Model


@dataclass(frozen=True, kw_only=True)
class Size:
    """Sensor width and height must be equal to one of the 'size' options
    in the list of sensor formats."""

    sensor_width: int
    sensor_height: int
    width: int
    height: int

    @staticmethod
    def from_model(model: Model) -> "Size":
        match model:
            case Model.V3_WIDE:
                return Size(
                    sensor_width=2304,
                    sensor_height=1296,
                    width=1152,
                    height=648,
                )
            case Model.V2:
                return Size(
                    sensor_width=1640,
                    sensor_height=1232,
                    width=832,
                    height=616,
                )
            case Model.GS:
                return Size(
                    sensor_width=1456,
                    sensor_height=1088,
                    width=1408,
                    height=1088,
                )
            case Model.THRIFTY:
                return Size(
                    sensor_width=1600,
                    sensor_height=1304,
                    width=1600,
                    height=1304,
                )
            case Model.ARDU_OV9782:
                return Size(
                    sensor_width=1280,
                    sensor_height=800,
                    width=1280,
                    height=800
                )
            case Model.ARDU_BO61O:
                return Size(
                    sensor_width=1920,
                    sensor_height=1200,
                    width=1920,
                    height=1200
                )
            case _:
                return Size(
                    sensor_width=100,
                    sensor_height=100,
                    width=100,
                    height=100,
                )
