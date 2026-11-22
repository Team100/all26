# pylint: disable=C0114,C0115,C0116,E0401
from datetime import datetime
from enum import Enum, unique
from typing import Any


@unique
class Model(Enum):
    V3_WIDE = "imx708_wide"
    V2 = "imx219"
    GS = "imx296"
    THRIFTY = "SPCA2630 PC Camera: PC Camera"
    ARDU_OV9782 = "Arducam OV9782 USB Camera: Ardu"
    ARDU_BO61O = "Arducam Global Shutter 2.3MP"
    UNKNOWN = "unknown"

    @classmethod
    def _missing_(cls, value: object) -> Any:
        return Model.UNKNOWN

    @staticmethod
    def get(camera_properties: dict[str, str]) -> "Model":  # type: ignore
        """Use the properties to avoid import errors in tests."""
        model_str: str = camera_properties["Model"]  # type: ignore
        print(f"[{datetime.now()}] " + "*** Camera model string:", model_str)
        model: Model = Model(model_str)
        print(f"[{datetime.now()}] " + "*** Camera model:", model)
        return model
