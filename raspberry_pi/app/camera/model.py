# pylint: disable=E0401

from enum import Enum, unique
from typing import Any


@unique
class Model(Enum):
    V3_WIDE = "imx708_wide"
    V2 = "imx219"
    GS = "imx296"
    THRIFTY = "SPCA2630 PC Camera: PC Camera"
    ARDU_OV9782 = "Arducam OV9782 USB Camera: Ardu"
    UNKNOWN = "unknown"

    @classmethod
    def _missing_(cls, value: object) -> Any:
        return Model.UNKNOWN

    @staticmethod
    def get(camera_properties: dict[str, str]) -> "Model":  # type:ignore
        """Use the properties to avoid import errors in tests."""
        model_str: str = camera_properties["Model"]  # type:ignore
        print("*** Camera model string:", model_str)
        model: Model = Model(model_str)
        print("*** Camera model:", model)
        return model
