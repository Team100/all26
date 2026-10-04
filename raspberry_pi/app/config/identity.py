# pylint: disable=C0103

from enum import Enum, unique
from typing import Any


def _read_cpu_info() -> str:
    with open("/proc/cpuinfo", "r", encoding="ascii") as cpuinfo:
        for line in cpuinfo:
            if line[0:6] == "Serial":
                return line[10:26]
    return ""


try:
    _serial = _read_cpu_info()
except FileNotFoundError:
    # windows doesn't have this file
    _serial = ""


@unique
class Identity(Enum):
    """Represents the Raspberry Pi identity, used to select configuration.

    Don't put anything in this class about the actual configuration, use a different class for that.

    Keep this synchronized with java team100.config.Camera.
    """

    #
    # Camera-bot 2026
    #
    CAMERA_BACK = "d44649628c20d4d4"
    """2026 camerabot, 10.1.0.24"""
    CAMERA_FRONT = "8ddb2ed6c49a9bce"
    """2026 camerabot, 10.1.0.32"""

    #
    # TODO: clean up the entries below
    #

    #
    # Comp bot 2024
    #
    DEV2 = "10000000a7a892c0"
    RIGHTAMP = "10000000caeaae82"
    LEFTAMP = "100000004e0a1fb9"
    DIST_TEST = "cf206cd348d20bdd"

    # Comp bot 2025
    FUNNEL = "1e5acbaa5a7f9d10"  # 30

    # Calgames Partner
    COLOR_DETECT_1 = "e1714bf12b94e414"  # 35

    # Comp bot 2026
    CLIMB_LEFT = "82c4c3fe4f941e96"  # 34
    CLIMB_RIGHT = "364f07fb090a3bf7"  # 31
    SHOOTER = "e47055a1bcbcead0"  # 33
    GAME_PIECE = "FILL THIS IN"  #

    SWERVE_RIGHT = "47403d5eafe002a9"  # 34
    SWERVE_LEFT = "8132c256feeee63bbb4e"  # 33

    DEV = "test"
    FLIPPED = "flipme"  # example for per-identity config
    JOELS_TEST = "9b297e919a0634e6"
    SIM0 = "sim0"
    """For testing."""
    UNKNOWN = "unknown"

    @classmethod
    def _missing_(cls, value: object) -> Any:
        return Identity.UNKNOWN

    @staticmethod
    def get() -> "Identity":
        """Returns UNKNOWN if serial is not recognized, and in tests."""
        serial = _serial
        print("*** Coprocessor serial:", serial)
        identity: Identity = Identity(serial)
        print("*** Coprocessor identity:", identity)
        return identity
