# from datetime import datetime
# pylint: disable=C0301,R0902,R0903,W0212,W2301

import dataclasses

from wpimath.geometry import Rotation3d, Transform3d
from wpiutil import wpistruct


@wpistruct.make_wpistruct  # type:ignore
@dataclasses.dataclass
class Blip:
    """AprilTag pose"""

    timestamp: wpistruct.int64
    """server microseconds"""
    id: wpistruct.int32
    """tag id"""
    pose: Transform3d
    """camera-relative"""


@wpistruct.make_wpistruct  # type:ignore
@dataclasses.dataclass
class BlipWithCorners:
    """AprilTag pose with corner pixels (x,y)"""

    timestamp: wpistruct.int64
    """Server time in microseconds."""
    id: wpistruct.int32
    """Apriltag id number."""
    # The corners are laid out as primitives because python can't do an array.
    # https://github.com/robotpy/mostrobotpy/issues/272
    x0: float
    """Lower-left x."""
    y0: float
    """Lower-left y."""
    x1: float
    """Lower-right x."""
    y1: float
    """Lower-right y."""
    x2: float
    """Upper-right x."""
    y2: float
    """Upper-right y."""
    x3: float
    """Upper-left x."""
    y3: float
    """Upper-left y."""
    pose: Transform3d
    """Camera-relative pose."""

    @classmethod
    def make(
        cls,
        m_timestamp: wpistruct.int64,
        m_id: wpistruct.int32,
        corners: tuple[float, float, float, float, float, float, float, float],
        m_pose: Transform3d,
    ) -> "BlipWithCorners":
        """Make a BlipWithCorners using the corners tuple."""
        return cls(m_timestamp, m_id, *corners, m_pose)


@wpistruct.make_wpistruct  # type:ignore
@dataclasses.dataclass

class Target:

    # """Game piece target"""
    #area: int
    """target area"""
    timestamp: wpistruct.int64
    """server microseconds"""
    sight: Rotation3d
    """camera-relative"""
    range: float
    """the target's distance in meters"""


@wpistruct.make_wpistruct  # type:ignore
@dataclasses.dataclass
class SyncRequest:
    """Clock sync request packet.
    See lib/network/SYNC.md."""

    org: wpistruct.int64
    """client microseconds"""


@wpistruct.make_wpistruct  # type:ignore
@dataclasses.dataclass
class SyncReply:
    """Clock sync reply packet.
    See lib/network/SYNC.md."""

    org: wpistruct.int64
    """client microseconds"""
    rec: wpistruct.int64
    """server microseconds"""
    xmt: wpistruct.int64
    """server microseconds"""
