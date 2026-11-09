# from datetime import datetime
# pylint: disable=R0903

import ntcore
from app.config.identity import Identity


class UndistortView:
    """Undistort mode is controlled by a Network Table boolean."""

    def __init__(self, inst: ntcore.NetworkTableInstance, identity: Identity) -> None:
        topic = inst.getBooleanTopic("pi/" + identity.value + "/undistort_view")
        self._undistort_view_pub = topic.publish()
        self._undistort_view_pub.set(False)
        self._undistort_view = topic.subscribe(False)

    def get(self) -> bool:
        return self._undistort_view.get()
