
# pylint: disable=R0903

import ntcore
from app.config.identity import Identity


class Calibrate:
    """Calibration mode is controlled by a Network Table boolean."""

    def __init__(self, inst: ntcore.NetworkTableInstance, identity: Identity) -> None:
        topic = inst.getBooleanTopic("pi/" + identity.value + "/calibrate")
        self._calibrate_pub = topic.publish()
        self._calibrate_pub.set(False)
        self._calibrate = topic.subscribe(False)

    def get(self) -> bool:
        return self._calibrate.get()
