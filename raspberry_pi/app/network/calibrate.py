# pylint: disable=C0114,C0115,C0116,E0401,E1101,R0902,R0903,R0913,R0917,W2301
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
