# pylint: disable=E0401,R0902

from pprint import pprint
from typing import Any
from typing import override
import numpy as np
from numpy.typing import NDArray
from picamera2 import CompletedRequest, Picamera2  # type: ignore
from app.camera.camera_protocol import Camera, Request
from app.camera.capture_timestamp import CaptureTimestamp
from app.camera.config.config_factory import ConfigFactory
from app.camera.config.config_protocol import Config
from app.camera.distortion import Distortion
from app.camera.intrinsic import Intrinsic
from app.camera.model import Model
from app.camera.real_request import RealRequest
from app.camera.size import Size
from app.config.identity import Identity
from app.decoder.decoder_protocol import Decoder
from app.util.timer import Timer


class RealCamera(Camera):
    """This is a wrapper for Picamera2.

    It handles configuration of each camera according to the Pi identity.

    For more on the Picamera2 library, see the manual:

    https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf

    and the source:

    https://github.com/raspberrypi/picamera2/
    """

    def __init__(self, identity: Identity) -> None:
        print("*** Camera: RealCamera")
        Picamera2.set_logging(Picamera2.INFO)  # type: ignore
        # debug logs with every frame (!)
        # Picamera2.set_logging(Picamera2.DEBUG)  # type: ignore

        print("*** GLOBAL CAMERA INFO")
        pprint(Picamera2.global_camera_info())  # type: ignore

        self._cam: Picamera2 = Picamera2()  # type: ignore

        print("*** SENSOR MODES AVAILABLE")
        pprint(self._cam.sensor_modes)  # type:ignore

        print("*** CAMERA CONTROLS")
        pprint(self._cam.camera_controls)  # type:ignore

        print("*** RAW MODES")
        pprint(self._cam._raw_modes)  # type:ignore

        self._mtx: Intrinsic = Intrinsic(identity)
        self._dist: Distortion = Distortion(identity)

        model: Model = Model.get(self._cam.camera_properties)  # type: ignore
        self._size: Size = Size.from_model(model)

        config: Config = ConfigFactory.get(identity, self._size)
        self._camera_config: dict[str, Any] = self._get_config(  # type: ignore
            config, identity, self._cam  # type: ignore
        )
        self._decoder: Decoder = config.decoder()
        self._timestamp: CaptureTimestamp = CaptureTimestamp(config)

        print("*** REQUESTED CONFIG")
        pprint(self._camera_config)

        # optimal alignment makes the ISP a little faster
        self._cam.align_configuration(self._camera_config, optimal=True)  # type:ignore
        print("*** ALIGNED CONFIG")
        pprint(self._camera_config)

        self._cam.configure(self._camera_config)  # type:ignore
        self._check_config(config)

        self._cam.start()  # type:ignore
        # Controls need to be set *after* the camera starts,
        # or AeEnable doesn't do anything!
        self._cam.set_controls(config.controls())  # type:ignore
        self._frame_time = Timer.time_ns()
        print("*** Camera setup complete")

    @override
    def capture_request(self) -> Request:
        capture_start: int = Timer.time_ns()
        req: CompletedRequest = self._cam.capture_request()  # type:ignore
        total_time_ms = (capture_start - self._frame_time) / 1000000
        self._frame_time = capture_start
        fps = 1000 / total_time_ms
        return RealRequest(req, fps, self._decoder, self._timestamp)  # type: ignore

    @override
    def stop(self) -> None:
        self._cam.stop()  # type: ignore
        print("*** Camera stop")

    @override
    def get_size(self) -> Size:
        return self._size

    @override
    def get_intrinsic(self) -> NDArray[np.float32]:
        return self._mtx.get()

    @override
    def get_dist(self) -> NDArray[np.float32]:
        return self._dist.get()

    def _get_config(
        self,
        conf: Config,
        identity: Identity,
        cam: Picamera2,  # type: ignore
    ) -> dict[str, Any]:
        """Produce a camera configuration dict for a single
        "main" stream.  Years ago, we attempted to use both "main"
        and "lores" so we could run both color and greyscale
        detection on the same frame without conversion, but we
        never really needed that, and it's not supported for
        UVC cameras, so it's gone.  There's just one "main"
        stream."""
        controls = conf.controls()
        print("*** REQUESTED CONTROLS:")
        pprint(controls)
        return cam.create_still_configuration(  # type:ignore
            buffer_count=conf.buffer_count(),
            queue=conf.queue(),
            sensor=conf.sensor(),
            main=conf.main(),
            raw=conf.raw(),
            transform=conf.transform(identity),  # type:ignore
            controls=controls,
        )

    def _check_config(self, config: Config):
        """Verify the requested config matches the camera config."""
        if not config.ok(self._camera_config, self._cam.camera_config):  # type:ignore
            print("*** CONFIG CHECK FAIL")
            pprint(self._camera_config)
            pprint(self._cam.camera_config)  # type:ignore
            raise ValueError("Config check fail")
