# pylint: disable=C0114,C0115,C0116,C0415,E0401,R0903,R0911,W0707,W0719
from datetime import datetime
from sys import stderr

from app.camera.camera_protocol import Camera
from app.config.identity import Identity


class CameraFactory:
    """Choose a camera implementation based on identity."""

    @staticmethod
    def get(identity: Identity) -> Camera:
        print(f"[{datetime.now()}] " + "*** CameraFactory for identity:", identity)
        try:
            # ImportError if we're not running on a Raspberry Pi.
            from app.camera.real_camera import RealCamera

            # IndexError if no camera is attached.
            print(
                f"[{datetime.now()}] " + "***real camera started",
                file=stderr,
                flush=True,
            )
            return RealCamera(identity)

        except ImportError:
            from app.camera.fake_camera import FakeCamera

            print(f"[{datetime.now()}] " + "*** Fake Camera", file=stderr, flush=True)

            # These images can only be read from the filesystem,
            # not from the zip file (without more work), so they're
            # not even included in the zip file (see build.gradle)
            # 1/4 scale
            # return FakeCamera("images/tag_and_board.jpg", (1100, 620), -5)
            # full-size (huge)
            # return FakeCamera("images/tag_and_board.jpg", (5504, 3096), -0.1)
            return FakeCamera("images/tag_and_board.jpg", (5504, 3096), 0)
            # return FakeCamera("images/blob.jpg")

        except IndexError:
            # from app.camera.no_camera import NoCamera
            raise Exception("we mad the cameras tak tom's advice :)")
            # return NoCamera()
