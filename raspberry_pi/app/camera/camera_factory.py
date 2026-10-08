# pylint: disable=C0415,R0903

from app.camera.camera_protocol import Camera
from app.config.identity import Identity


class CameraFactory:
    """Choose a camera implementation based on identity."""

    @staticmethod
    def get(identity: Identity) -> Camera:
        print("*** CameraFactory for identity:", identity)
        try:
            # ImportError if we're not running on a Raspberry Pi.
            from app.camera.real_camera import RealCamera

            # IndexError if no camera is attached.
            return RealCamera(identity)

        except ImportError:
            from app.camera.fake_camera import FakeCamera

            # These images can only be read from the filesystem,
            # not from the zip file (without more work), so they're
            # not even included in the zip file (see build.gradle)
            # 1/4 scale
            # return FakeCamera("images/tag_and_board.jpg", (1100, 620), -5)
            # full-size (huge)
            return FakeCamera("images/tag_and_board.jpg", (5504, 3096), -0.1)
            # return FakeCamera("images/blob.jpg")

        except IndexError:
            from app.camera.no_camera import NoCamera

            return NoCamera()
