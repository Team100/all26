# pylint: disable=C0415

from platform import system

from app.camera.camera_protocol import Camera
from app.camera.size import Size
from app.config.identity import Identity
from app.dashboard.display_protocol import Display


class DisplayFactory:
    @staticmethod
    def get(
        identity: Identity,
        cam: Camera,
        name: str,
        scale: float,
    ) -> Display:
        """Select a Display implementation.

        :identity: to identify a real Raspberry Pi
        :cam: for size
        :name: must be unique
        :scale: 0.25 is good for comp, 1.0 is good for debugging (but slow)"""
        print("*** DisplayFactory for identity:", identity)
        size: Size = cam.get_size()
        display_width: int = int(scale * size.width)
        display_height = int(scale * size.height)

        # imports are inline to avoid import errors
        if system() == "Windows":
            from app.dashboard.windows_display import WindowsDisplay

            return WindowsDisplay(name, display_width, display_height)

        if identity == Identity.UNKNOWN:
            from app.dashboard.fake_display import FakeDisplay

            return FakeDisplay()

        from app.dashboard.linux_display import LinuxDisplay

        return LinuxDisplay(name, display_width, display_height)
