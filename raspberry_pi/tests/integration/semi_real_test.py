import unittest
import time
from threading import Event, Thread

import numpy as np

from app.camera.camera_loop import CameraLoop
from app.camera.camera_protocol import Camera
from app.camera.fake_camera import FakeCamera
from app.config.identity import Identity
from app.dashboard.display_protocol import Display
from app.dashboard.linux_display import LinuxDisplay
from app.interpreter.interpreter_protocol import Interpreter
from app.analysis.apriltags import AprilTags
from app.analysis.blobs import Blobs
from app.interpreter.dual_interpreter import DualInterpreter
from app.network.network_protocol import Network
from app.network.real_network import RealNetwork
from app.util.timestamps import Timestamps


class SemiRealTest(unittest.TestCase):
    """Tests that actually run the whole thing for a long time,
    so you can see it working."""

    # don't run this all the time, it takes too long.
    @unittest.skip
    def test_end_to_end(self) -> None:
        """Use a fake image with a real network, and run
        for awhile, to test the listener part (in Java).
        This works just like main().
        The file it uses shows tag 1 in a position mostly yawed
        to the left.  In 2026 this tag was around 3/4 of the
        way down the field, near the left side, facing back
        towards the baseline, so the correct robot position is
        about a half meter away, facing about 45 degrees to the
        right."""
        done: Event = Event()  # to shut down all threads

        # unknown uses localhost for the server
        identity: Identity = Identity.SIM0
        camera: Camera = FakeCamera("images/tag_and_board.jpg", (1100, 620), -0.2)
        # Don't use the display at all.
        # display1: Display = FakeDisplay()
        # display2: Display = FakeDisplay()
        # Look at the undistorted image in display2.
        display1: Display = LinuxDisplay("display1", 550, 310)
        display2: Display = LinuxDisplay("display2", 1100, 620)
        network: Network = RealNetwork(identity, done)
        timestamps = Timestamps(network)
        interpreter: Interpreter = DualInterpreter(
            camera,
            display1,
            display2,
            network,
            timestamps,
            AprilTags(identity, camera, network),
            None,
        )
        camera_loop: CameraLoop = CameraLoop(camera, interpreter, done)
        thread: Thread = Thread(target=camera_loop.run)
        thread.start()
        time.sleep(60)
        done.set()
        # looper.run will return when done, so wait for that.
        thread.join()

    @unittest.skip
    def test_end_to_end_blob(self) -> None:
        """Use a fake image with a real network, and run
        for awhile, to test the listener part (in Java).
        This works just like main().
        The file it uses shows tag 1 in a position mostly yawed
        to the left.  In 2026 this tag was around 3/4 of the
        way down the field, near the left side, facing back
        towards the baseline, so the correct robot position is
        about a half meter away, facing about 45 degrees to the
        right."""
        done: Event = Event()  # to shut down all threads

        # unknown uses localhost for the server
        identity: Identity = Identity.SIM0
        camera: Camera = FakeCamera("images/blob.jpg", (1100, 620), -0.2)
        # Don't use the display at all.
        # display1: Display = FakeDisplay()
        # display2: Display = FakeDisplay()
        # Look at the undistorted image in display2.
        display1: Display = LinuxDisplay("display1", 550, 310)
        display2: Display = LinuxDisplay("display2", 1100, 620)
        network: Network = RealNetwork(identity, done)
        timestamps = Timestamps(network)

        object_lower = np.array((0, 50, 100))
        object_higher = np.array((30, 255, 255))
        interpreter: Interpreter = DualInterpreter(
            camera,
            display1,
            display2,
            network,
            timestamps,
            None,
            Blobs(camera, network, object_lower, object_higher),
        )
        camera_loop: CameraLoop = CameraLoop(camera, interpreter, done)
        thread: Thread = Thread(target=camera_loop.run)
        thread.start()
        time.sleep(60)
        done.set()
        # looper.run will return when done, so wait for that.
        thread.join()

    @unittest.skip
    def test_end_to_end_both(self) -> None:
        """Use a fake image with a real network, and run
        for awhile, to test the listener part (in Java).
        This works just like main().
        The file it uses shows tag 1 in a position mostly yawed
        to the left.  In 2026 this tag was around 3/4 of the
        way down the field, near the left side, facing back
        towards the baseline, so the correct robot position is
        about a half meter away, facing about 45 degrees to the
        right."""
        done: Event = Event()  # to shut down all threads

        # unknown uses localhost for the server
        identity: Identity = Identity.SIM0
        camera: Camera = FakeCamera("images/blob.jpg", (1100, 620), -0.2)
        # Don't use the display at all.
        # display1: Display = FakeDisplay()
        # display2: Display = FakeDisplay()
        # Look at the undistorted image in display2.
        display1: Display = LinuxDisplay("display1", 550, 310)
        display2: Display = LinuxDisplay("display2", 1100, 620)
        network: Network = RealNetwork(identity, done)
        timestamps = Timestamps(network)

        object_lower = np.array((0, 50, 100))
        object_higher = np.array((30, 255, 255))
        interpreter: Interpreter = DualInterpreter(
            camera,
            display1,
            display2,
            network,
            timestamps,
            AprilTags(identity, camera, network),
            Blobs(camera, network, object_lower, object_higher),
        )
        camera_loop: CameraLoop = CameraLoop(camera, interpreter, done)
        thread: Thread = Thread(target=camera_loop.run)
        thread.start()
        time.sleep(60)
        done.set()
        # looper.run will return when done, so wait for that.
        thread.join()
