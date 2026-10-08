"""This is the coprocessor main loop.

Each task is run by its own Looper, in its own thread.

You can't run this from the command line.  To run the app,
use the script called "runapp.py" in the raspberry_pi directory
(one level above this one).
"""

# pylint: disable=R0914

import time
from threading import Event, Thread

from app.camera.camera_factory import CameraFactory
from app.camera.camera_protocol import Camera
from app.interpreter.interpreter_factory import InterpreterFactory
from app.interpreter.interpreter_protocol import Interpreter
from app.camera.camera_loop import CameraLoop
from app.config.identity import Identity
from app.dashboard.display_protocol import Display
from app.dashboard.display_factory import DisplayFactory
from app.network.network_protocol import Network
from app.network.real_network import RealNetwork
from app.util.banner import print_banner
from app.util.timestamps import Timestamps

def main() -> None:
    print_banner()
    print("*** Starting main.py")
    identity: Identity = Identity.get()
    done: Event = Event()  # to shut down all threads
    thread: Thread | None = None
    try:
        camera: Camera = CameraFactory.get(identity)
        # main display for annotated images
        display1: Display = DisplayFactory.get(identity, camera, "display1", 0.25)
        # secondary display
        display2: Display = DisplayFactory.get(identity, camera, "display2", 1.0)
        network: Network = RealNetwork(identity, done)
        timestamps = Timestamps(network)
        interpreter: Interpreter = InterpreterFactory.get(
            identity, camera, display1, display2, network, timestamps
        )
        camera_loop: CameraLoop = CameraLoop(camera, interpreter, done)
        thread = Thread(target=camera_loop.run)
        print("*** Starting the looper thread")
        thread.start()
        # looper.run will return when done, so wait for that.
        thread.join()
        print("*** Looper has exited")

    finally:
        done.set()  # exit all threads cleanly
        # if the loop is hung, we don't want to wait for it,
        # so just wait a little bit of time instead.
        time.sleep(1)
