# pylint: disable=C0114,C0115,C0116,E0401,E1101,R0903,W2301
from abc import ABC, abstractmethod
from threading import Event


class Looper(ABC):
    """Base class for loops."""

    def __init__(self, done: Event) -> None:
        self._done = done

    def run(self) -> None:
        try:
            while True:
                if self._done.is_set():  # exit cleanly
                    return
                self.execute()
        except RuntimeError:
            pass
        #    print(f"[{datetime.now()}] " + "exiting, we crashed",file=sys.stderr, flush=True)
        #    return

        finally:
            # Cleanup this loop.
            self.end()
            # The event may be shared by multiple loops, so make sure they stop too.
            self._done.set()

    def stop(self) -> None:
        """Call to terminate the loop"""
        self._done.set()

    @abstractmethod
    def execute(self) -> None:
        """Called once every loop."""

    @abstractmethod
    def end(self) -> None:
        """Called when the loop terminates.  Use for cleanup."""
