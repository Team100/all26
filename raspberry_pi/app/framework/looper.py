from abc import ABC, abstractmethod
from threading import Event
import sys


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
        #    print("exiting, we crashed",file=sys.stderr, flush=True)
        #    return
            
        finally:
            self.end()
        
            self._done.set()

    @abstractmethod
    def execute(self) -> None: ...

    @abstractmethod
    def end(self) -> None: ...
