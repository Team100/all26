# pylint: disable=C0114,C0115,C0116,E0401,R0913,R0917
from collections.abc import Buffer
from contextlib import AbstractContextManager
from typing import Any, override

from picamera2 import CompletedRequest  # type: ignore
from picamera2.request import _MappedBuffer  # type: ignore

from app.camera.capture_timestamp import CaptureTimestamp
from app.camera.request_protocol import Request
from app.decoder.decoder_protocol import Decoder

# Extra constant delay.
EXTRA_DELAY_MS: float = 2.5


class RealRequest(Request):
    def __init__(
        self,
        req: CompletedRequest,  # type: ignore
    
        fps: float,
        decoder: Decoder,
        timestamp: CaptureTimestamp,
    ):
        # Before we get a CompletedRequest, its constructor has used the
        # camera allocator sync property to:
        # * instantiate a DMA allocator sync for each buffer
        # * tell the camera allocator to mark the buffers as 'in use'
        # * __enter__() each buffer's DmaSync, which calls ioctl DMA_BUF_SYNC_START
        self._req: CompletedRequest = req 
        self._fps = fps
        self._decoder = decoder
        self._timestamp = timestamp

    @override
    def decoder(self) -> Decoder:
        return self._decoder

    @override
    def fps(self) -> float:
        # a typical value for a real camera
        return self._fps

    @override
    def timestamp_boottime_us(self) -> int:
        metadata: dict[str, Any] = self._req.get_metadata()  # type: ignore
        return self._timestamp.timestamp_boottime_us(metadata)  # type: ignore

    @override
    def buffer(self) -> AbstractContextManager[Buffer]:
        # Returns AbstractContextManager[Buffer] because the flow is:
        #
        # During picamera2.configure(), the DmaAllocator allocates
        # the requested (buffer_count) number of dma buffers, which
        # are mmap.mmap objects, which are Buffers.
        #
        # The camera uses DmaAllocator, whose sync property
        # is the DmaSync constructor.
        #
        # The _MappedBuffer constructor invokes the DmaSync constructor
        # to get a DmaSync.  _MappedBuffer.__enter__() delegates to DmaSync to
        # do the ioctl DMA_BUF_SYNC_START (again), and return the mmap.
        #
        # _MappedBuffer.__exit__() also delegates to DmaSync, which implements
        # DMA_BUF_SYNC_END.
        #
        # since _MappedBuffer implements __enter__ and __exit__, we
        # can duck-type it an AbstractContextManager.
        #
        # Note that when the _MappedBuffer is __exit__'ed, the DMA buffer should
        # not be touched anymore, which means not using numpy views on it.
        # The Picamera code addresses this in CompletedRequest.make_buffer by
        # *copying* the buffer, but we definitely don't want to do that.  Just
        # do all your work on the buffer within the scope of the context manager.
        #
        # This use of _MappedBuffer is not necessary for the buffer-reservation
        # (it's done by the CompletedRequest), but using _MappedBuffer is, by far,
        # the easiest way to get at the mmap buffer.
        #
        # To use the buffer, you can pass it to np.frombuffer().
        return _MappedBuffer(self._req, "main")  # type: ignore

    @override
    def release(self) -> None:
        # Calls DmaSync.__exit__() which invokes ioctl DMA_BUF_SYNC_END
        # to release the DMA buffer, and unmark the app-level 'in use' flag.
        # Note that the _MappedBuffer has already done the ioctl work, so this
        # is redundant.
        self._req.release()  # type: ignore
    