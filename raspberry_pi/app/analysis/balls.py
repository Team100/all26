# pylint: disable=C0103,E1101,R0903,R0914
from typing import override
from cv2.typing import MatLike
from app.camera.camera_protocol import Camera, Size
from app.analysis.analysis_protocol import ColorAnalysis
from app.network.network_protocol import Network
from app.dashboard.display_util import DisplayUtil
from ultralytics import YOLO


from app.analysis.analysis_protocol import ColorAnalysis # type: ignore

        


class Balls(ColorAnalysis):
    """A wrapper for OpenCV target finding."""

    def __init__(
        self,
        cam: Camera,
        network: Network,
    ) -> None:
        """
        Finds yellow balls 

        :cam: camera implementation
        :network: to send results
        
        """
        print("\n*** ColorAnalysis: Blobs")
        self._mtx = cam.get_intrinsic()
        self._dist = cam.get_dist()
        size: Size = cam.get_size()
        self._width: int = size.width
        self._height: int = size.height
        # network output for target sightings
        self._targets = network.get_target_sender()
        try:
            self._model = YOLO("/home/pi/models/ball-detection_ncnn_model", task="detect")
        except:
            self._model = None
            print("Model loading failed")

    @override
    def analyze_color(
        self,
        img: MatLike,
        img_display: MatLike | None,
        servertime: int,
        min_pixels: int,
    ) -> None:
        """Find things in img_bgr

        :img: 24-bit color, distorted, for detection
        :img_display: distorted, for display, annotated.
        :servertime: drift-corrected server-time microsecond timestamp"""
        # DisplayUtil.rectangle(img_display, (200, 200), (50, 50), (0, 255, 0))
        if self._model is not None:
            result = self._model.predict(source=img, imgsz=512, conf=0.5, verbose=False)[0] # type: ignore
            for(cx, cy, w, h) in result.boxes.xywh.tolist(): # type: ignore
                if img_display is not None:
                    DisplayUtil.rectangle(img_display, (int(cx), int(cy)), (int(w), int(h)), (0, 255, 0)) # type: ignore
#         result = self._model.predict(source=img, conf=0.25, verbose=False)[0]

#         names.getattr(self._model,"names",{}) or {}
#         for (cx, cy, w, h), score, cls in zip(
#         result.boxes.xywhn.tolist(), 
#         result.boxes.conf.tolist(), 
#         result.boxes.cls.tolist()
# ):
#     box_info = {
#         "cls": int(cls),
#         "name": str(names.get(int(cls), int(cls))),
#         "conf": round(float(score), 4),
#         "cx": cx,
#         "cy": cy,
#         "w": w,
#         "h": h,
#     }
#     boxes.append(box_info)