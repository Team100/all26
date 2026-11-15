# pylint: disable=C0103,E1101,R0903,R0914
import cv2
from wpimath.geometry import Rotation3d
import numpy as np
from typing import override
from typing import Any
from cv2.typing import MatLike
from app.camera.camera_protocol import Camera, Size
from app.analysis.analysis_protocol import ColorAnalysis
from app.network.network_protocol import Network
from app.dashboard.display_util import DisplayUtil
from app.network.structs import Target
from ultralytics import YOLO 


from app.analysis.analysis_protocol import ColorAnalysis # type: ignore




class Balls(ColorAnalysis):
    """A wrapper for YOLO based target finding."""
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
            print("Model loading failed:")

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
        targets: list[Target] = []  
        if self._model is None:
            return
        results = self._model.predict(source=img, imgsz=512, conf=0.85, verbose=False)  # type: ignore
        if not results:
            return
        boxes: list[Any] = results[0].boxes.xywh.tolist()  # type: ignore
        if not boxes:
            return
        
        points_to_undistort: list[list[float]] = [] # type: ignore
        box_data: list[tuple[float, float, float]] = []

        for box in boxes: # type: ignore
            cx: float = float(box[0]) # type: ignore
            cy: float = float(box[1]) # type: ignore
            w: float = float(box[2]) # type: ignore
            h: float = float(box[3]) # type: ignore
            if img_display is not None:
                DisplayUtil.rectangle(img_display,(int(cx), int(cy)), (int(w), int(h)), (0, 255, 0))
                cv2.circle(img_display, (int(cx), int(cy)), 3, (0, 255, 0), -1)

            points_to_undistort.append([cx, cy])
            range1 = 1.0
            box_data.append((cx, cy, range1))
    
        points_arr = np.array(points_to_undistort, dtype=np.float32).reshape(-1, 1, 2)
        undistorted_points = cv2.undistortPoints(
            points_arr, self._mtx, self._dist,R=None, P=self._mtx).reshape(-1, 2)

        initial = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        
        for (cx, cy, range1), (undist_cX, undist_cY) in zip(box_data, undistorted_points):
            yNormalized: float = (self._height / 2 - undist_cY) / self._mtx[1, 1]
            xNormalized: float = (self._width / 2 - undist_cX) / self._mtx[0, 0]

            final = np.array([1.0, xNormalized, yNormalized], dtype=np.float64)
            rotation = Rotation3d(initial, final)
            
            targets.append(Target(int(servertime), rotation, range1))  # type: ignore

        
        if targets:
            self._targets.send(targets)