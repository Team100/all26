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
        targets: list[Target] = []
        # list of [x, y]
        points_to_undistort: list[list[int]] = []
        points_to_undistort.append([cX, cY])
        if self._model is not None:
            result = self._model.predict(source=img, imgsz=512, conf=0.85, verbose=False)[0] # type: ignore
            for(cx, cy, w, h) in result.boxes.xywh.tolist(): # type: ignore
                if img_display is not None:
                    DisplayUtil.rectangle(img_display, (int(cx), int(cy)), (int(w), int(h)), (0, 255, 0))
        if points_to_undistort:
            # Undistort all points at once
            undistorted_points = self.undistort_points(points_to_undistort)

            c: MatLike
            orig_cX: int
            orig_cY: int
            undist_cX: float
            undist_cY: float
            for (c, orig_cX, orig_cY, cA, range1), (undist_cX, undist_cY) in zip(
                contour_info, undistorted_points
            ):

                yNormalized: float = (self._height / 2 - undist_cY) / self._mtx[1, 1]
                xNormalized: float = (self._width / 2 - undist_cX) / self._mtx[0, 0]

                initial = np.array([1, 0, 0], dtype=np.float64)
                final = np.array([1, xNormalized, yNormalized], dtype=np.float64)
                rotation = Rotation3d(initial=initial, final=final)
                
                targets.append(Target(int(servertime), rotation, range1))


                if img_display is not None:
                    DisplayUtil.note(img_display, c, orig_cX, orig_cY)

            

            # only send if there's anything to say
            self._targets.send(targets)