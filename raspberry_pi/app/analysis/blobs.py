# pylint: disable=C0103,E1101,R0903,R0914
from typing import override
import cv2
import numpy as np
from cv2.typing import MatLike, Moments
from numpy.typing import NDArray
from wpimath.geometry import Rotation3d
from app.camera.camera_protocol import Camera, Size
from app.dashboard.display_util import DisplayUtil
from app.analysis.analysis_protocol import ColorAnalysis
from app.network.network_protocol import Network
from app.network.structs import Target
import math

class Blobs(ColorAnalysis):
    """A wrapper for OpenCV target finding."""

    def __init__(
        self,
        cam: Camera,
        network: Network,
        object_lower: NDArray[np.int32],
        object_higher: NDArray[np.int32],
    ) -> None:
        """
        Finds blobs within a color range in HSV.
        
        Note: hue values are 0-180, half the usual range.

        :cam: camera implementation
        :network: to send results
        :object_lower: ([H, S, V]) lower bound
        :object_higher: ([H, S, V]) upper bound
        """
        print("\n*** ColorAnalysis: Blobs")
        self._mtx = cam.get_intrinsic()
        self._dist = cam.get_dist()
        size: Size = cam.get_size()
        self._width: int = size.width
        self._height: int = size.height
        self._object_lower = object_lower
        self._object_higher = object_higher
        # network output for target sightings
        self._targets = network.get_target_sender()

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

        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_hsv = np.ascontiguousarray(img_hsv)

        # Threshold for object color
        img_range = cv2.inRange(img_hsv, self._object_lower, self._object_higher)

        # sometimes it's useful to see the thresholded image
        # self.display.put(img_range)

        # Clean up the mask
        floodfill = img_range.copy()
        mask = np.zeros((self._height + 2, self._width + 2), np.uint8)
        cv2.floodFill(floodfill, mask, [0, 0], [255])
        # floodfill_inv = cv2.bitwise_not(floodfill)
        # img_floodfill = cv2.bitwise_or(img_range, floodfill_inv)
        median = cv2.medianBlur(img_range, 5)

        # Find contours
        contours, _ = cv2.findContours(median, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        targets: list[Target] = []
        # list of [x, y]
        points_to_undistort: list[list[int]] = []
        contour_info: list[tuple[MatLike, int, int, int, float]] = []

        for contour in contours:
            # https://en.wikipedia.org/wiki/Image_moment
            mmnts: Moments = cv2.moments(contour)
                     
            # reject too small (m00 is in pixels)
            if mmnts["m00"] < min_pixels:
                continue

            # reject if not round enough

            # Roundness tolerance

            roundtolo = 0.52

            perimeter = cv2.arcLength(contour, True) ** 2

            area = cv2.contourArea(contour)
            
            # In a circle, the area is c^2 over 4 pi
            fourpi = 4*3.14

            if area < roundtolo * ( perimeter / fourpi):
                continue
            k_area = 1.50E4

            cX: int = int(mmnts["m10"] / mmnts["m00"])
            cY: int = int(mmnts["m01"] / mmnts["m00"])
            cA: int =  int(area)

            # Insures that range_1 always has a value
            range_1 = 0

            if area >= min_pixels:
                range_1 = math.sqrt(k_area/area)
            
            low_y = 600

            if low_y < cY:
                continue


                    
                    

                    

            # Calculates how big the object is based on how far (high in the image) it is.
            min_area = 100
            max_area = (300000) * ((self._height / 2) / (self._height - cY + 1))
            if mmnts["m00"] < min_area or mmnts["m00"] > max_area:
                continue

            points_to_undistort.append([cX, cY])
            contour_info.append((contour, cX, cY, cA, range_1))
            
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

    def undistort_points(self, pointlist: list[list[int]]) -> MatLike:
        """Undistort image points using camera matrix and distortion coefficients.
        pointlist is list of N points, each [x,y]"""
        # shape is (N, 2)
        pointarray: NDArray[np.float32] = np.array(pointlist, dtype=np.float32)
        # points shape is (N, 1, 2)
        points: NDArray[np.float32] = pointarray.reshape((-1, 1, 2))
        # points: NDArray[np.float32] = pointarray.reshape((-1, 2))
        # undistorted shape is (N, 1, 2)
        undistorted: MatLike = cv2.undistortImagePoints(points, self._mtx, self._dist)
        return undistorted.reshape(-1, 2)
