# pylint: disable=R0903

from typing import Any, override
import collections.abc
import typing
import numpy as np

from napa_vision import (
    NapaDetector,
    VisionProcessor,
    PoseWithCovariance,
    FiducialImageObservation,
)

from robotpy_apriltag import AprilTagDetection, AprilTagDetector, AprilTagPoseEstimator


class FiducialImageObservationWrapper:
    """
    A wrapper for `napa_vision`'s `FiducialImageObservation`, allowing `robotpy_apriltag`'s API.
    """

    def __init__(self, sup: FiducialImageObservation):

        self._detection: FiducialImageObservation = sup
        self._center: AprilTagDetection.Point = AprilTagDetection.Point(
            self._detection.center[0] + 0.5, self._detection.center[1] + 0.5
        )
        self._corners: Any = (
            (self._detection.corners[[1, 0, 3, 2]] + 0.5).flatten().tolist()
        )

        H = np.array(self._detection.homography, dtype=np.float64, copy=True)
        H[:2] += 0.5 * H[2]

        self._homography: Any = (
            H.flatten().tolist()
        )  # self._detection.homography.flatten().tolist()
        self._homographyMatrix: Any = (
            H  # np.asarray(self._detection.homography, dtype=np.float64)
        )

    def getCenter(self) -> AprilTagDetection.Point:
        """
        Gets the center of the detection in image pixel coordinates.

        :returns: Center point
        """

        return self._center

    def getCorner(
        self, ndx: typing.SupportsInt | typing.SupportsIndex
    ) -> AprilTagDetection.Point:
        """
        Gets a corner of the tag in image pixel coordinates. These always
        wrap counter-clock wise around the tag. Index 0 is the bottom left corner.

        :param ndx: Corner index (range is 0-3, inclusive)

        :returns: Corner point
        """
        return AprilTagDetection.Point(self._corners[ndx][0],self._corners[ndx][1])

    def getCorners(
        self,
        cornersBuf: tuple[
            typing.SupportsFloat | typing.SupportsIndex,
            typing.SupportsFloat | typing.SupportsIndex,
            typing.SupportsFloat | typing.SupportsIndex,
            typing.SupportsFloat | typing.SupportsIndex,
            typing.SupportsFloat | typing.SupportsIndex,
            typing.SupportsFloat | typing.SupportsIndex,
            typing.SupportsFloat | typing.SupportsIndex,
            typing.SupportsFloat | typing.SupportsIndex,
        ],
    ) -> tuple[float, float, float, float, float, float, float, float]:
        """
        Gets the corners of the tag in image pixel coordinates. These always
        wrap counter-clock wise around the tag. The first set of corner coordinates
        are the coordinates for the bottom left corner.

        :param cornersBuf: Corner point array (X and Y for each corner in order)

        :returns: Corner point array (copy of cornersBuf span)
        """

        cornersBuf = tuple(self._corners)
        return cornersBuf


    def getDecisionMargin(self) -> float:
        """
        Gets a measure of the quality of the binary decoding process: the
        average difference between the intensity of a data bit versus
        the decision threshold. Higher numbers roughly indicate better
        decodes. This is a reasonable measure of detection accuracy
        only for very small tags-- not effective for larger tags (where
        we could have sampled anywhere within a bit cell and still
        gotten a good detection.)

        :returns: Decision margin
        """
        return self._detection.decision_margin

    def getFamily(self) -> str:
        """
        Gets the decoded tag's family name.

        :returns: Decoded family name
        """
        return self._detection.family

    def getHamming(self) -> int:
        """
        Gets how many error bits were corrected. Note: accepting large numbers of
        corrected errors leads to greatly increased false positive rates.
        NOTE: As of this implementation, the detector cannot detect tags with
        a hamming distance greater than 2.

        :returns: Hamming distance (number of corrected error bits)
        """
        return self._detection.hamming

    def getHomography(
        self,
    ) -> tuple[float, float, float, float, float, float, float, float, float]:
        """
        Gets the 3x3 homography matrix describing the projection from an
        "ideal" tag (with corners at (-1,1), (1,1), (1,-1), and (-1,
        -1)) to pixels in the image.

        :returns: Homography matrix data
        """
        return self._homography

    def getHomographyMatrix(
        self,
    ) -> typing.Annotated[np.typing.NDArray[np.float64], "[3, 3]"]:
        """
        Gets the 3x3 homography matrix describing the projection from an
        "ideal" tag (with corners at (-1,1), (1,1), (1,-1), and (-1,
        -1)) to pixels in the image.

        :returns: Homography matrix
        """
        return self._homographyMatrix

    def getId(self) -> int:
        """
        Gets the decoded ID of the tag.

        :returns: Decoded ID
        """
        return self._detection.tag_id


class NapaDetectorWrapper:
    """
    A wrapper for `napa_vision`'s `NapaDetector`, allowing `robotpy_apriltag`'s API.
    """

    Config = AprilTagDetector.Config
    QuadThresholdParameters = AprilTagDetector.QuadThresholdParameters

    def __init__(self):

        self._detector: NapaDetector = NapaDetector()
        self._config: AprilTagDetector.Config | None = None
        self._quadThresholdParameters: (
            AprilTagDetector.QuadThresholdParameters | None
        ) = None

    def addFamily(
        self, fam: str, bitsCorrected: typing.SupportsInt | typing.SupportsIndex = 2
    ) -> bool:
        """
        Adds a family of tags to be detected.

        :param fam:           Family name, e.g. "tag16h5"
        :param bitsCorrected: Maximum number of bits to correct

        :returns: False if family can't be found
        """
        return True  # TODO

    def clearFamilies(self) -> None:
        """
        Unregister all families.
        """
        return  # TODO

    def close(self) -> None:
        """
        Frees all resources associated with this detector
        """
        self._detector.close()

    def detect(
        self, image: collections.abc.Buffer
    ) -> list[FiducialImageObservationWrapper]:
        """
        Detect tags from an 8-bit grayscale image with shape (height, width)

        :return: list of results
        """

        detections: list[FiducialImageObservation] = self._detector.detect(image)
        return [
            FiducialImageObservationWrapper(i) for i in detections
        ]  # TODO: surely there's a more elegant way to do this? (most likely does not matter)

    def getConfig(self) -> AprilTagDetector.Config | None:
        """
        Gets detector configuration.

        :returns: Configuration
        """
        return self._config

    def getQuadThresholdParameters(
        self,
    ) -> AprilTagDetector.QuadThresholdParameters | None:
        """
        Gets quad threshold parameters.

        :returns: Parameters
        """
        return self._quadThresholdParameters

    def removeFamily(self, fam: str) -> None:
        """
        Removes a family of tags from the detector.

        :param fam: Family name, e.g. "tag16h5"
        """
        return  # TODO

    def setConfig(self, config: AprilTagDetector.Config) -> None:
        """
        Sets detector configuration.

        :param config: Configuration
        """
        self._config = config  # TODO: find a way to implement configs

    def setQuadThresholdParameters(
        self, params: AprilTagDetector.QuadThresholdParameters
    ) -> None:
        """
        Sets quad threshold parameters.

        :param params: Parameters
        """
        self._quadThresholdParameters = params  # TODO: find a way to implement configs
