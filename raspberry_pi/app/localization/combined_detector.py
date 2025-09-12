""" A combined detector for AprilTags and colored objects (notes) using BGR image only."""

# pylint: disable=C0103,E0611,E1101,R0902,R0903,R0913,R0914,W0212,W0612

from typing import cast

import cv2
import numpy as np
from numpy.typing import NDArray
from robotpy_apriltag import AprilTagDetection, AprilTagDetector, AprilTagPoseEstimator
from typing_extensions import override
from wpimath.geometry import Rotation3d

from app.camera.camera_protocol import Camera, Request, Size
from app.camera.interpreter_protocol import Interpreter
from app.config.identity import Identity
from app.dashboard.display import Display
from app.network.network import Blip24, Network

Mat = NDArray[np.uint8]


class CombinedDetector(Interpreter):
    def __init__(
        self,
        identity: Identity,
        cam: Camera,
        camera_num: int,
        display: Display,
        network: Network,
        object_lower: np.ndarray,
        object_higher: np.ndarray
    ) -> None:
        """
        Parameters:
        - object_lower and object_higher: HSV bounds for object detection ([H, S, V])
          Note: hue values are 0-180, half the usual range.
        """
        self.identity = identity
        self.cam = cam
        self.camera_num = camera_num
        self.display = display
        self.network = network

        self.mtx: Mat = cam.get_intrinsic()
        self.dist: Mat = cam.get_dist()

        size: Size = cam.get_size()
        self.width: int = size.width
        self.height: int = size.height

        # AprilTag detector setup
        self.at_detector = AprilTagDetector()
        config = self.at_detector.Config()
        config.numThreads = 4
        self.at_detector.setConfig(config)
        self.at_detector.addFamily("tag36h11")

        if identity == Identity.DIST_TEST:
            tag_size = 0.033  # 33 mm for distortion rig
        else:
            tag_size = 0.1651  # normal tag size (6.5 inches)
        
        self.estimator = AprilTagPoseEstimator(
            AprilTagPoseEstimator.Config(
                tag_size,
                self.mtx[0, 0],
                self.mtx[1, 1],
                self.mtx[0, 2],
                self.mtx[1, 2],
            )
        )

        # Object detection setup
        self.object_lower = object_lower
        self.object_higher = object_higher

        # Network tables setup
        tag_path = "vision/" + identity.value + "/" + str(camera_num)
        note_path = "objectVision/" + identity.value + "/" + str(camera_num)
        self._blips = network.get_blip_sender(tag_path + "/blips")
        self._notes = network.get_note_sender(note_path + "/Rotation3d")

    def undistort_points(self, points):
        """Undistort image points using camera matrix and distortion coefficients."""
        points = np.array(points, dtype=np.float32).reshape(-1, 1, 2)
        undistorted = cv2.undistortPoints(points, self.mtx, self.dist, P=self.mtx)
        return undistorted.reshape(-1, 2)

    def detect_tags(self, img_bgr, img_display, delay_us: int) -> None:
        """Detect AprilTags in a BGR image by converting to grayscale internally."""
        # Convert BGR to grayscale for tag detection
        img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        result: list[AprilTagDetection] = self.at_detector.detect(img_gray.data)
        blips: list[Blip24] = []

        for result_item in result:
            if result_item.getHamming() > 0:
                continue

            # Undistort tag corners
            corners: tuple[float, float, float, float, float, float, float, float] = (
                result_item.getCorners((0, 0, 0, 0, 0, 0, 0, 0)))
            pairs = np.reshape(corners, [4, 2])
            pairs = cv2.undistortImagePoints(pairs, self.mtx, self.dist)
            
            # Reformat corners for pose estimation
            corners = (
                pairs[0][0][0], pairs[0][0][1],
                pairs[1][0][0], pairs[1][0][1],
                pairs[2][0][0], pairs[2][0][1],
                pairs[3][0][0], pairs[3][0][1],
            )

            homography = result_item.getHomography()
            pose = self.estimator.estimate(homography, corners)
            blips.append(Blip24(result_item.getId(), pose))
            self.display.tag(img_display, result_item, pose)  # Display on BGR image

        self._blips.send(blips, delay_us)

    def detect_objects(self, img_bgr: Mat, img_display: Mat, delay_us: int) -> None:
        """Detect colored objects in a BGR image."""
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        img_hsv = np.ascontiguousarray(img_hsv)

        # Threshold for object color
        img_range = cv2.inRange(img_hsv, self.object_lower, self.object_higher)
        
        # Clean up the mask
        floodfill = img_range.copy()
        mask = np.zeros((self.height + 2, self.width + 2), np.uint8)
        cv2.floodFill(floodfill, mask, [0, 0], [255])
        floodfill_inv = cv2.bitwise_not(floodfill)
        img_floodfill = cv2.bitwise_or(img_range, floodfill_inv)
        median = cv2.medianBlur(img_range, 5)

        # Find contours
        contours, _ = cv2.findContours(median, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        objects: list[Rotation3d] = []
        points_to_undistort = []
        contour_info = []
        
        for c in contours:
            mmnts = cv2.moments(c)
            if mmnts["m00"] < 500:  # Minimum contour area
                continue

            cX = int(mmnts["m10"] / mmnts["m00"])
            cY = int(mmnts["m01"] / mmnts["m00"])
            
            points_to_undistort.append([cX, cY])
            contour_info.append((c, cX, cY))

        if points_to_undistort:
            # Undistort all object centers at once
            undistorted_points = self.undistort_points(points_to_undistort)
            
            for (c, orig_cX, orig_cY), (undist_cX, undist_cY) in zip(contour_info, undistorted_points):
                yNormalized = (self.height / 2 - undist_cY) / self.mtx[1, 1]
                xNormalized = (self.width / 2 - undist_cX) / self.mtx[0, 0]

                initial = np.array([1, 0, 0], dtype=np.float64)
                final = np.array([1, xNormalized, yNormalized], dtype=np.float64)
                rotation = Rotation3d(initial=initial, final=final)
                
                objects.append(rotation)
                self.display.note(img_display, c, orig_cX, orig_cY)

        self._notes.send(objects, delay_us)

    @override
    def analyze(self, req: Request) -> None:
        """Process both tags and objects from the BGR image."""
        with req.rgb() as buffer_rgb:
            # Get BGR image for both detectors
            img_bgr = cast(Mat, np.frombuffer(buffer_rgb, dtype=np.uint8))
            img_bgr = img_bgr.reshape((self.height, self.width, 3))
            img_display = img_bgr.copy()
            delay_us = req.delay_us()
            
            # Run both detectors on the BGR image
            self.detect_tags(img_bgr, img_display,delay_us)
            self.detect_objects(img_bgr,img_display, delay_us)

            # Network flush and display
            self.network.flush()
            
            fps = req.fps()
            self.display.text(img_display, f"FPS {fps:2.0f}", (5, 65))
            self.display.text(img_display, f"delay (ms) {delay_us/1000:2.0f}", (5, 105))
            self.display.put(img_display)