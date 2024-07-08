import os

import cv2
import numpy as np
from scipy.ndimage import median_filter
import time
from PyQt6.QtCore import QThread, pyqtSignal, QObject, QMutex, QWaitCondition, pyqtSlot

class OpticalFlowOdometerThread(QThread):
    signal_request_processed = pyqtSignal(object)
    
    def __init__(self):
        super().__init__()
        self.wait_condition = QWaitCondition()
        self.mutex = QMutex()
        self.optical_flow_estimator = OpticalFlowOdometer()
    
    def run(self):
        while True:
            self.mutex.lock()
            self.wait_condition.wait(self.mutex)
            if self.current_msg is not None:
                result = self.optical_flow_estimator.get_odom_estimate(self.current_msg['rgb_image'], self.current_msg['depth_image'])
                self.signal_request_processed.emit(result)
                self.current_msg = None
            self.mutex.unlock()
        
    @pyqtSlot(dict)
    def handle_request(self, request_data):
        self.mutex.lock()
        self.current_msg = request_data["current_msg"]
        
        self.wait_condition.wakeAll()
        self.mutex.unlock()
        

class OpticalFlowOdometer:
    def __init__(self, intrinsic_matrix=None, scale=0.5):
        self.sift = cv2.SIFT_create()

        if intrinsic_matrix is None:
            # d435 camera intrinsics
            intrinsic_matrix = np.array([[608.389 * scale, 0.0, 428.622 * scale], [0.0, 607.683 * scale, 246.877 * scale], [0, 0, 1]])

            # azure kinect camera intrinsics (full size)
            # intrinsic_matrix = np.array([[902.98, 0.0, 956.55], [0.0, 902.77, 547.68], [0, 0, 1]])
        
        self.scale = scale
        
        self.intrinsic_matrix = intrinsic_matrix
        self.keypoints_previous = None
        self.descriptors_previous = None
        self.depth_image_previous = None

    def filter_depth_image(self, depth_img):
        return median_filter(depth_img, size=5)

    def get_matched_keypoints(self, rgb_img, depth_img):
        # get keypoints and descriptors
        keypoints_2D, descriptors = self.sift.detectAndCompute(rgb_img, None)
        
        if self.keypoints_previous is None or self.descriptors_previous is None or self.depth_image_previous is None:
            self.depth_image_previous = self.filter_depth_image(depth_img)
            self.keypoints_previous = keypoints_2D
            self.descriptors_previous = descriptors
            return None, None
        
        # match descriptors between the two images
        bf = cv2.BFMatcher()
        matches = bf.knnMatch(self.descriptors_previous, descriptors, k=2)

        # Apply Lowe's ratio test to find good matches
        good_matches = []
        for m, n in matches:
            if m.distance < 0.75 * n.distance:
                good_matches.append(m)

        # Convert 2D keypoints to 3D points using depth data
        points_previous_img_3d = []
        points_current_img_2d = []

        for m in good_matches:
            x1, y_matched_keypoint_previous = int(self.keypoints_previous[m.queryIdx].pt[0]), int(self.keypoints_previous[m.queryIdx].pt[1])
            depth_matched_keypoint_previous = self.depth_image_previous[y_matched_keypoint_previous, x1]

            if np.isclose(depth_matched_keypoint_previous, 0):
                continue

            X = (x1 - self.intrinsic_matrix[0, 2]) * depth_matched_keypoint_previous / self.intrinsic_matrix[0, 0]
            Y = (y_matched_keypoint_previous - self.intrinsic_matrix[1, 2]) * depth_matched_keypoint_previous / self.intrinsic_matrix[1, 1]
            Z = depth_matched_keypoint_previous
            points_previous_img_3d.append((X, Y, Z))

            x2, y2 = int(keypoints_2D[m.trainIdx].pt[0]), int(keypoints_2D[m.trainIdx].pt[1])
            points_current_img_2d.append((x2, y2))


        self.depth_image_previous = self.filter_depth_image(depth_img)
        self.keypoints_previous = keypoints_2D
        self.descriptors_previous = descriptors

        points_previous_img_3d = np.ascontiguousarray(points_previous_img_3d, dtype=np.float32)
        points_current_img_2d = np.ascontiguousarray(points_current_img_2d, dtype=np.float32)


        return points_previous_img_3d, points_current_img_2d

    def get_movement_estimate(self, rgb_img, depth_img):
        rgb_img = cv2.resize(rgb_img, (int(rgb_img.shape[1] * self.scale), int(rgb_img.shape[0] * self.scale)))
        depth_img = cv2.resize(depth_img, (int(depth_img.shape[1] * self.scale), int(depth_img.shape[0] * self.scale)))

        points_previous_img_3d, points_current_img_2d = self.get_matched_keypoints(rgb_img, depth_img)
        
        if points_previous_img_3d is None or points_current_img_2d is None:
            print("Need another image to estimate movement")
            return None, None

        # SolvePnP to estimate camera movement
        _, rotation_vector, translation_vector, inliers = cv2.solvePnPRansac(points_previous_img_3d,
                                                                             points_current_img_2d,
                                                                             self.intrinsic_matrix,
                                                                             None,
                                                                             iterationsCount=1000,
                                                                             reprojectionError=4.0,
                                                                             confidence=0.99,)

        if inliers is None:
            # print("No inliers found")
            return None, None

        num_outliers = len(points_previous_img_3d) - len(inliers)
        # print("Number of outliers: {}".format(num_outliers))
        # Convert rotation vector to a rotation matrix
        # rotation_matrix, _ = cv2.Rodrigues(rotation_vector)

        return translation_vector, rotation_vector
    
    def get_odom_estimate(self, rgb_img: np.ndarray, depth_img: np.ndarray):
        translation_vector, rotation_vector = self.get_movement_estimate(rgb_img, depth_img)
        
        if translation_vector is None:
            return None
        
        return translation_vector[0, 0]/1000


if __name__=="__main__":
    # # img_num_1 = "1603920744_911183729"
    # # img_num_2 = "1603920744_979714187"
    # img_num_1 = "1603920738_406265403"
    # img_num_2 = "1603920738_473242224"
    # # img_num_1 = "1603920738_406265403"
    # # img_num_2 = "1603920738_473242224"
    # file_path_img_1 = "/media/jostan/MOAD/research_data/map_making_data/jazz_apple/data_by_row/row_0/rgb/" + img_num_1 + ".png"
    # file_path_img_2 = "/media/jostan/MOAD/research_data/map_making_data/jazz_apple/data_by_row/row_0/rgb/" + img_num_2 + ".png"
    #
    # # file_path_depth_1 = "/media/jostan/portabits/map_making_data/depth_images/" + img_num_1 + ".png"
    # # file_path_depth_2 = "/media/jostan/portabits/map_making_data/depth_images/" + img_num_2 + ".png"
    # file_path_depth_1 = "/media/jostan/MOAD/research_data/map_making_data/jazz_apple/data_by_row/row_0/depth_aligned/" + img_num_1 + ".png"
    # file_path_depth_2 = "/media/jostan/MOAD/research_data/map_making_data/jazz_apple/data_by_row/row_0/depth_aligned/" + img_num_2 + ".png"

    # rgb_dir = "/media/jostan/MOAD/research_data/map_making_data/jazz_apple/data_by_row/row_0/rgb/"
    # depth_dir = "/media/jostan/MOAD/research_data/map_making_data/jazz_apple/data_by_row/row_0/depth_aligned/"
    # rgb_dir = "/media/jostan/portabits/blueberry_data/horz_scan_4-5-24/extraction_bush_3_west_2_2/color/"
    # depth_dir = "/media/jostan/portabits/blueberry_data/horz_scan_4-5-24/extraction_bush_3_west_2_2/depth_to_color/"
    rgb_dir = "/media/jostan/portabits/vo_test/rgb/"
    depth_dir = "/media/jostan/portabits/vo_test/depth/"

    image_names = os.listdir(rgb_dir)
    image_names.sort()
    depth_names = os.listdir(depth_dir)
    depth_names.sort()

    odom_data = np.load("/media/jostan/portabits/vo_test/odom_data.npy")
    odom_timestamps = np.load("/media/jostan/portabits/vo_test/odom_timestamps.npy")


    # d435 camera intrinsics
    scale = 0.5
    # intrinsic_matrix = np.array([[608.389, 0.0, 428.622], [0.0, 607.683, 246.877], [0, 0, 1]])
    intrinsic_matrix = np.array([[608.389 * scale, 0.0, 428.622 * scale], [0.0, 607.683 * scale, 246.877 * scale], [0, 0, 1]])

    # azure kinect camera intrinsics (full size)
    # scale = 1
    # intrinsic_matrix = np.array([[902.98, 0.0, 956.55], [0.0, 902.77, 547.68], [0, 0, 1]])
    # intrinsic_matrix = np.array([[902.98 * scale, 0.0, 956.55 * scale], [0.0, 902.77 * scale, 547.68 * scale], [0, 0, 1]])

    start_point = 150
    image_names = image_names[start_point:]
    depth_names = depth_names[start_point:]

    # keep every 10th image:
    skip = 1
    image_names = image_names[::skip]
    depth_names = depth_names[::skip]

    optical_flow_estimator = OpticalFlowOdometer(intrinsic_matrix=intrinsic_matrix, scale=scale)

    for image_name, depth_name in zip(image_names, depth_names):

        file_path_rgb = rgb_dir + image_name
        file_path_depth = depth_dir + depth_name

        rgb = cv2.imread(file_path_rgb)
        depth = cv2.imread(file_path_depth, cv2.IMREAD_ANYDEPTH)

        start_time = time.time()
        of_estimate = optical_flow_estimator.get_odom_estimate(rgb, depth)
        
        if of_estimate is None:
            time_stamp_part = image_name.split(".")[0]
            time_stamp = int(time_stamp_part.split("_")[0]) + int(time_stamp_part.split("_")[1]) / 1e9
            continue
        
        print("Time to estimate movement: {}".format(time.time() - start_time))
        time_stamp_previous = time_stamp
        time_stamp_part = image_name.split(".")[0]
        time_stamp = int(time_stamp_part.split("_")[0]) + int(time_stamp_part.split("_")[1]) / 1e9


        idx = np.argmin(np.abs(odom_timestamps - time_stamp))
        velocity = odom_data[idx]
        distance = velocity * (time_stamp - time_stamp_previous) * 1000
        print("Odom Estimate: {}".format(round(distance, 2)))

        # print the translation vector with two decimal places
        np.set_printoptions(precision=2, suppress=True)
        print("Optical flow estimate:\n {}".format(of_estimate))

        # cv2.imshow("RGB Image 1", rgb_1)
        cv2.imshow("RGB Image 2", rgb)
        
        #if q was pressed, break the loop
        if cv2.waitKey(0) & 0xFF == ord('q'):
            break

        # rgb_1 = rgb_2q
        # depth_1 = depth_2