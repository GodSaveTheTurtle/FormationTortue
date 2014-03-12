import rospy
import cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import math

NB_ROBOTS = 4
FOV_ANGLE_DEG = 57
FOV_Y_ORIGIN = 100
FOV_Y_SIZE = 100


class ColorTracking():
    hsv_colors = {
        'yellow': {'min': cv.Scalar(20, 100, 100), 'max': cv.Scalar(35, 255, 255)},
        'pink': {'min': cv.Scalar(140, 50, 50), 'max': cv.Scalar(160, 255, 255)},
        'blue': {'min': cv.Scalar(115, 50, 100), 'max': cv.Scalar(135, 255, 255)},
        'orange': {'min': cv.Scalar(18, 40, 90), 'max': cv.Scalar(27, 255, 255)},
        'green': {'min': cv.Scalar(50, 50, 50), 'max': cv.Scalar(70, 255, 255)}
    }

    param_robots = [
        {'color': 'pink', 'pos_x': 0},
        {'color': 'blue', 'pos_x': 0},
        {'color': 'green', 'pos_x': 0},
        {'color': 'yellow', 'pos_x': 0}
    ]

    def convert_rbg_to_hsv(self, imgrgb):
        '''this function take RGB image, then convert it into HSV'''
        imghsv = cv.CreateImage(cv.GetSize(imgrgb), 8, 3)  # size, depth, channel
        cv.CvtColor(imgrgb, imghsv, cv.CV_BGR2HSV)  # Convert image from RGB to HSV
        return imghsv

    def get_thresholded_img(self, imghsv, color):
        '''this function take HSV image.Then threshold it with color part as
        white and all other regions as black.Then return that image'''

        # Creates images
        imgthreshold = cv.CreateImage(cv.GetSize(imghsv), 8, 1)

        # Select a range of color
        cv.InRangeS(imghsv, self.hsv_colors[color]['min'],
                    self.hsv_colors[color]['max'],
                    imgthreshold)
        return imgthreshold

    def angle_calculation(self, frame_width, point_x):
        alpha = FOV_ANGLE_DEG / 180. * math.PI
        theta = math.atan((2 * math.fabs((frame_width // 2) - point_x) / frame_width) * math.tan(alpha / 2.0))
        if frame_width // 2 > point_x:
            return -theta
        else:
            return theta

    def find_depth(self, x, y):
        if not self.is_depth_initialized:
            depth = 'nan'
        else:
            depth = self.depth_image[y, x]
        return depth

    def print_images(self, img_real_fov, img_centroids):
        cv.NamedWindow("FOV", 0)
        cv.NamedWindow("Centroids", 0)
        cv.ShowImage("FOV", img_real_fov)
        cv.ShowImage("Centroids", img_centroids)
        cv.WaitKey(1)

    def find_robots(self, current_cv_frame):
        # Preprocessing cv image
        original_frame_size = cv.GetSize(current_cv_frame)
        ROI_frame = cv.GetSubRect(current_cv_frame, (0, FOV_Y_ORIGIN,
                                                     original_frame_size[0], FOV_Y_ORIGIN + FOV_Y_SIZE))
        ROI_frame_size = cv.GetSize(ROI_frame)
        cv.Smooth(ROI_frame, ROI_frame, cv.CV_GAUSSIAN, 3, 0)
        img_hsv = self.convert_rbg_to_hsv(ROI_frame)

        # Centroids display creation
        img_centroids = cv.CreateImage(ROI_frame_size, 8, 3)
        cv.SetZero(img_centroids)

        for i in range(NB_ROBOTS):
            imgthresh = self.get_thresholded_img(img_hsv, self.param_robots[i]['color'])
            cv.Erode(imgthresh, imgthresh, None, 3)
            cv.Dilate(imgthresh, imgthresh, None, 10)
            storage = cv.CreateMemStorage(0)
            list_contours_objets = cv.FindContours(imgthresh, storage, cv.CV_RETR_CCOMP, cv.CV_CHAIN_APPROX_SIMPLE)

            points = []
            while list_contours_objets:
                # Draw bounding rectangles
                bound_rect = cv.BoundingRect(list(list_contours_objets))
                list_contours_objets = list_contours_objets.h_next()
                # for more details about cv.BoundingRect,see documentation
                pt1 = (bound_rect[0], bound_rect[1])
                pt2 = (bound_rect[0] + bound_rect[2], bound_rect[1] + bound_rect[3])
                points.append(pt1)
                points.append(pt2)
                cv.Rectangle(ROI_frame, pt1, pt2, cv.CV_RGB(255, 0, 0), 1)

                # Calculating centroids

                centroidx = cv.Round((pt1[0] + pt2[0]) / 2)
                centroidy = cv.Round((pt1[1] + pt2[1]) / 2)
                centroids = (centroidx, centroidy)

                cv.Circle(img_centroids, centroids, 25, (0, 255, 255))
                print 'couleur ' + self.param_robots[i]['color'] + ' posx : ' + str(centroidx)
        self.print_images(ROI_frame, img_centroids)

    def kinect_depth_listener(self, msg):
        try:
            # The depth image is a single-channel float32 image
            self.depth_image = self.bridge.imgmsg_to_cv(msg, "32FC1")
            self.is_depth_initialized = True
        except CvBridgeError, e:
            print e

    def kinect_color_listener(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
            print e
        if self.is_depth_initialized is True:
            self.find_robots(cv_image)
        else:
            print 'Attente de recup profondeur'

    def __init__(self):
        #Init cvbridge
        self.bridge = CvBridge()
        #Init depth recup
        self.is_depth_initialized = False
        self.depth_image = None
        rospy.init_node('kinect_depth_listener', anonymous=True)
        rospy.Subscriber('camera/depth/image', Image, self.kinect_depth_listener)

        #Init color tracking
        rospy.init_node('kinect_color_listener', anonymous=True)
        rospy.Subscriber('/camera/rgb/image_color', Image, self.kinect_color_listener)
        rospy.spin()

if __name__ == '__main__':
    ctrack = ColorTracking()
