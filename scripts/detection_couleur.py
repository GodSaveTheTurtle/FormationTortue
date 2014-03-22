import rospy
import cv
from sensor_msgs.msg import Image
from data_utils import SlaveData
from cv_bridge import CvBridge, CvBridgeError
import math

# Field of view definition
FOV_ANGLE_DEG = 57
FOV_Y_ORIGIN = 100
FOV_Y_SIZE = 100


class ColorTracking(object):

    # HSV values of colors
    hsv_colors = {
        'yellow': {'min': cv.Scalar(20, 100, 100), 'max': cv.Scalar(35, 255, 255)},
        'pink': {'min': cv.Scalar(140, 50, 50), 'max': cv.Scalar(160, 255, 255)},
        'blue': {'min': cv.Scalar(115, 50, 100), 'max': cv.Scalar(135, 255, 255)},
        'orange': {'min': cv.Scalar(18, 40, 90), 'max': cv.Scalar(27, 255, 255)},
        'green': {'min': cv.Scalar(50, 50, 50), 'max': cv.Scalar(70, 255, 255)}
    }

    def convert_rbg_to_hsv(self, imgrgb):
        '''this function takes RBG image, then convert it into HSV'''
        # CreateImage : size, depth, channel
        imghsv = cv.CreateImage(cv.GetSize(imgrgb), 8, 3)
        # Convert image from RBG to HSV
        cv.CvtColor(imgrgb, imghsv, cv.CV_BGR2HSV)
        return imghsv

    def get_thresholded_img(self, imghsv, color):
        '''this function take HSV image.Then threshold it with color part as
        white and all other regions as black.Then return that image'''

        # Create result image
        imgthreshold = cv.CreateImage(cv.GetSize(imghsv), 8, 1)

        # Select a range of color
        cv.InRangeS(imghsv, self.hsv_colors[color]['min'],
                    self.hsv_colors[color]['max'],
                    imgthreshold)
        return imgthreshold

    def angle_calculation(self, frame_width, point_x):
        '''this function return the angle theta between the middle of its FOV
        and the coordinate point_x'''
        alpha = FOV_ANGLE_DEG / 180. * math.pi
        theta = math.atan(
            (2 * math.fabs((frame_width // 2) - point_x) / frame_width)
            * math.tan(alpha / 2.0))

        # Check if theta is negative or positive
        if frame_width // 2 > point_x:
            return -theta
        else:
            return theta

    def find_depth(self, x, y):
        '''take the last depth image in memory and return the depth at x,y'''
        if not self.is_depth_initialized:
            # Have to wait for the first depth image (second check)
            depth = 'nan'
        else:
            depth = self.depth_image[y, x]
        return depth

    def print_images(self, img_real_fov, img_centroids):
        ''' print 2 imgs : the Kinect FOV and the centroids display'''
        cv.NamedWindow("FOV", 0)
        cv.NamedWindow("Centroids", 0)
        cv.ShowImage("FOV", img_real_fov)
        cv.ShowImage("Centroids", img_centroids)
        cv.WaitKey(1)

    def find_robots(self, current_cv_frame):
        '''core function, that search colors (-> slaves) on kinect image'''

        # Preprocessing cv image
        original_frame_size = cv.GetSize(current_cv_frame)
        # Take only a part of the view, to minimize noise and computation
        ROI_frame = cv.GetSubRect(current_cv_frame, (0, FOV_Y_ORIGIN,
                                                     original_frame_size[0],
                                                     FOV_Y_ORIGIN + FOV_Y_SIZE))
        ROI_frame_size = cv.GetSize(ROI_frame)
        cv.Smooth(ROI_frame, ROI_frame, cv.CV_GAUSSIAN, 3, 0)

        # Convert RGB into HSV
        img_hsv = self.convert_rbg_to_hsv(ROI_frame)

        # Centroids display creation
        img_centroids = cv.CreateImage(ROI_frame_size, 8, 3)
        cv.SetZero(img_centroids)

        # Get color to search
        for slave_color in self.slaves:
            # Treshold the image
            imgthresh = self.get_thresholded_img(img_hsv, slave_color)
            cv.Erode(imgthresh, imgthresh, None, 3)
            cv.Dilate(imgthresh, imgthresh, None, 10)
            storage = cv.CreateMemStorage(0)

            # Find all color zone
            list_contours_objets = cv.FindContours(imgthresh, storage,
                                                   cv.CV_RETR_CCOMP,
                                                   cv.CV_CHAIN_APPROX_SIMPLE)

            points = []
            while list_contours_objets:
                # Draw bounding rectangles
                bound_rect = cv.BoundingRect(list(list_contours_objets))
                list_contours_objets = list_contours_objets.h_next()
                # for more details about cv.BoundingRect,see documentation
                pt1 = (bound_rect[0], bound_rect[1])
                pt2 = (bound_rect[0] + bound_rect[
                       2], bound_rect[1] + bound_rect[3])
                points.append(pt1)
                points.append(pt2)
                cv.Rectangle(ROI_frame, pt1, pt2, cv.CV_RGB(255, 0, 0), 1)

                # Calculating centroids
                centroidx = cv.Round((pt1[0] + pt2[0]) / 2)
                centroidy = cv.Round((pt1[1] + pt2[1]) / 2)
                centroids = (centroidx, centroidy)

                # Draw centroid
                cv.Circle(img_centroids, centroids, 25, (0, 255, 255))

                # Get depth and angle of centroid
                angle = self.angle_calculation(ROI_frame_size[0], centroidx)
                depth = self.find_depth(centroidx, FOV_ANGLE_DEG + centroidy)

                # Refresh slaves data
                self.slaves[slave_color].d = depth
                self.slaves[slave_color].theta_rad = angle

                # TRACE
                print slave_color + ' posx: ' + str(centroidx) \
                    + ' angle: ' + str(angle / math.pi * 180) \
                    + ' dist ' + str(depth)

        # Display FOV and centroids
        self.print_images(ROI_frame, img_centroids)

    def kinect_depth_listener(self, msg):
        '''Callback to get the kinect depth image'''
        try:
            # The depth image is a single-channel float32 image
            self.depth_image = self.bridge.imgmsg_to_cv(msg, "32FC1")
            self.is_depth_initialized = True
        except CvBridgeError, e:
            print e

    def kinect_color_listener(self, data):
        '''Callback to get the RBG kinect image'''
        try:
            cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
            print e
        if self.is_depth_initialized is True:
            # Process image
            self.find_robots(cv_image)
        else:
            # Wait for the first depth image
            print 'Attente de recup profondeur'

    def __init__(self, slaves):

        # Init cvbridge
        self.bridge = CvBridge()

        # Init slaves
        self.slaves = slaves

        # Init depth recup
        self.is_depth_initialized = False
        self.depth_image = None
        rospy.Subscriber(
            'camera/depth/image', Image, self.kinect_depth_listener)

        # Init color tracking
        rospy.Subscriber(
            '/camera/rgb/image_color', Image, self.kinect_color_listener)


if __name__ == '__main__':
    '''main for unit testing'''

    # Init du noeud.
    rospy.init_node('kinect_color_tracking', anonymous=True)

    #Examples of slaves
    slaves = {}
    for slave_name in ['yellow', 'pink']:
        slaves[slave_name] = SlaveData()

    # Launch color tracking
    ctrack = ColorTracking(slaves)

    rospy.spin()
