import rospy
import cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

NB_ROBOTS = 2

hsv_colors = {
    'yellow': {'min': cv.Scalar(20, 100, 100), 'max': cv.Scalar(40, 255, 255)},
    'pink': {'min': cv.Scalar(150, 50, 50), 'max': cv.Scalar(175, 255, 255)},
    'red': {'min': cv.Scalar(0, 100, 100), 'max': cv.Scalar(15, 255, 255)},
    'green': {'min': cv.Scalar(55, 50, 50), 'max': cv.Scalar(65, 255, 255)}
}

param_robots = [
    {'color': 'yellow', 'pos_x': 0},
    {'color': 'red', 'pos_x': 0}
]


def convertRGB2HSV(imgrgb):
    '''this function take RGB image, then convert it into HSV'''
    imghsv = cv.CreateImage(cv.GetSize(imgrgb), 8, 3)  # size, depth, channel
    cv.CvtColor(imgrgb, imghsv, cv.CV_BGR2HSV)  # Convert image from RGB to HSV
    return imghsv


def getthresholdedimg(imghsv, color):
    '''this function take HSV image.Then threshold it with color part as
    white and all other regions as black.Then return that image'''

    # Creates images
    imgthreshold = cv.CreateImage(cv.GetSize(imghsv), 8, 1)

    # Select a range of color
    cv.InRangeS(imghsv, hsv_colors[color]['min'],
                        hsv_colors[color]['max'],
                        imgthreshold)
    return imgthreshold


def find_robots(current_cv_frame):
    frame_size = cv.GetSize(current_cv_frame)

    cv.NamedWindow("Real", 0)
    cv.NamedWindow("Final", 0)

    for i in range(NB_ROBOTS):
        imgdraw = cv.CreateImage(frame_size, 8, 3)
        cv.SetZero(imgdraw)
        cv.Flip(current_cv_frame, current_cv_frame, 1)
        cv.Smooth(current_cv_frame, current_cv_frame, cv.CV_GAUSSIAN, 3, 0)

        imghsv = convertRGB2HSV(current_cv_frame)
        imgthresh = getthresholdedimg(imghsv, param_robots[i]['color'])

        cv.Erode(imgthresh, imgthresh, None, 3)
        cv.Dilate(imgthresh, imgthresh, None, 10)
        storage = cv.CreateMemStorage(0)
        contour = cv.FindContours(imgthresh, storage, cv.CV_RETR_CCOMP, cv.CV_CHAIN_APPROX_SIMPLE)

        points = []
        centroids = []

        while contour:
            # Draw bounding rectangles
            bound_rect = cv.BoundingRect(list(contour))
            contour = contour.h_next()
            # for more details about cv.BoundingRect,see documentation
            pt1 = (bound_rect[0], bound_rect[1])
            pt2 = (bound_rect[0] + bound_rect[2], bound_rect[1] + bound_rect[3])
            points.append(pt1)
            points.append(pt2)
            cv.Rectangle(current_cv_frame, pt1, pt2, cv.CV_RGB(255, 0, 0), 1)

            # Calculating centroids

            centroidx = cv.Round((pt1[0] + pt2[0]) / 2)
            centroidy = cv.Round((pt1[1] + pt2[1]) / 2)
            centroids.append((centroidx, centroidy))

            cv.Circle(imgdraw, centroids[0], 25, (0, 255, 255))
            print 'couleur '+param_robots[i]['color']+' posx : '+centroidx
            centroids.pop(0)

    cv.ShowImage("Real", current_cv_frame)
    cv.ShowImage("Final", imgdraw)
    cv.WaitKey(1)


def callback_kinect(data):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv(data, "bgr8")
    except CvBridgeError, e:
        print e
    find_robots(cv_image)


def listener_kinect():
    rospy.init_node('kinect_rgb_listener', anonymous=True)
    rospy.Subscriber('/camera/rgb/image_color', Image, callback_kinect)
    rospy.spin()


if __name__ == '__main__':
    listener_kinect()
