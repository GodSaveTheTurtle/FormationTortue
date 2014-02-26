import rospy
import cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

NB_ROBOTS = 1


def convertRGB2HSV(imgrgb):
    '''this function take RGB image, then convert it into HSV'''
    imghsv = cv.CreateImage(cv.GetSize(imgrgb), 8, 3)  # size, depth, channel
    cv.CvtColor(imgrgb, imghsv, cv.CV_BGR2HSV)  # Convert image from RGB to HSV
    return imghsv


def getthresholdedimg(imghsv, color):
    '''this function take HSV image.Then threshold it with yellow and blue part as
    white and all other regions as black.Then return that image'''

    # Creates images for blue and yellow (or whatever color you like).
    imgthreshold = cv.CreateImage(cv.GetSize(imghsv), 8, 1)

    # Select a range of yellow color
    cv.InRangeS(imghsv, cv.Scalar(20, 100, 100), cv.Scalar(30, 255, 255), imgthreshold)
    return imgthreshold


def find_robots(current_cv_frame):
    frame_size = cv.GetSize(current_cv_frame)

    imgDrawTresh = cv.CreateImage(frame_size, 8, 3)

    cv.NamedWindow("Real", 0)
    cv.NamedWindow("Threshold", 0)

    for i in range(NB_ROBOTS):
        imgdraw = cv.CreateImage(frame_size, 8, 3)
        cv.SetZero(imgdraw)
        cv.Flip(current_cv_frame, current_cv_frame, 1)
        cv.Smooth(current_cv_frame, current_cv_frame, cv.CV_GAUSSIAN, 3, 0)
        imghsv = convertRGB2HSV(current_cv_frame)
        imgthresh = getthresholdedimg(imghsv)
        cv.Erode(imgthresh, imgthresh, None, 3)
        cv.Dilate(imgthresh, imgthresh, None, 10)
        imgDrawTresh = cv.CloneImage(imgthresh)
        storage = cv.CreateMemStorage(0)
        contour = cv.FindContours(imgthresh, storage, cv.CV_RETR_CCOMP, cv.CV_CHAIN_APPROX_SIMPLE)

        points = []
        centroids = []

        # This is the new part here. ie Use of cv.BoundingRect()
        while contour:
            # Draw bounding rectangles
            bound_rect = cv.BoundingRect(list(contour))
            contour = contour.h_next()
            print contour
            # for more details about cv.BoundingRect,see documentation
            pt1 = (bound_rect[0], bound_rect[1])
            pt2 = (bound_rect[0] + bound_rect[2], bound_rect[1] + bound_rect[3])
            points.append(pt1)
            points.append(pt2)
            cv.Rectangle(current_cv_frame, pt1, pt2, cv.CV_RGB(255, 0, 0), 1)

            # Calculating centroids

            centroidx = cv.Round((pt1[0] + pt2[0]) / 2)
            centroidy = cv.Round((pt1[1] + pt2[1]) / 2)

            # Identifying if blue or yellow blobs and adding centroids to corresponding lists

            if (20 < cv.Get2D(imghsv, centroidy, centroidx)[0] < 30):
                centroids.append((centroidx, centroidy))

        # Now drawing part. Exceptional handling is used to avoid IndexError.
        # After drawing is over, centroid from previous part is # removed from
        # list by pop. So in next frame,centroids in this frame become initial points of line to draw.
        try:
            cv.Circle(imgdraw, centroids[1], 5, (0, 255, 255))
            cv.Line(imgdraw, centroids[0], centroids[1], (0, 255, 255), 3, 8, 0)
            centroids.pop(0)
        except IndexError:
            print "Wait for Yellow"

        cv.ShowImage("Real cv frame", current_cv_frame)
        cv.ShowImage("Threshold", imgDrawTresh)
        cv.ShowImage("final", imgdraw)


def callback_kinect(data):
    print 'kinect callback !'
    #rospy.loginfo(rospy.get_name() + ": I heard %s" % data.data)
    cv_image = bridge.imgmsg_to_cv(data.data)
    find_robots(cv_image)


def listener_kinect():
    rospy.init_node('kinect_rgb_listener', anonymous=True)
    rospy.Subscriber('/camera/rgb/image_color', Image, callback_kinect)
    rospy.spin()


if __name__ == '__main__':
    listener_kinect()
