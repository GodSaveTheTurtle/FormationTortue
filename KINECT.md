DETECTION KINECT
================

## topic OPENNI utiles
* /camera/rgb/image_color
* /camera/rgb/image_raw

## conversion image OPENNI en OPENCV
cv_image = bridge.imgmsg_to_cv(image_message, desired_encoding="passthrough")

