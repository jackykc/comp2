#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge, numpy

global bridge, action
action = 3
# 0 1 2 3



def detect_1(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_red = numpy.array([130, 132,  110])
    upper_red = numpy.array([180, 255, 255])
    

    # lower_red = numpy.array([0, 205,  38])
    # upper_red = numpy.array([180, 255, 125])

    mask_red = cv2.inRange(hsv, lower_red, upper_red)

    ret, thresh = cv2.threshold(mask_red, 127, 255, 0)
    
    kernel = numpy.ones((9,9),numpy.float32)/25
    thresh = cv2.filter2D(thresh,-1,kernel)
    
    im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = list(filter(lambda c: c.size > 100, contours))

    cv2.drawContours(image, contours, -1, (0, 0, 255), 3)

    masked = cv2.bitwise_and(image, image, mask=mask_red)

    count = clamp_count(len(contours))
    return masked, count

def detect_2(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_red = numpy.array([130, 132,  110])
    upper_red = numpy.array([200, 256, 256])
    lower_green = numpy.array([44, 54,  63])
    upper_green = numpy.array([88, 255, 255])
    
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    ret, thresh_red = cv2.threshold(mask_red, 127, 255, 0)

    # thresh_red = mask_red
    thresh_green = mask_green # did not bother doing threshold on green

    kernel = numpy.ones((3,3),numpy.float32)/25
    thresh_red = cv2.filter2D(thresh_red,-1,kernel)
    
    _, contours_green, hierarchy = cv2.findContours(thresh_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    _, contours_red, hierarchy = cv2.findContours(thresh_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    contours_green = list(filter(lambda c: c.size > 70, contours_green))
    contours_red = list(filter(lambda c: c.size > 40, contours_red))
    
    vertices = get_vertices(contours_green)

    cv2.drawContours(image, contours_green, -1, (0,255,0), 3)
    cv2.drawContours(image, contours_red, -1, (0,0,255), 3)

    mask = cv2.bitwise_or(mask_red, mask_green)
    masked = cv2.bitwise_and(image, image, mask=mask)

    count = clamp_count(len(contours_red) + 1)
    return masked, count, get_shape_id(vertices)
    
def detect_3(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_red = numpy.array([0, 205,  38])
    upper_red = numpy.array([180, 255, 125])

    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    h, w, d = image.shape

    mask_red[:,0:w/5] = 0
    mask_red[:,4*w/5:w] = 0
    # ret, thresh_red = cv2.threshold(mask_red, 127, 255, 0)
    thresh_red = mask_red

    kernel = numpy.ones((3,3),numpy.float32)/25
    thresh_red = cv2.filter2D(thresh_red,-1,kernel)

    _, contours_red, hierarchy = cv2.findContours(thresh_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    contours_red = list(filter(lambda c: c.size > 40, contours_red))
    
    vertices = get_vertices(contours_red)

    cv2.drawContours(image, contours_red, -1, (0,0,255), 3)

    
    mask = mask_red
    masked = cv2.bitwise_and(image, image, mask=mask)

    count = clamp_count(len(contours_red))
    return masked, count, get_shape_id(vertices)
    

def clamp_count(count):
    if count < 1:
        return 1
    elif count > 3:
        return 3
    else:
        return count

def get_shape_id(vertices):
    if vertices == 3:
        id = 0
    elif vertices == 4:
        id = 1
    else:
        id = 2

    return id

def get_shape(shape_id):
    if shape_id == 0:
        shape = "triangle"
    elif shape_id == 1:
        shape = "square"
    else:
        shape = "circle"

    return shape

def get_vertices(contours):
    approx = []
    areas = [cv2.contourArea(c) for c in contours]
    if len(areas):
        max_index = numpy.argmax(areas)
        largest_contour = contours[max_index]

        peri = cv2.arcLength(largest_contour, True)
        approx = cv2.approxPolyDP(largest_contour, 0.04 * peri, True)
    return len(approx)

def image_callback(msg):
    image = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

    if (action == 1):
        image, count = detect_1(image)
        print str(count)
    elif (action == 2):
        image, count, shape_id = detect_2(image)
        print str(count) + " " + str(get_shape(shape_id))
    elif (action == 3):
        image, count, shape_id = detect_3(image)
        print str(count) + " " + str(get_shape(shape_id))

    image_pub.publish(bridge.cv2_to_imgmsg(image, encoding='bgr8'))

rospy.init_node('detector')
bridge = cv_bridge.CvBridge()
image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)
image_pub = rospy.Publisher('transformed_img', Image, queue_size=1)
rospy.spin()