#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2, cv_bridge, numpy
from sensor_msgs.msg import Joy
global stop, donot_check_time, image_pub, err, cmd_vel_pub, bridge, LH, LS, LV, UH, US, UV, CURR_F, buttons
buttons = [0, 0, 0, 0]
CURR_F = 'lh'
LH = 0
LS = 0
LV = 242
UH = 170
US = 50
UV = 256
rospy.init_node('follower')

def joy_callback(msg):
    global LH, LS, LV, UH, US, UV, CURR_F, buttons
    buttons = msg.buttons
    if msg.buttons[2] == 1 and CURR_F == 'lh':
        CURR_F = 'ls'
    elif msg.buttons[2] == 1 and CURR_F == 'ls':
        CURR_F = 'lv'
    elif msg.buttons[2] == 1 and CURR_F == 'lv':
        CURR_F = 'uh'
    elif msg.buttons[2] == 1 and CURR_F == 'uh':
        CURR_F = 'us'
    elif msg.buttons[2] == 1 and CURR_F == 'us':
        CURR_F = 'uv'
    elif msg.buttons[2] == 1 and CURR_F == 'uv':
        CURR_F = 'lh'
    # print ' '
    # print 'current at: ' + CURR_F
    # print 'lower HSV are: ' + str(LH) + ', ' + str(LS) + ', ' + str(LV)
    # print 'upper HSV are: ' + str(UH) + ', ' + str(US) + ', ' + str(UV)
    # print ' '
    # if CURR_F == 'lh':
    #     if msg.buttons[0] == 1:
    #         LH -= 1
    #     elif msg.buttons[3] == 1:
    #         LH += 1
    # elif CURR_F == 'ls':
    #     if msg.buttons[0] == 1:
    #         LS -= 1
    #     elif msg.buttons[3] == 1:
    #         LS += 1
    # elif CURR_F == 'lv':
    #     if msg.buttons[0] == 1:
    #         LV -= 1
    #     elif msg.buttons[3] == 1:
    #         LV += 1
    # elif CURR_F == 'uh':
    #     if msg.buttons[0] == 1:
    #         UH -= 1
    #     elif msg.buttons[3] == 1:
    #         UH += 1
    # elif CURR_F == 'us':
    #     if msg.buttons[0] == 1:
    #         US -= 1
    #     elif msg.buttons[3] == 1:
    #         US += 1
    # elif CURR_F == 'uv':
    #     if msg.buttons[0] == 1:
    #         UV -= 1
    #     elif msg.buttons[3] == 1:
    #         UV += 1

def image_callback(msg):
    global stop, donot_check_time, image_pub, err, LH, LS, LV, UH, US, UV, buttons, CURR_F
    # if buttons[2] == 1 and CURR_F == 'lh':
    #     CURR_F = 'ls'
    # elif buttons[2] == 1 and CURR_F == 'ls':
    #     CURR_F = 'lv'
    # elif buttons[2] == 1 and CURR_F == 'lv':
    #     CURR_F = 'uh'
    # elif buttons[2] == 1 and CURR_F == 'uh':
    #     CURR_F = 'us'
    # elif buttons[2] == 1 and CURR_F == 'us':
    #     CURR_F = 'uv'
    # elif buttons[2] == 1 and CURR_F == 'uv':
    #     CURR_F = 'lh'
    print ' '
    print 'current at: ' + CURR_F
    print 'lower HSV are: ' + str(LH) + ', ' + str(LS) + ', ' + str(LV)
    print 'upper HSV are: ' + str(UH) + ', ' + str(US) + ', ' + str(UV)
    print ' '
    if CURR_F == 'lh':
        if buttons[0] == 1:
            LH -= 1
        elif buttons[3] == 1:
            LH += 1
    elif CURR_F == 'ls':
        if buttons[0] == 1:
            LS -= 1
        elif buttons[3] == 1:
            LS += 1
    elif CURR_F == 'lv':
        if buttons[0] == 1:
            LV -= 1
        elif buttons[3] == 1:
            LV += 1
    elif CURR_F == 'uh':
        if buttons[0] == 1:
            UH -= 1
        elif buttons[3] == 1:
            UH += 1
    elif CURR_F == 'us':
        if buttons[0] == 1:
            US -= 1
        elif buttons[3] == 1:
            US += 1
    elif CURR_F == 'uv':
        if buttons[0] == 1:
            UV -= 1
        elif buttons[3] == 1:
            UV += 1

    image = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_white = numpy.array([LH, LS,  LV])
    upper_white = numpy.array([UH, US, UV])
    lower_red = numpy.array([130, 132,  110])
    upper_red = numpy.array([200, 256, 256])
    
    mask = cv2.inRange(hsv, lower_white, upper_white)
    mask_red = cv2.inRange(hsv, lower_red, upper_red)

     # imgray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # blurred = cv2.pyrMeanShiftFiltering(masked, 31, 91)
    ret, thresh = cv2.threshold(mask_red, 127, 255, 0)
    
    kernel = numpy.ones((9,9),numpy.float32)/25
    thresh = cv2.filter2D(thresh,-1,kernel)
    
    im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    print 'contours number is: '
    print len(contours)
    cv2.drawContours(image, contours, -1, (0,255,0), 3)

    masked = cv2.bitwise_and(image, image, mask=mask_red)

   

    image_pub.publish(bridge.cv2_to_imgmsg(masked, encoding='bgr8'))

    
    

    # check red line
    # h, w, d = image.shape
    # search_top = h-70
    # search_bot = h-50
    # mask_red[0:search_top, 0:w] = 0
    # mask_red[search_bot:h, 0:w] = 0
    # M = cv2.moments(mask_red)
    # if M['m00'] > 0 and rospy.Time.now() > donot_check_time:
    #     cx = int(M['m10']/M['m00']) - 80
    #     cy = int(M['m01']/M['m00'])
    #     cv2.circle(image, (cx, cy), 20, (0,255,0), -1)
    #     donot_check_time = rospy.Time.now()+rospy.Duration(5)
    #     image_pub.publish(self.bridge.cv2_to_imgmsg(image, encoding='bgr8'))

    # masked = cv2.bitwise_and(image, image, mask=mask)
    # image_pub.publish(bridge.cv2_to_imgmsg(masked, encoding='bgr8'))

    # # track white line
    # h, w, d = image.shape
    # search_top = h-70
    # search_bot = h-50
    # mask[0:search_top, 0:w] = 0
    # mask[search_bot:h, 0:w] = 0
    # M = cv2.moments(mask)
    # if M['m00'] > 0:
    #     cx = int(M['m10']/M['m00']) - 80
    #     cy = int(M['m01']/M['m00'])
    #     cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
    #     err = cx - w/2
    # image_pub.publish(bridge.cv2_to_imgmsg(image, encoding='bgr8'))
rospy.Subscriber("/joy", Joy, joy_callback)
bridge = cv_bridge.CvBridge()
image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)
image_pub = rospy.Publisher('transformed_img', Image, queue_size=1)
rospy.spin()