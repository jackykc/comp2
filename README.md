# Competition 2: Run Robot, Run

## Purpose
In this competition we demonstrate how a turtlebot can complete a course filled with various tasks. Computer vision is used both to stay on the course as well as detecting and matching simple objects to complete each task. 

## Prerequisites
* Kobuki Turtlebot with an Asus Xtion Pro
* Ubuntu 16.04
* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) (Desktop or Desktop-Full)
* [Turtlebot](http://wiki.ros.org/action/show/Robots/TurtleBot), [Kobuki Packages](http://wiki.ros.org/kobuki) and [camera](http://wiki.ros.org/openni_camera)
  ```
  sudo apt-get install ros-kinetic-turtlebot
  sudo apt-get install ros-kinetic-kobuki
  sudo apt-get install ros-kinetic-kobuki-core
  sudo apt-get install ros-kinetic-openni2-camera
  sudo apt-get install ros-kinetic-openni2-launch
  ```

## Resources used
HSV Thresholding
https://docs.opencv.org/3.1.0/df/d9d/tutorial_py_colorspaces.html

Contour Detection
https://docs.opencv.org/3.1.0/dd/d49/tutorial_py_contour_features.html

## Execution:
1. Build and source setup.bash
   ```
   (In catkin_ws directory after cloning repo into catkin_ws/src)
   catkin_make
   source ./devel/setup.bash
   ```
1. Connect to the Kobuki and Asus Xtion Pro
1. Place Kobuki on race track
1. Launch the competition node `roslaunch comp2 comp2.launch`

## Concepts and code

* State Machine
![alt text](https://raw.githubusercontent.com/jackykc/comp2/master/state_machine.png)
```
1. In the GO state, the turtlebot does line tracking.
2. Upon seeing a red line, we move to a stop state, incrementing 
   the number of stops it has seen
3. After stopping, it will then move onto one of the three tasks
   or the finish state after the last red line has been seen
```
* GO (Line Tracking)
  * Convert each frame into HSV format.
  * Use opencv’s inRange function to filter out the desired color.
  * Crop out the frame’s most top part.
  * Use opencv’s moments function to to find the centroid of color in each frame. 
  * Calculate the distance between color regin and the center of the frame which is the error that the turtlebot is off from     the course.
  * Use P control to convert the error to how much the turtlebot needs to turn.
* Task 1
  * Threshold to keep only the red pixels of the image
  * Blur threshold image
  * Find and count contours larger than a specified minimum size from blurred image
* Task 2
  * Create two thresholded images, one for red and one for green
  * Find contours bigger than a specified minimum size for both images
  * Count the returned contours
  * Get the shape of the green contour using the number of sides of the contour
* Task 3
  * For each of the stop lines in task three:
    * Threshold to keep only the red pixels of the image
    * Blur thresholded image
    * Get the shape of the largest red contour using the number of sides
    * Match the shape with the one obtained from task two

## Code explanations
[Thresholding of images by color](https://github.com/jackykc/comp2/blob/master/src/follower.py#L178)

function used for task three, but thresholding for task one and two is the same
``` python
def detect_3(image):
    # keep only red
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_red = numpy.array([0, 205,  38])
    upper_red = numpy.array([180, 255, 125])

    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    # curtain off left and right of the image for task three
    h, w, d = image.shape
    mask_red[:,0:w/5] = 0
    mask_red[:,4*w/5:w] = 0
```
[Get the number of sides on the shape](https://github.com/jackykc/comp2/blob/master/src/follower.py#L241)

function used for both task two and task three for finding the shape
``` python
# this is incorrectly named vertices, when it should be sides instead 
def get_vertices(contours):
    approx = []
    areas = [cv2.contourArea(c) for c in contours]
    if len(areas):
        # only care for the largest contour
        max_index = numpy.argmax(areas)
        largest_contour = contours[max_index]
        # approximate the contour shape to remove noisy vertices
        peri = cv2.arcLength(largest_contour, True)
        approx = cv2.approxPolyDP(largest_contour, 0.04 * peri, True)
    return len(approx) # return the sides of the approximated contour
```
Video:
https://www.youtube.com/watch?v=mmgdHccBFf8

