import json

from sensor_msgs.msg import Image
from std_msgs.msg import String

import flight_api as fly
import compute_vision
import campoint2globalcord as c2g

image_pub = fly.rospy.Publisher('insets_image', Image)
insets_pub = fly.rospy.Publisher('tubes', String)

insets = []

def get_distance(x1, y1, x2, y2):
    return fly.math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

def add_inset(x1, y1):
    global insets
    for x2, y2 in insets:
        if get_distance(x1, y1, x2, y2) <= 0.5:
            return None
    insets.append((x1, y1))
    insets_pub.publish(json.dumps(insets))


def callback_video(data):
    cords, cv_image = compute_vision.callback_video(data)
    for x, y in cords:
        compute_vision.cv2.circle(cv_image, (x, y), 5, (0, 0, 255), -1)
        x, y, _ = c2g.get_marker_coordinates_in_aruco_map(x, y)
        add_inset(x, y)
    image_pub.publish(compute_vision.bridge.cv2_to_imgmsg(cv_image, 'bgr8'))


fly.navigate(z=1, speed=1, frame_id='body', auto_arm=True)
fly.rospy.sleep(5)

teleme = fly.get_telemetry(frame_id='aruco_map')
startx = round(teleme.x)
starty = round(teleme.y)

image_sub = fly.rospy.Subscriber('main_camera/image_raw_throttled', Image, callback_video)

fly.navigate_wait(1, 1)
fly.navigate_wait(1, 7)
fly.navigate_wait(3, 10)

image_sub.unregister()

fly.navigate_wait(startx, starty)
fly.land()
