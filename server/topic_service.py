import json
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import rospy

bridge = CvBridge()
rospy.init_node('web_server')

img_map = cv2.imread('map.png')
mission_data = {
    'aruco_pose': None,
    'tubes': None
}

map_pub = rospy.Publisher('/web_server/map_mission', Image, queue_size=1)

def translate_coords(x_old, y_old):
    """
    Перевод реальных координат в координаты на изображении карты
    """
    x_new = 23 + 70.75 * x_old
    y_new = 25 + 70.63636363636364 * (11 - y_old)
    return int(round(x_new)), int(round(y_new))

def update_data(*args):
    global mission_data
    img = img_map.copy()

    # Траектория робота (как было)
    cv2.line(img, translate_coords(1, 1), translate_coords(1, 7), (255, 125, 125), thickness=14, lineType=cv2.LINE_AA)
    cv2.line(img, translate_coords(1, 7), translate_coords(1+1.99, 7+3.47), (255, 125, 125), thickness=14, lineType=cv2.LINE_AA)

    # Рисуем трубки + номера
    if mission_data['tubes'] is not None:
        cords = mission_data['tubes']
        for idx, tube in enumerate(cords):
            center = translate_coords(*tube)
            
            # Красный кружок
            cv2.circle(img, center, 10, (0, 0,255), -1)
            
            # Номер трубки (начиная с 1)
            text = str(idx + 1)
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.8
            color = (255, 255, 255)      # белый текст
            thickness = 2
            
            # Получаем размер текста, чтобы отцентрировать
            (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)
            text_x = center[0] - text_width // 2
            text_y = center[1] + text_height // 2
            
            # Чёрная обводка (рисуем текст несколько раз со смещением)
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx != 0 or dy != 0:
                        cv2.putText(img, text, (text_x + dx, text_y + dy),
                                    font, font_scale, (0, 0, 0), thickness, cv2.LINE_AA)
            
            # Основной белый текст
            cv2.putText(img, text, (text_x, text_y),
                        font, font_scale, color, thickness, cv2.LINE_AA)

    # Позиция ArUco (синий круг)
    if mission_data['aruco_pose'] is not None:
        x = mission_data['aruco_pose']['x']
        y = mission_data['aruco_pose']['y']
        cv2.circle(img, translate_coords(y, x), 12, (255, 0, 0), -1)

    map_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))

def aruco_pose_callback(data):
    global mission_data
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    mission_data['aruco_pose'] = {"x": x, "y": y}


def tubes_callback(data):
    global mission_data
    try:
        mission_data['tubes'] = json.loads(data.data)
    except json.JSONDecodeError:
        rospy.logwarn("Не удалось распарсить данные трубок")

def main():
    rospy.Subscriber("/tubes", String, tubes_callback)
    rospy.Subscriber("/aruco_map/pose", PoseWithCovarianceStamped, aruco_pose_callback)

    # Обновляем карту 5 раз в секунду (каждые 0.2 с)
    rospy.Timer(rospy.Duration(0.2), update_data)

    rospy.spin()