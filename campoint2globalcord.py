import rospy
import numpy as np
from sensor_msgs.msg import CameraInfo
from clover import srv
import tf2_ros
from scipy.spatial.transform import Rotation # pip install 

# rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)

tf_buffer = tf2_ros.Buffer(rospy.Duration(10))  # кэш на 10 секунд
tf_listener = tf2_ros.TransformListener(tf_buffer)

def get_cam_param():
    '''
    import rospy
    from sensor_msgs.msg import CameraInfo

    D, K, R, P = get_cam_param()
    '''
    msg = rospy.wait_for_message('/main_camera/camera_info', CameraInfo, timeout=3)
    return (msg.D, msg.K, msg.R, msg.P)

# def get_pos():
#     t = get_telemetry()
#     return (t.x, t.y, t.z)

def get_tranfosrm():
    while not rospy.is_shutdown():
        try:
            trans = tf_buffer.lookup_transform(target_frame="aruco_map", source_frame="main_camera_optical", time=rospy.Time(0), timeout=rospy.Duration(1.0))
            return trans.transform
        except:
            rospy.sleep(0.3)

def get_marker_coordinates_in_aruco_map(point_x, point_y):
    '''
    в качестве аргумента передаются координаты точки на картинке
    возвращаются x y z в глобальной системе (aruco_map)
    '''
    # 1. Получаем параметры камеры
    D, K, R, P = get_cam_param()
    K = np.array(K).reshape(3, 3)
    fx, fy = K[0, 0], K[1, 1] # Фокусные расстояния
    cx, cy = K[0, 2], K[1, 2] # Центр оптической оси

    # 2. получаем положение камеры
    transform = get_tranfosrm()
    pos_camera = np.array((
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    ))
    quat = [
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w,
    ]
    rot_matrix = Rotation.from_quat(quat).as_matrix()

    pixel_u = point_x
    pixel_v = point_y
    
    # 3. Переводим пиксель в вектор направления (в системе камеры)
    vector_cam = np.array([
        (pixel_u - cx) / fx,
        (pixel_v - cy) / fy,
        1.0
    ])

    # 4. Поворачиваем этот вектор так же, как повернут дрон (в глобальную систему)
    vector_global = rot_matrix @ vector_cam

    # 5. Ищем, где этот луч коснется пола (где Z = 0)
    # Нам нужно узнать, на сколько умножить вектор, чтобы с высоты дрона дойти до 0
    scale = -pos_camera[2] / vector_global[2]
    
    # Итоговая точка = Позиция_Дрона + Длина_Луча * Направление
    result_point = pos_camera + scale * vector_global

    return (result_point[0], result_point[1], result_point[2]) # x y z



# import rospy
# import numpy as np
# import cv2
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image
# from std_msgs.msg import Header

# # Глобальные паблишеры (создаём один раз)
# pub_original = rospy.Publisher('/blue_detection/original', Image, queue_size=1)
# pub_mask     = rospy.Publisher('/blue_detection/mask',     Image, queue_size=1)
# pub_result   = rospy.Publisher('/blue_detection/result',   Image, queue_size=1)

# bridge = CvBridge()

# # === НАСТРОЙКИ ЦВЕТА (подбери под свою камеру и освещение) ===
# # Вариант для синего автобуса (в HSV!)
# lower_blue = np.array([100, 70, 70])    # H: 100-120, S и V — не слишком тёмные
# upper_blue = np.array([130, 255, 255])

# # Минимальная площадь в пикселях (чтобы отсечь мусор)
# MIN_AREA = 50

# def process_image():
#     rate = rospy.Rate(30)  # 30 Гц максимум

#     while not rospy.is_shutdown():
#         try:
#             # Ждём новый кадр
#             ros_image = rospy.wait_for_message('/main_camera/image_raw_throttled', Image, timeout=5.0)
#         except rospy.ROSException:
#             rospy.logerr("Не пришёл кадр с камеры за 5 секунд!")
#             continue

#         # Конвертируем в OpenCV
#         cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')

#         # 1. Переводим в HSV
#         hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

#         # 2. Маска синего
#         mask = cv2.inRange(hsv, lower_blue, upper_blue)

#         # 3. Немного морфологии — убираем шум
#         kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
#         mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
#         mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

#         # 4. Находим контуры
#         contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         # print(contours, _)

#         # 5. Копия для отрисовки результата
#         result_image = cv_image.copy()
#         detected = False
#         biggest_area = 0
#         biggest_cx = biggest_cy = 0

#         for c in contours:
#             area = cv2.contourArea(c)
#             if area < MIN_AREA:
#                 continue

#             M = cv2.moments(c)
#             if M["m00"] == 0:
#                 continue

#             cx = int(M["m10"] / M["m00"])
#             cy = int(M["m01"] / M["m00"])

#             # Запоминаем самый большой
#             if area > biggest_area:
#                 biggest_area = area
#                 biggest_cx, biggest_cy = cx, cy
#                 detected = True

#             # Рисуем
#             cv2.drawContours(result_image, [c], -1, (0, 255, 0), 4)
#             cv2.circle(result_image, (cx, cy), 5, (0, 255, 255), -1)
#             cv2.putText(result_image, f"{area:.0f}", (cx + 20, cy - 20),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

#         # Если нашли — пишем в консоль
#         if detected:
#             x, y, z = get_marker_coordinates_in_aruco_map(cx, cy)
#             rospy.loginfo(f"СИНИЙ АВТОБУС! Площадь: {biggest_area:.0f}  Центр: ({biggest_cx}, {biggest_cy}) {x} {y} {z}")
#         else:
#             cv2.putText(result_image, "NO BLUE BUS", (50, 100),
#                         cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 6)

#         # === ПУБЛИКУЕМ ВСЁ В ТОПИКИ ===
#         stamp = ros_image.header.stamp

#         pub_original.publish(bridge.cv2_to_imgmsg(cv_image,     encoding="bgr8", header=Header(stamp=stamp, frame_id="camera")))
#         pub_mask.publish(    bridge.cv2_to_imgmsg(mask,         encoding="mono8", header=Header(stamp=stamp, frame_id="camera")))
#         pub_result.publish(  bridge.cv2_to_imgmsg(result_image, encoding="bgr8", header=Header(stamp=stamp, frame_id="camera")))

#         rate.sleep()

# if __name__ == '__main__':
#     # rospy.init_node('blue_bus_detector', anonymous=True)
#     rospy.loginfo("Запущен детектор синего автобуса. Смотри топики /blue_detection/*")

#     try:
#         process_image()
#     except rospy.ROSInterruptException:
#         pass