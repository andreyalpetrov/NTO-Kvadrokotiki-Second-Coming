import rospy
import numpy as np
from sensor_msgs.msg import CameraInfo
from clover import srv
import tf2_ros
from scipy.spatial.transform import Rotation


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

