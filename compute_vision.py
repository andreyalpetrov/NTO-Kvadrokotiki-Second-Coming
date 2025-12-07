import numpy as np
import cv2
from cv_bridge import CvBridge
import numpy as np


bridge = CvBridge()

# ======================== ОСНОВНОЙ КОД ========================
def callback_video(data):
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8') # OpenCV image
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # blurred = cv2.GaussianBlur(img, (5, 5), 0)

    lower_blue = np.array([100, 60, 50])
    upper_blue = np.array([150, 255, 255])
    # 120, 130, 255

    # Маска: только синие пиксели
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Улучшаем маску: убираем мелкий шум и соединяем разрывы
    # blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9,9)))
    # blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN,  cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5)))

    # Детекция краёв только на синей маске
    edges = cv2.Canny(blue_mask, 50, 150)

    # Поиск отрезков ТОЛЬКО на синих линиях
    lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180,
                            threshold=40,          # можно чуть снизить — линий меньше, но они "чище"
                            minLineLength=40,
                            maxLineGap=15)

    perpendicular_intersections = []

    if lines is not None:
        lines = lines[:, 0, :]  # (N, 4)

        print(f"Найдено {len(lines)} синих отрезков")

        # Рисуем все найденные синие линии (для отладки)
        # for x1, y1, x2, y2 in lines:
        #     cv2.line(img, (x1, y1), (x2, y2), (255, 255, 0), 2)  # жёлтые линии

        # Перебираем пары и ищем перпендикулярные пересечения
        for i in range(len(lines)):
            for j in range(i + 1, len(lines)):
                line1 = lines[i]
                line2 = lines[j]

                # Вычисляем угол между линиями через направляющие векторы
                dx1, dy1 = line1[2] - line1[0], line1[3] - line1[1]
                dx2, dy2 = line2[2] - line2[0], line2[3] - line2[1]

                # Скалярное произведение векторов
                dot = dx1*dx2 + dy1*dy2
                mag1 = np.sqrt(dx1*dx1 + dy1*dy1)
                mag2 = np.sqrt(dx2*dx2 + dy2*dy2)

                if mag1 == 0 or mag2 == 0:
                    continue

                cos_angle = dot / (mag1 * mag2)
                cos_angle = np.clip(cos_angle, -1.0, 1.0)
                angle = np.degrees(np.arccos(np.abs(cos_angle)))  # угол между 0° и 90°

                # Проверяем, близок ли угол к 90° (допуск ±12°)
                if abs(angle - 90) <= 12:
                    # Находим точку пересечения
                    x1, y1, x2, y2 = line1
                    x3, y3, x4, y4 = line2

                    denom = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4)
                    if abs(denom) < 1e-8:
                        continue  # параллельны

                    px = ((x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4)) / denom
                    py = ((x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4)) / denom

                    x, y = int(px), int(py)

                    # Проверяем, что точка лежит на обоих отрезках
                    if (min(x1, x2) <= x <= max(x1, x2) and min(y1, y2) <= y <= max(y1, y2) and
                        min(x3, x4) <= x <= max(x3, x4) and min(y3, y4) <= y <= max(y3, y4)):

                        perpendicular_intersections.append((x, y))
                        # cv2.circle(img, (x, y), 15, (0, 0, 255), -1)      # большой красный круг
                        # cv2.line(img, (line1[0], line1[1]), (line1[2], line1[3]), (0, 255, 255), 4)
                        # cv2.line(img, (line2[0], line2[1]), (line2[2], line2[3]), (0, 255, 255), 4)
    return perpendicular_intersections, cv_image
        # print(f"Найдено {len(perpendicular_intersections)} перпендикулярных пересечений синих линий:")
        # for pt in perpendicular_intersections:
        #     print(pt)
    # else:
    #     print("Синие линии не обнаружены")

    # # Показываем результат
    # cv2.imshow('Только СИНИЕ линии + пересечения под ~90°', img)
    # cv2.imshow('Синяя маска', blue_mask)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()