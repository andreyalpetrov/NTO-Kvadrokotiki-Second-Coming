from xml.etree import ElementTree as ET  # Импортируем модуль для работы с XML
import os
import shutil

# Определяем пути к различным ресурсам и необходимые константы
arucoPath = '/home/clover/catkin_ws/src/clover/clover/launch/aruco.launch'
cloverPath = '/home/clover/catkin_ws/src/clover/clover/launch/clover.launch'
emptyWorldPath = '/home/clover/catkin_ws/src/clover/clover_simulation/resources/worlds/clover.world'
worldPath = '/home/clover/catkin_ws/src/clover/clover_simulation/resources/worlds/clover_aruco.world'
modelsPath = '/home/clover/catkin_ws/src/clover/clover_simulation/models/'
mitPath = '/home/clover/catkin_ws/src/clover/aruco_pose/map/'
mitFilename = 'cmit.txt'
length = 0.33            # длина метки аруко карты


# ===============pip================
res = os.system('pip3 install Flask==2.2.2')

if res != 0:
    print('Ошибка установки библиотеки flask. Установить её вручную')

# ==============genmap==============

# создаем карту 5 на 12
res = os.system(f'rosrun aruco_pose genmap.py {length} 5 12 1 1 0 > {mitPath}{mitFilename} --top-left')

if res != 0:
    print('Ошибка создания карты')
    exit()

# ==============world===============

# применяем карту в gazebo
res = os.system('rosrun clover_simulation aruco_gen --single-model \\' 
          f'--source-world={emptyWorldPath} \\' 
          f'{mitPath}{mitFilename} > \\' + worldPath)
if res != 0:
    print('Ошибка применения карты в gazebo')
    exit()
# копируем все объекты мира
shutil.copytree('./gazebo_settings/gazebo_models', modelsPath, dirs_exist_ok=True)
# копируем конфигурацию мира
shutil.copy('./gazebo_settings/clover_aruco.world', worldPath)

# ==============launch==============

# Обработка файла aruco.launch
root = ET.parse(arucoPath)  # Парсим XML файл aruco
aruco = root.getroot()  # Получаем корневой элемент

# Устанавливаем значения для аргументов в aruco.launch
for i in range(len(root.findall('arg'))):
    if aruco[i].attrib['name'] in ['aruco_detect', 'aruco_map', 'aruco_vpe']:
        aruco[i].attrib['default'] = 'true'  # Устанавливаем значение на true для этих аргументов
    elif aruco[i].attrib['name'] == 'placement':
        aruco[i].attrib['default'] = 'floor'  # Устанавливаем значение ию для размещения
    elif aruco[i].attrib['name'] == 'map':
        aruco[i].attrib['default'] = mitFilename  # Устанавливаем имя карты
    elif aruco[i].attrib['name'] == 'length':
        aruco[i].attrib['default'] = str(length)  # Устанавливаем длину из файла карты
root.write(arucoPath)  # Записываем изменения обратно в файл aruco.launch

# Обработка файла clover.launch
root = ET.parse(cloverPath)  # Парсим XML файл clover
aruco = root.getroot()  # Получаем корневой элемент

# Устанавливаем значения  для аргументов в clover.launch
for i in range(len(root.findall('arg'))):
    if aruco[i].attrib['name'] in ['simulator', 'web_video_server', 'rosbridge', 'main_camera', 'optical_flow', 'aruco', 'rangefinder_vl53l1x', 'led']:
        aruco[i].attrib['default'] = 'true'  # Устанавливаем значение на true для этих аргументов
    elif aruco[i].attrib['name'] in ['blocks', 'rc']:
        aruco[i].attrib['default'] = 'false'  # Устанавливаем значение на false для этих аргументов

root.write(cloverPath)  # Записываем изменения обратно в файл clover.launch
