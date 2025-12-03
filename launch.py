from xml.etree import ElementTree as ET  # Импортируем модуль для работы с XML

# Определяем пути к различным ресурсам
arucoPath = '/home/clover/catkin_ws/src/clover/clover/launch/aruco.launch'
cloverPath = '/home/clover/catkin_ws/src/clover/clover/launch/clover.launch'
worldPath = '/home/clover/catkin_ws/src/clover/clover_simulation/resources/worlds/clover_aruco.world'
mitPath = '/home/clover/catkin_ws/src/clover/aruco_pose/map/'


try: # Попытка открыть и обработать файл мира (world)
    root = ET.parse(worldPath)  # Парсим XML файл мира
    world = root.getroot()  # Получаем корневой элемент
    mit = world[0][2][0].text.split('_')  # Извлекаем текст и разбиваем его на части
    f = open(mitPath + mit[1] + '.' + mit[2], 'r')  # Открываем файл карты по извлеченному имени
    length = f.read().split('\n')[1].split('\t')[1]  # Читаем длину из файла карты
except:
    print('Map not found! Run genmap.py, https://clover.coex.tech/en/aruco_map.html#marker-map-definition')  # Выводим сообщение об ошибке
    exit()  # Завершаем выполнение программы

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
        aruco[i].attrib['default'] = mit[1] + '.' + mit[2]  # Устанавливаем имя карты
    elif aruco[i].attrib['name'] == 'length':
        aruco[i].attrib['default'] = length  # Устанавливаем длину из файла карты
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
