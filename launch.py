#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Автоматическая подготовка симуляции CLOVER с ArUco-картой 5×12
и правильными настройками launch-файлов.

Что делает скрипт:
1. Устанавливает зависимости (Flask, scipy)
2. Генерирует карту ArUco-меток 5×12 (размер метки 0.33 м)
3. Применяет эту карту к пустому миру Gazebo → создаёт clover_aruco.world
4. Копирует дополнительные модели и перезаписывает world-файл настроенной версией
5. Автоматически правит launch-файлы aruco.launch и clover.launch
"""

from xml.etree import ElementTree as ET
import os
import shutil

# =========================== ПУТИ ===========================
aruco_launch_path   = '/home/clover/catkin_ws/src/clover/clover/launch/aruco.launch'
clover_launch_path  = '/home/clover/catkin_ws/src/clover/clover/launch/clover.launch'
empty_world_path    = '/home/clover/catkin_ws/src/clover/clover_simulation/resources/worlds/clover.world'
target_world_path   = '/home/clover/catkin_ws/src/clover/clover_simulation/resources/worlds/clover_aruco.world'
models_dest_path    = '/home/clover/catkin_ws/src/clover/clover_simulation/models/'
aruco_map_dir       = '/home/clover/catkin_ws/src/clover/aruco_pose/map/'
aruco_map_filename  = 'cmit.txt'                     # Имя файла карты
aruco_map_fullpath  = os.path.join(aruco_map_dir, aruco_map_filename)

marker_length = 0.33                                  # Размер одной ArUco-метки в метрах


# =========================== 1. Установка зависимостей ===========================
print("Установка Python-зависимостей (Flask, scipy)...")
result = os.system('pip3 install Flask scipy')
if result != 0:
    print("Ошибка установки Flask/scipy. Установите вручную: pip3 install Flask scipy")


# =========================== 2. Генерация карты ArUco ===========================
print(f"Генерация карты ArUco {5}x{12}, размер метки {marker_length} м...")
cmd_genmap = (
    f"rosrun aruco_pose genmap.py {marker_length} 5 12 1 1 0 "
    f"--top-left > {aruco_map_fullpath}"
)
result = os.system(cmd_genmap)
if result != 0:
    print("Ошибка при генерации карты ArUco!")
    exit(1)
print(f"Карта успешно сохранена: {aruco_map_fullpath}")


# =========================== 3. Применение карты к миру Gazebo ===========================
print("Применяем карту к пустому миру → создаём clover_aruco.world...")
cmd_aruco_gen = (
    f"rosrun clover_simulation aruco_gen "
    f"--single-model "
    f"--source-world={empty_world_path} "
    f"{aruco_map_fullpath} "
    f"> {target_world_path}"
)
result = os.system(cmd_aruco_gen)
if result != 0:
    print("Ошибка при генерации мира с ArUco-метками!")
    exit(1)
print(f"Мир с метками успешно создан: {target_world_path}")


# =========================== 4. Копирование дополнительных моделей и настроенного мира ===========================
print("Копируем дополнительные модели и финальный world-файл...")

# Копируем свои модели (трубы, врезки и т.д.) в папку симуляции
source_models = './gazebo_settings/gazebo_models'
if os.path.isdir(source_models):
    shutil.copytree(source_models, models_dest_path, dirs_exist_ok=True)
    print("Модели скопированы")
else:
    print(f"Папка с моделями не найдена: {source_models}")

# Перезаписываем мир своей «готовой» версией
custom_world = './gazebo_settings/clover_aruco.world'
if os.path.isfile(custom_world):
    shutil.copy(custom_world, target_world_path)
    print("Кастомный clover_aruco.world установлен")
else:
    print(f"Кастомный мир не найден: {custom_world}")


# =========================== 5. Редактирование aruco.launch ===========================
print("Настраиваем aruco.launch...")

# Обработка файла aruco.launch
root = ET.parse(aruco_launch_path)  # Парсим XML файл aruco
aruco = root.getroot()  # Получаем корневой элемент

# Устанавливаем значения для аргументов в aruco.launch
for i in range(len(root.findall('arg'))):
    if aruco[i].attrib['name'] in ['aruco_detect', 'aruco_map', 'aruco_vpe']:
        aruco[i].attrib['default'] = 'true'  # Устанавливаем значение на true для этих аргументов
    elif aruco[i].attrib['name'] == 'placement':
        aruco[i].attrib['default'] = 'floor'  # Устанавливаем значение ию для размещения
    elif aruco[i].attrib['name'] == 'map':
        aruco[i].attrib['default'] = aruco_map_filename  # Устанавливаем имя карты
    elif aruco[i].attrib['name'] == 'length':
        aruco[i].attrib['default'] = str(marker_length)  # Устанавливаем длину из файла карты
root.write(aruco_launch_path)  # Записываем изменения обратно в файл aruco.launch


# =========================== 6. Редактирование clover.launch ===========================
print("Настраиваем clover.launch...")

# Обработка файла clover.launch
root = ET.parse(clover_launch_path)  # Парсим XML файл clover
aruco = root.getroot()  # Получаем корневой элемент

# Устанавливаем значения  для аргументов в clover.launch
for i in range(len(root.findall('arg'))):
    if aruco[i].attrib['name'] in ['simulator', 'web_video_server', 'rosbridge', 'main_camera', 'optical_flow', 'aruco', 'rangefinder_vl53l1x', 'led']:
        aruco[i].attrib['default'] = 'true'  # Устанавливаем значение на true для этих аргументов
    elif aruco[i].attrib['name'] in ['blocks', 'rc']:
        aruco[i].attrib['default'] = 'false'  # Устанавливаем значение на false для этих аргументов

root.write(clover_launch_path)  # Записываем изменения обратно в файл clover.launch


print("\nНастройка завершена!")
