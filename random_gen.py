from xml.etree import ElementTree as ET  # Импортируем модуль для работы с XML
import random
import numpy as np

# ==============const===============
worldPath = '/home/clover/catkin_ws/src/clover/clover_simulation/resources/worlds/clover_aruco.world'
start_vector = np.array([1, 1])
pipe1_vector = np.array([0, 6])
pipe2_vector = np.array([1.987521, 3.471277])
pipe1_vector_norm = pipe1_vector / np.linalg.norm(pipe1_vector)
pipe2_vector_norm = pipe2_vector / np.linalg.norm(pipe2_vector)
pipe1i_vector = np.array([0.75, 0])
pipe2i_vector = np.array([0.650864, -0.372660])


def generate_insets():
    '''генерируем случ список расстояний между началом трубы и врезкой'''
    def check(res):
        '''проверка расстояния между врезками'''
        mi = float('inf')
        for i in range(len(res) - 1):
            mi = min(mi, res[i+1] - res[i])
        return mi <= 0.75

    res = [0]*5
    while check(res):
        # генерируем случайные расстояния врезок
        # и проверям сгенерированное с условием 
        res = [
            random.randrange(0, 200) / 100,
            random.randrange(200, 400) / 100,
            random.randrange(400, 600) / 100,
            random.randrange(600, 800) / 100,
            random.randrange(800, 1000) / 100,
        ]
    return res

def add_insets(dist: float, pipe2: bool, n: int):
    '''добавляет в мир gazebo врезку'''
    if pipe2:
        yaw = -0.52
        cords = start_vector + pipe1_vector + \
            pipe2_vector_norm * (dist - 6) + \
            (random.choice((-1, 1)) * pipe2i_vector)
    else:
        yaw = 0
        cords = start_vector + pipe1_vector_norm * dist + \
            (random.choice((-1, 1)) * pipe1i_vector)
    x, y = cords

    root = ET.parse(worldPath)
    world = root.getroot()
    include = ET.fromstring(
        '<include>'
        '   <uri>model://insets_pipe</uri>'
        f'  <name>insets_pipe_{n}</name>'
        f'  <pose>{x} {y} 0.03 0 0 {yaw}</pose>'
        '</include>'
    )
    world[0].insert(0, include)
    root.write(worldPath)
    

def main():
    dists = generate_insets()
    for i, d in enumerate(dists):
        add_insets(d, i >= 3, i)
    return dists

print(main())