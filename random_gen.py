from xml.etree import ElementTree as ET    # Модуль для парсинга и изменения XML-файлов (world-файлы Gazebo)
import random                              # Для генерации случайных чисел
import numpy as np                         # Для удобной работы с векторами и нормализацией

# ============== Константы ===============
# Путь к основному world-файлу симуляции CLOVER с ArUco-метками
worldPath = '/home/clover/catkin_ws/src/clover/clover_simulation/resources/worlds/clover_aruco.world'

# Начальная точка отсчёта (координаты старта первой трубы)
start_vector = np.array([1.0, 1.0])

# Векторы, описывающие направление и длину участков труб
pipe1_vector = np.array([0.0, 6.0])          # Первый прямой участок длиной 6 м по оси Y
pipe2_vector = np.array([1.987521, 3.471277])  # Второй наклонный участок

# Нормализованные векторы-единицы (длина = 1) — нужны для расчёта позиций по расстоянию
pipe1_vector_norm = pipe1_vector / np.linalg.norm(pipe1_vector)   # [0, 1]
pipe2_vector_norm = pipe2_vector / np.linalg.norm(pipe2_vector)

# Перпендикулярные векторы — используются для случайного смещения врезки влево/вправо от оси трубы
pipe1i_vector = np.array([0.75, 0.0])        # Смещение для первой трубы (±0.75 м по X)
pipe2i_vector = np.array([0.650864, -0.372660])  # Смещение для второй трубы (перпендикулярно её направлению)


def generate_insets() -> list[float]:
    """
    Генерирует список из 5 случайных расстояний от начала соответствующей трубы до врезки.
    
    Условие: минимальное расстояние между любыми двумя врезками >= 0.75 м
    (чтобы модели не пересекались и не «залипали» друг в друга).
    
    Возвращает отсортированный список из 5 значений в метрах (от 0 до 10 м).
    """
    def has_too_close_insets(distances: list[float]) -> bool:
        """Проверяет, есть ли врезки ближе 0.75 м друг к другу"""
        min_distance = float('inf')
        for i in range(len(distances) - 1):
            min_distance = min(min_distance, distances[i + 1] - distances[i])
        return min_distance <= 0.75                     # Если нашли слишком близкие — возвращаем True

    while True:
        # Генерируем 5 случайных расстояний в разных диапазонах,
        # чтобы они изначально были в возрастающем порядке и покрывали всю длину труб
        candidates = [
            random.randrange(0,   200) / 100,   # 0.00 – 1.99 м  (первая труба)
            random.randrange(200, 400) / 100,   # 2.00 – 3.99 м  (первая труба)
            random.randrange(400, 600) / 100,   # 4.00 – 5.99 м  (первая труба)
            random.randrange(600, 800) / 100,   # 6.00 – 7.99 м  (вторая труба)
            random.randrange(800, 1000) / 100,  # 8.00 – 9.99 м  (вторая труба)
        ]
        # Если минимальный зазор больше 0.75 м — считаем генерацию удачной
        if not has_too_close_insets(candidates):
            return candidates


def add_insets(dist: float, pipe2: bool, n: int) -> None:
    """
    Добавляет в world-файл Gazebo одну модель врезки (insets_pipe).
    
    :param dist: расстояние от начала соответствующей трубы (в метрах)
    :param pipe2: True — врезка на второй (наклонной) трубе, False — на первой
    :param n:    порядковый номер врезки (используется в имени модели)
    """
    if pipe2:
        # Вторая труба имеет наклон ≈ -0.52 рад (примерно -30°)
        yaw = -0.52
        # Расчёт координат:
        # start + полный первый участок + смещение по второй трубе + случайное боковое смещение
        cords = (start_vector +
                 pipe1_vector +                                 # доходим до начала второй трубы (6 м)
                 pipe2_vector_norm * (dist - 6.0) +             # идём по второй трубе нужное расстояние
                 random.choice((-1, 1)) * pipe2i_vector)        # ± перпендикулярное смещение
    else:
        # Первая труба идёт строго вдоль оси Y
        yaw = 0.0
        cords = (start_vector +
                 pipe1_vector_norm * dist +                     # идём по первой трубе
                 random.choice((-1, 1)) * pipe1i_vector)        # ± смещение влево/вправо

    x, y = cords

    # Читаем текущий world-файл
    root = ET.parse(worldPath)
    world = root.getroot()

    # Формируем блок <include> с моделью врезки
    include_block = ET.fromstring(f'''
        <include>
            <uri>model://insets_pipe</uri>
            <name>insets_pipe_{n}</name>
            <pose>{x:.6f} {y:.6f} 0.03 0 0 {yaw:.6f}</pose>
        </include>
    ''')

    # Вставляем новую врезку в самое начало списка моделей (world[0] — это обычно <world><...><include>...</include></world>)
    # Вставка в начало
    world[0].insert(0, include_block)

    # Перезаписываем файл
    root.write(worldPath, encoding='utf-8', xml_declaration=True)


def main() -> list[float]:
    """
    Основная функция: генерирует случайные позиции и добавляет 5 врезок в мир.
    Первые три врезки — на первой трубе, последние две — на второй.
    """
    distances = generate_insets()          # Получаем 5 расстояний с гарантированным зазором
    
    for i, dist in enumerate(distances):
        is_second_pipe = i >= 3            # с индекса 3 и далее — вторая труба
        add_insets(dist, is_second_pipe, i)
    
    return distances                       # Возвращаем для печати/логирования


if __name__ == '__main__':
    result = main()
    print('Сгенерированные расстояния от начала трубы до врезок (м):')
    for i, d in enumerate(result):
        pipe_name = "вторая труба" if i >= 3 else "первая труба"
        print(f"  Врезка {i} ({pipe_name}): {d:.2f} м")