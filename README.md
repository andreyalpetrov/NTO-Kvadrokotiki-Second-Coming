|||
| --- | --- |
| Трек | Летающая робототехника |
| Задание | Командное задание |
| Тип | Инструкция |
| Команда | КвадроКотики |

| Участник           | Роль                |
| ------------------ | ------------------- |
| • Андрей Петров    | инженер техник |
| • Даниил Белов     | инженер программист |
| • Иван Александров | инженер программист |
| • Дмитрий Ковалев  | 3D-моделлер         |

## 1. Установка виртуальной машины.

Для установки виртуальной машины необходимо скачать ПО для виртуализации (Virtual Box, WMVare Player или WMVare Workstation). 
В нашем примере мы будем использовать WMVare Workstation Pro. 
- Устанавливаем ПО для виртуализации
- Скачиваем образ виртуальной машины с симулятором Клевера с репозитория по ссылке: 
 [Release v1.4 · CopterExpress/clover_vm](https://github.com/CopterExpress/clover_vm/releases/tag/v1.4) 
 - Открываем ПО для виртуализации, нажимаем `CTRL+O` , или `File->Open` чтобы открыть существующую виртуальную машину. Выбираем скаченный файл `clover-devel_v1.4.ova`
 - Нам предложат ввести название виртуальной машины. Называем `Clover`
 - После установки образа и создания ВМ мы можем её настроить. Выбираем нашу ВМ в правой колонке `Library`. Затем нажимаем `Edit virtual machine settings`. В открывшемся окне настраиваем параметры:
	 - Memory. Необходимо выделить минимум половину от имеющейся оперативной памяти. (У нас это будет 8Гб), но мы выбираем выделить около 12-13 Гб, т.к. в целом при 16Гб это не сильно повлияет на работоспособность ПК, но ускорит работу симуляции. 
	- Processors. Количество процессоров - 1 (Number of processors).
				Количество ядер - 3. (Number of cores per perocessor).
	- Network Adapter.  Выбрать тип соединения `Bridged: Connected directly to the physical network.` Поставить галочку  на `Connect at power on`. 
	Остальное можно оставить по умолчанию. Жмем кнопку `Ok`. Теперь отображаются актуальные параметры.

Для запуска скрипта прописываем.
```bash
cd Desktop
git clone https://github.com/andreyalpetrov/NTO-Kvadrokotiki-Second-Coming.git
```
Перейдя в проводник, перетащите все файлы из скаченной папки на рабочий стол.
## 2. Настройка симулятора. 


Для настройки симулятора был написан скрипт, который выполняет следующие действия: 
	- Проверяет, существует ли Aruco-карта с тем же названием из файла `clover_aruco.world` в папке `aruco_pose/map` 
	- Если карта существует, то скрипт вносит изменения в `aruco.launch`, где в том числе меняет название карты, а также вносит параметр размера маркера из файла карты `.txt`
	- Вносит необходимые изменения в файл `clover.launch` и `aruco.launch`.
	- Генерирует модель нефтепровода с одним изгибом.


- Файл `launch.py` должен находится на рабочем столе (см. пункт 1 Установка виртуальной машины).
- Вызываем терминал из рабочего стола, либо переходим с помощью команды и вызываем скрипт.
```bash
cd Desktop #Если вы все ещё не находитесь в данной дериктории - перейдите в неё
python3 launch.py
```

Исходный код скрипта c комментариями: 

``` python

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
```

В нашем случае мы будем работать со стандартной картой `cmit.txt`, именно она прописана в мире `clover_aruco.world`, но у вас может быть сгенерирована другая карта. В случае, если в "мозгах" Клевера не будет найден файл карты как в мире, то скрипт предложить сгенерировать вам карту с помощью утилиты `genmap.py` и даст ссылку на конкретный пункт в документации. Вам нужно будет создать такую же карту, как и на поле в симуляции. Соответствие модели карты и ещё описания скрипт не проверяет. 
## 3. Случайная генерация мира.

Случайная генерация мира происходит с помощью скрипта, который прописывает в файл `clover_aruco.world` подключение папок с нефтепроводом и врезками, а также обеспечивает рандомную генерацию врезок согласно правилам.


С каждым запуском скрипт генерирует расположение врезок случайно. 

Применение скрипта: 

- Закройте Gazebo. 
- Откройте терминал и вызовите следующие команды: НАДО ИСПРАВИТЬ
```bash
cd Desktop #Если вы все ещё не находитесь в данной дериктории - перейдите в неё
python3 houses.py
```
- Запустите Gazebo 

Исходный код программы с комментариями: 

``` python

```

## 4. Полетная миссия НАДО ПОЛНОСТЬЮ ПЕРЕПИСАТЬ

Полетная миссия представляет из себя полет дрона из точки `(0, 0)` траекторией "змейка". Происходит сканирование каждого ArUco-маркера на наличие здания. Как только крыша здания будет находится в центре кадра камеры, то программа определит цвет, тип и координаты здания.  Отладочную информацию можно увидеть в терминале, откуда происходит запуск ноды, а также информацию о типах здания можно увидеть, подписавшись на топик `/buldings.
`
> [!IMPORTANT]
> Перед запуском полетной миссии нужно выполнить в терминале команду: 
> ``` bash
>pip install Flask==2.2.2
>```
>Данная команда устанавливает более новую версию модуля Flask. Он необходим для работы веб-сервиса. 

- Файл `colour_vse.py` должен находится на рабочем столе (см. пункт 1 Установка виртуальной машины).
- Создайте терминал. В данном терминале мы будем запускать миссию. и наблюдать за отладочной информацией.
- Откройте Gazebo. 
- Введите в терминале следующие команды:
```bash
cd Desktop #Если вы все ещё не находитесь в данной дериктории - перейдите в неё
python3 colour_vse.py
```
Миссия запущенна. Начать полет можно в веб сервисе (см. пункт 5. Работа с веб-сервисом).
> [!IMPORTANT]
> Полная остановка кода осуществляется через закрытие окна терминала.

Также возможен просмотр топика.
- Запустите ещё один терминал и введите следующую команду, чтобы подписаться на топик `/buildings`: 
```bash
rostopic echo /buildings
```
В случае нахождения здания в данный топик будет публиковаться информация о цвете, типе здания и его координатах.

Исходный код программы с комментариями: 
```python
```

## 5. Работа с веб-сервисом !!! Надо ПЕРЕПИСАТЬ

Веб-сервис позволяет нам контролировать полёт дрона.
- Перед открытием веб-сервиса необходимо запустить полетную миссию (см. пункт 4. Полетная миссия).
- Откройте браузер.
- В поисковой строке введите адрес http://localhost:4000/

<img src="/web-service.png" style="width:50%; height:auto;">

В веб-сервисе можно наблюдать 2d карту, с началом координат в левом нижнем углу. На данной карте отображаются все найденые дома с обозначением его координат и цвета. Каждый квадрат на карте соответствует 1м в симуляторе. Для того чтобы не нагружать систему дрона, расположение домов обновляется каждые 100мс.

Ниже карты имеются кнопки управления, а также подпись последнего действия (изначально `Landed`). 
- Если была нажата кнопка `Start`, то дрон начнёт или продолжит с последнего шага свой полет. Доступны последующие кнопки `Stop` и `Kil switch`. Действие `Started`.
- Если была нажата кнопка `Stop`, то дрон будет посажен. Доступна последующая кнопка `Start` (только после полной посадки). Действие `Landed`.
- Если была нажата кнопка `Kill switch`, то дрон опустится на землю, а код будет прерван. Никакие последующие кнопки не доступны. Действие `Killed`.

> [!IMPORTANT]
> Команда `Kill switch` прерывает работу кода, но полное отключение осуществляется через закрытие окна терминала.

> [!IMPORTANT]
> Посадка дрона осуществляется только на поверхность пола. Если посадить дрон на дом, то сработает `Failsafe` из-за не верной поверхности и не возможно будет взлететь.

Также чуть ниже в веб-сервисе представлен список найденых зданий с указанием их цвета, координат и типа.

Исходный код веб-сервиса с комментариями: 

```python
from flask import Flask, request, jsonify

# Инициализация приложения Flask
app = Flask(__name__)

# Создание класса Server, для хранения функций и переменных веб-сервиса
class Server:
  action = 'landed' # Текущее состояние, по умолчанию "landed"
  buildings = [] # Список зданий на "карте"
  
  # Эндпоинт для обновления текущего состояния сервера
  @app.route('/action', methods=['POST'])
  def actPage():
    Server.action = request.json['action'] # Установка нового состояния из запроса
    return '' # Пустой ответ
  
  # Эндпоинт для получения текущего состояния и списка зданий
  @app.route('/buildings')
  def positionsPage():
    return [Server.action, Server.buildings] # Возвращает состояние и список зданий
  
  # Главная страница, генерирует HTML-код с интерфейсом
  @app.route('/')
  def actionPage():
    return '''
  <div style="text-align:center">
    <!-- Основной блок с отображением карты и кнопок управления -->
    <div style="display:flex; flex-direction:column;">
      
      <div id="map" style="position:relative; margin: auto">
        <!-- Карта, состоящая из ячеек -->
        <span style="position:absolute; left: -15px; bottom: -5px; font-size: 20px"> 0</span>
        
        <div id="building0" style="display:none; position:absolute; width:50px; height:50px; text-align:center;">
        </div>
        <div id="building1" style="display:none; position:absolute; width:50px; height:50px; text-align:center;">
        </div>
        <div id="building2" style="display:none; position:absolute; width:50px; height:50px; text-align:center;">
        </div>
        <div id="building3" style="display:none; position:absolute; width:50px; height:50px; text-align:center;">
        </div>
        <div id="building4" style="display:none; position:absolute; width:50px; height:50px; text-align:center;">
        </div>
        
      </div>
      
      
        
    </div>
    <div style="margin-top: 20px">
        <!-- Кнопки управления -->
        <button id="start-button" type="button" style="dislay:block; color:green">Start</button>
        <button id="stop-button" type="button" style="dislay:block; color:red">Stop</button>
        <button id="kill-button" type="button" style="dislay:block; color:red">Kill switch</button>
    </div>
    <span id="message"></span>
    <span id="buildings"></span>
  </div>

  <script>
    // Код JavaScript, управляющий отображением карты и обработкой кнопок
    
    // При загрузке страницы создаётся карта и запускается обновление зданий
    document.body.onload = () => {createMap(); setInterval(updateBuildings, 100);};
    
    // Обработчики для кнопок
    document.getElementById("start-button").addEventListener("click", startClick);
    document.getElementById("stop-button").addEventListener("click", stopClick);
    document.getElementById("kill-button").addEventListener("click", killClick);
    
    let startStatus = "''' + Server.action + '''"; // Хранит текущее состояние
    
    // Обновляет информацию о зданиях на карте
    function updateBuildings(){
      fetch("/buildings", {
        method: "GET",
        headers: {"Content-type": "application/json"}
      })
      .then(response => response.json())
      .then(json => {
        let bs = document.getElementById("buildings");
        bs.innerHTML = "";
        if(json[0] == "landed" && startStatus == "landing"){ // Обработка посадки
          startStatus = json[0];
          document.getElementById("start-button").style.color = "green";
          document.getElementById("stop-button").style.color = "red";
          document.getElementById("kill-button").style.color = "green";
          document.getElementById("message").innerHTML = "Landed";
        }
        for(let i = 0; i < json[1].length; i++){
          let b = document.getElementById("building" + i.toString());
          b.style.display = "block"
          b.style.left = json[1][i][0] * 50 + 2 * json[1][i][0] - 24
          b.style.bottom = json[1][i][1] * 50 + 2 * json[1][i][1] - 24
          b.style.background = (json[1][i][2] == "yellow"?"#e6db00":json[1][i][2])
          b.innerHTML = "<span style=\\"color:white\\">x: " + json[1][i][0] + "</span><br/><span style=\\"color:white\\">y: " + json[1][i][1] + "</span>";
          bs.innerHTML = bs.innerHTML + "<br/>" + (json[1][i][2]=="red"?"Администрация":(json[1][i][2]=="green"?"Лаборатория":(json[1][i][2]=="yellow"?"Шахта":"Здание для обогащения угля"))) + "; color: " + json[1][i][2] + "; x: " + json[1][i][0] + "; y: " + json[1][i][1];
        }
      });
    }
    
    // Создаёт карту (9x9 ячеек)
    function createMap() {
      let map = document.getElementById('map');
      document.getElementById("start-button").style.color = "'''+('green' if Server.action == 'landed' else 'red') + '''";
      document.getElementById("stop-button").style.color = "'''+('green' if Server.action == 'start' else 'red') + '''";
      document.getElementById("kill-button").style.color = "'''+('green' if Server.action != 'kill' else 'red') + '''";
      document.getElementById("message").innerHTML = "''' + ('Landed' if Server.action == 'landed' else ('Landing' if Server.action == 'landing' else ('Started' if Server.action == 'start' else 'Killed'))) + '''";

      for (let i = 0; i < 9; i++) {
        let row = document.createElement('div');
        row.style = 'display: flex; justify-content:center';
        for (let i = 0; i < 9; i++) {
          let col = document.createElement('div');
          col.style = 'width:50px; height:50px; border:solid 1px gray';
          row.appendChild(col);
        }
        map.appendChild(row);
      }
    }
    
    // Логика обработки кнопки "Start"
    function startClick() {
      if(startStatus == "start" || startStatus == "kill"|| startStatus == "landing"){
        return
      }
      fetch("/action", {
        method: "POST",
        body:JSON.stringify({action:"start"}),
        headers: {"Content-type": "application/json"}
      })
      document.getElementById("start-button").style.color = "red";
      document.getElementById("stop-button").style.color = "green";
      document.getElementById("kill-button").style.color = "green";
      document.getElementById("message").innerHTML = "Started";
      startStatus = "start";
    }
    
    // Логика обработки кнопки "Stop"
    function stopClick() {
      if(startStatus == "landed" || startStatus == "kill" || startStatus == "landing"){
        return
      }
      fetch("/action", {
        method: "POST",
        body:JSON.stringify({action:"landing"}),
        headers: {"Content-type": "application/json"}
      })
      document.getElementById("start-button").style.color = "red";
      document.getElementById("stop-button").style.color = "red";
      document.getElementById("kill-button").style.color = "red";
      document.getElementById("message").innerHTML = "Landing";
      startStatus = "landing";
    }
    
    // Логика обработки кнопки "Kill"
    function killClick() {
      if(startStatus == "kill" || startStatus == "landing"){
        return
      }
      fetch("/action", {
        method: "POST",
        body:JSON.stringify({action:"kill"}),
        headers: {"Content-type": "application/json"}
      })
      document.getElementById("start-button").style.color = "red";
      document.getElementById("stop-button").style.color = "red";
      document.getElementById("kill-button").style.color = "red";
      document.getElementById("message").innerHTML = "Killed";
      startStatus = "kill";
    }
    
  </script>
  '''
  
  # Метод для запуска сервера
  def start():
    app.run(host='0.0.0.0', port='4000') # Сервер доступен на порту 4000

```
