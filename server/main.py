import threading
import subprocess
import rospy
from std_srvs.srv import Trigger, TriggerRequest
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from flask import Flask, render_template, jsonify
import topic_service as topic 


land = rospy.ServiceProxy('/land', Trigger)
arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

app = Flask(__name__)

# Глобальная переменная для процесса миссии
mission_process: subprocess.Popen = None

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/start')
def start_mission():
    global mission_process
    if mission_process is not None and mission_process.poll() is None:
        return jsonify({"status": "error", "message": "Миссия уже запущена"}), 400
    
    try:
        mission_process = subprocess.Popen(['python3', 'mission.py']) 
        return jsonify({"status": "ok", "message": "Миссия запущена"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/api/stop')
def stop_mission():
    global mission_process
    if mission_process is None or mission_process.poll() is not None:
        return jsonify({"status": "error", "message": "Миссия не запущена"}), 400
    
    mission_process.terminate()  # сначала мягко
    try:
        mission_process.wait(timeout=0.5)
    except subprocess.TimeoutExpired:
        mission_process.kill()  # если не умер — принудительно
    land()
    mission_process = None
    return jsonify({"status": "ok", "message": "Миссия остановлена"})

@app.route('/api/killswitch')
def killswitch():
    global mission_process
    
    # Убиваем миссию
    if mission_process is not None and mission_process.poll() is None:
        mission_process.kill()
        mission_process = None

    arming(False)

    return jsonify({"status": "ok", "message": "Аварийная остановка выполнена"})

@app.route('/api/get_insets_cords')
def get_insets_cords():
    return jsonify(topic.mission_data.get('tubes', []))

# Запуск потока для подписки на топики (из topic_service.py)
thread = threading.Thread(target=topic.main, name="TopicThread", daemon=True)
thread.start()

if __name__ == '__main__':
    try:
        app.run(host='localhost', port=5000, debug=False)
    except (KeyboardInterrupt, SystemExit):
        rospy.signal_shutdown("Web server stopped")