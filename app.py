from flask import Flask, render_template
from flask_socketio import SocketIO
import serial
import threading

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

ARDUINO_PORT = "/dev/cu.usbmodem101"
BAUD_RATE = 115200

def read_serial():
    try:
        ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            if line:
                socketio.emit('sensor_data', line)
    except Exception as e:
        print("Serial read error:", e)

@app.route("/")
def index():
    return render_template("index.html")

if __name__ == "__main__":
    thread = threading.Thread(target=read_serial)
    thread.daemon = True
    thread.start()
    socketio.run(app, host="0.0.0.0", port=8000)