import serial 
import time
from datetime import datetime

LOG_FILE = "bms_data_log.txt"

ser = serial.Serial(
    port = '/dev/ttyACM0', 
    baudrate=115200,
    timeout=1
)

def send(cmd) :

    ser.write((cmd + '\r').encode())
    time.sleep(0.1)
    resp = ser.read_all().decode(errors='replace')
    log(cmd, resp)
    return ser.read_all().decode()

def log(cmd, response):

    ts = datetime.now().strftime("%Y-%m-%d  %H:%M:%S")
    with open(LOG_FILE, "a") as f:
        f.write(f"[{ts}] - CMD : {cmd} | RESP : {response.strip()}\n")

def fetchData():
    
    send('?A 1')
    send('?V 4')

fetchData()

ser.close()