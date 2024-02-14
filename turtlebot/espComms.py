import serial
import time
from serial.tools import list_ports

class ESPcomms():
    def __init__(self):
        self.port = "" #port to connect to
    
    def initSTMhandshake(self):
        try:
            self.serial = serial.Serial(self.port, baudrate=115200) # type: ignore
            #print("Serial port opened successfully")
            return True
        except Exception as e:
            #print("Error opening serial port: ", e)
            return False
        
    def send_data(self, data):
        try:
            self.serial.write(data.encode())
            return True
        except Exception as e:
            #print("Error sending data: ", e)
            return False
         
    def read_data(self):
        try:
            dataArray = []
            data = self.serial.read(6)
            if len(data) > 0:
                for bit in data:
                    dataArray.append(bit)
            return dataArray
        except Exception as e:
            #print("Error reading data: ", e)
            return False
        
    def close_connection(self):
        try:
            self.serial.close()
            #print("Serial port closed successfully")
            return True
        except Exception as e:
            #print("Error closing serial port: ", e)
            return False
        
    def scan_devices(self, connectBool = False, portNum = 0):
        """Scan for available ports. Optional argument connectBool determines if the port should be connected to. Optional argument portNum determines which port to connect to."""
        ports = list_ports.comports()
        if connectBool:
            self.port = ports[portNum]
        return [port.device for port in ports]

if __name__ == "__main__":
    esp = ESPcomms()
    portList = esp.scan_devices()
    try:
        portNum = int(input("Select port %s: "%portList))
    except:
        print("port init failed, try again.")
    esp.port = portList[portNum]
    esp.initSTMhandshake()
    while True:
        command = input("motor command: ")
        esp.send_data(command+ "\n")
        print("data sent")