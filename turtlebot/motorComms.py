import serial
import time
import struct
from serial.tools import list_ports
from turtlebot.configuration import *

class MotorMessage:
    def __init__(self):
        self.position_x = 0
        self.position_y = 0
        self.position_angular = 0
        self.velocity_x = 0
        self.velocity_y = 0
        self.velocity_angular = 0

class MotorController:
    def __init__(self):
        self.port = ""

    def initHandshake(self,port=""):
        try:
            if port=="":
                self.port=self.scan_devices()[0]
            self.serial = serial.Serial(self.port, BAUD_RATE, timeout=TIMEOUT)
            return True
        except Exception as e:
            print("Error opening serial port: ", e)
            return False
        
    @staticmethod
    def scan_devices():
        """Scan for available ports."""
        ports = list_ports.comports()
        return [port.device for port in ports]
        
    def shutdown(self):
        if self.serial.is_open:
            self.send_velocity_command(0, 0)
            self.serial.close()
        else:
            print("[motorComms] Error closing serial port!")

    def calculate_receive_checksum(self,data):
        """Calculate checksum for received data."""
        return sum(data[2:50]) % 256  

    def calculate_transmit_checksum(self,data):
        """Calculate checksum for data to be transmitted."""
        return sum(data[3:11]) % 256

    def send_velocity_command(self,velocity_x, velocity_angular):
        """Send velocity command to turtlebot."""
        data = bytearray(SIZE_OF_RX_DATA)
        data[0:2] = HEADER, HEADER
        data[2] = CONTROL_MODE

        # Packing velocity into 4 bytes
        data[3:7] = struct.pack('f', velocity_x)  # velocity x
        data[7:11] = struct.pack('f', velocity_angular) #velocity angular z
        checksum = self.calculate_transmit_checksum(data)
        data[SIZE_OF_RX_DATA - 2] = checksum
        data[SIZE_OF_RX_DATA - 1] = TAIL
        self.serial.write(data)
        self.serial.flush() #flush data stream

    def read_serial_data(self,motorMessage):
        """Read data from serial and process it."""
        if self.serial.in_waiting > 0:
            received_data = self.serial.read(self.serial.in_waiting)
            if len(received_data) >= 52 and received_data[0:2] == bytes([HEADER, HEADER]):
                if self.calculate_receive_checksum(received_data) == received_data[50]:
                    unpacked_data = struct.unpack('f'*6, received_data[2:26])
                    #print(unpacked_data)
                    motorMessage.position_x = unpacked_data[0]
                    motorMessage.position_y = unpacked_data[1]
                    motorMessage.position_angular = unpacked_data[2]
                    motorMessage.velocity_x = unpacked_data[3]
                    motorMessage.velocity_y = unpacked_data[4]
                    motorMessage.velocity_angular = unpacked_data[5]
                else:
                    print("[motorComms] Checksum mismatch")
            else:
                print("[motorComms] Header mismatch")

def main():
    """Main function to control omni-wheel car."""
    esp = MotorController()
    motorMessage = MotorMessage()
    ports = esp.scan_devices()
    print("select port (0-n): %s",ports)
    portSelect = int(input())
    esp.port = ports[portSelect]
    connect_bool = esp.initHandshake(esp.port)
    if not connect_bool: 
        print("[motorComms] Error opening serial port!")
    else: 
        print("[motorComms] Serial port opened successfully!")
    print(f"[motorComms] Using port: {esp.port}")
    try:
        while True:
            speed = float(input("Enter speed: "))
            if type(speed) != float:
                print("[motorComms] Invalid input! Please enter a float.")
                continue
            #if q inputted exit task
            if (speed == "q" or speed == "Q"):
                print("[motorComms] Program stopped by user")
                break
            esp.send_velocity_command(0, speed)
            esp.read_serial_data(motorMessage)
            print(f"[motorComms] Linear Vel: {motorMessage.velocity_x}")
            print(f"[motorComms] Angular Vel: {motorMessage.velocity_angular}\n")
    except KeyboardInterrupt:
        esp.shutdown()
        print("[motorComms] Program stopped by user")
    finally:
        esp.shutdown()

if __name__ == "__main__":
    main()

