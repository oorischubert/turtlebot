import serial
import time
import struct

# Constants for communication protocol
HEADER = 200
TAIL = 199
SIZE_OF_RX_DATA = 17
VELOCITY_MODE = 1  

# Serial port configuration
SERIAL_PORT = '/dev/tty.usbserial-022EE911' #not always the same!
BAUD_RATE = 115200
TIMEOUT = 1

class MotorMessage:
    def __init__(self):
        self.position_x = 0
        self.position_y = 0
        self.position_angular = 0
        self.velocity_x = 0
        self.velocity_y = 0
        self.velocity_angular = 0

class MotorController:
    def __init__(self,port):
        self.port = port

    def initHandshake(self):
        try:
            self.serial = serial.Serial(self.port, BAUD_RATE, timeout=TIMEOUT)
            return True
        except Exception as e:
            #print("Error opening serial port: ", e)
            return False
        
    def shutdown(self):
        self.send_velocity_command(0, 0)
        self.serial.close()

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
        data[2] = VELOCITY_MODE

        # Packing velocity into 4 bytes
        data[3:7] = struct.pack('f', velocity_x)  # velocity x
        # data[7:11] = velocity_bytes # velocity y //not neccesary for diff-drive
        data[11:15] = struct.pack('f', velocity_angular) #velocity angular z
        checksum = self.calculate_transmit_checksum(data)
        data[SIZE_OF_RX_DATA - 2] = checksum
        data[SIZE_OF_RX_DATA - 1] = TAIL
        self.serial.write(data)

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
                    print("Checksum mismatch")
            else:
                print("Header mismatch")

def main():
    """Main function to control omni-wheel car."""
    esp = MotorController('/dev/tty.usbserial-022EE911')
    motorMessage = MotorMessage()
    connect_bool = esp.initHandshake()
    if not connect_bool: 
        print("Error opening serial port!")
    else: 
        print("Serial port opened successfully!")
    try:
        while True:
            for i in range(10):
                start_time = time.time()
                while time.time() - start_time < 10:
                    esp.send_velocity_command(i * 0.05, 0)
                    esp.read_serial_data(motorMessage)
                    print(f"Linear Vel: {motorMessage.velocity_x}")
                    print(f"Angular Vel: {motorMessage.velocity_angular}\n")
                    time.sleep(0.1)
    except KeyboardInterrupt:
        esp.shutdown()
        print("Program stopped by user")
    finally:
        esp.shutdown()

if __name__ == "__main__":
    main()

