#page for configs

# Constants for communication protocol
HEADER = 200
TAIL = 199
SIZE_OF_RX_DATA = 53 #output data
SIZE_OF_TX_DATA = 52 #input data
CONTROL_MODE = 1 # 0 = position, 1 = velocity, 2 = odometry

# Serial port configuration
ESP_SERIAL_PORT = '/dev/ttyUSB1' #not always the same! #deprecated
BAUD_RATE = 115200
TIMEOUT = 1