import struct
import time
import serial


class RoombaController:
    def __init__(self, port, baudrate=115200):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=1)
        print("### STARTING ###")
        self.start()
        # time.sleep(0.1)
        print("### SETTING MODE ###")
        self.set_mode("safe")
        time.sleep(0.1)

    def send_byte(self, byte):
        self.ser.write(byte.to_bytes(1, "big"))

    def send_int16(self, value):
        self.ser.write(value.to_bytes(2, "big", signed=True))

    def start(self):
        self.send_byte(128)  # Send "Start" command (opcode 128)

    def set_mode(self, mode):
        if mode == "safe":
            self.send_byte(131)  # Send "Safe" command (opcode 131)
        elif mode == "full":
            self.send_byte(132)  # Send "Full" command (opcode 132)

    def drive(self, velocity, radius):
        self.send_byte(137)  # Send "Drive" command (opcode 137)
        self.send_int16(velocity)
        self.send_int16(radius)

    def stop(self):
        self.drive(0, 0)

    def turn_off(self):
        self.send_byte(173)  # Send "Stop" command (opcode 173)

    def forward(self, velocity):
        self.drive(velocity, 0)

    def backward(self, velocity):
        self.drive(-velocity, 0)

    def spin_left(self, velocity):
        self.drive(velocity, 1)

    def spin_right(self, velocity):
        self.drive(velocity, -1)

    def close(self):
        self.ser.close()

    
    def get_sensor_data(self, sensor_packet_id):
        self.send_byte(142)  # Send "Sensors" command (opcode 142)
        self.send_byte(sensor_packet_id)

        packet_length = self._get_packet_length(sensor_packet_id)
        sensor_data = self.ser.read(packet_length)
        return self._parse_sensor_data(sensor_packet_id, sensor_data)

    def _get_packet_length(self, sensor_packet_id):
        packet_lengths = {
            7: 1,
            13: 1,
            43: 2,
            # Add more sensor packet IDs and their lengths here
        }
        return packet_lengths.get(sensor_packet_id, 0)

    def _parse_sensor_data(self, sensor_packet_id, sensor_data):
        if sensor_packet_id == 7:  # Bumps and Wheel Drops
            return struct.unpack('B', sensor_data)[0]
        elif sensor_packet_id == 13:  # Virtual Wall
            return struct.unpack('B', sensor_data)[0]
        elif sensor_packet_id == 43:  # Battery Charge
            return struct.unpack('>H', sensor_data)[0]
        # Add more sensor packet IDs and their parsing functions here

        return None
