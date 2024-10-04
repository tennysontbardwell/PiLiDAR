'''
using mpu6050 lib by https://github.com/OmidAlekasir/mpu6050
'''

import threading
from mpu6050 import MPU6050
from lib.config import format_value
import time

i2c_bus = 3
device_address = 0x68
freq_divider = 0x04

# Make an MPU6050
mpu = MPU6050(i2c_bus, device_address, freq_divider)

# Initiate your DMP
mpu.dmp_initialize()
mpu.set_DMP_enabled(True)

packet_size = mpu.DMP_get_FIFO_packet_size()
FIFO_buffer = [0]*64

# Shared data structure for roll, pitch, yaw
mpu_data = {'roll': 0, 'pitch': 0, 'yaw': 0}

def read_mpu6050():
    while True: # infinite loop
        try:
            if mpu.isreadyFIFO(packet_size): # Check if FIFO data are ready to use...
                FIFO_buffer = mpu.get_FIFO_bytes(packet_size) # get all the DMP data here
                
                q = mpu.DMP_get_quaternion_int16(FIFO_buffer)
                grav = mpu.DMP_get_gravity(q)
                roll_pitch_yaw = mpu.DMP_get_euler_roll_pitch_yaw(q)
                
                mpu_data['roll'] = roll_pitch_yaw.x
                mpu_data['pitch'] = roll_pitch_yaw.y
                mpu_data['yaw'] = roll_pitch_yaw.z
                time.sleep(0.01)

        except OSError as e:
            #print(f"I2C communication error: {e}")
            time.sleep(0.1)  # Wait before retrying


mpu_thread = threading.Thread(target=read_mpu6050)
mpu_thread.daemon = True
mpu_thread.start()

if __name__ == '__main__':
    while True:
        print(f'\rroll: {format_value(mpu_data["roll"],2)} pitch: {format_value(mpu_data["pitch"],2)} yaw: {format_value(mpu_data["yaw"],2)}', end='')
