import time
import threading
from mpu6050 import MPU6050

class MPU6050Wrapper:
    def __init__(self, i2c_bus=3, device_address=0x68, freq_divider=0x04):
        self.mpu = MPU6050(i2c_bus, device_address, freq_divider)
        self.mpu.dmp_initialize()
        self.mpu.set_DMP_enabled(True)
        
        self.packet_size = self.mpu.DMP_get_FIFO_packet_size()
        self.FIFO_buffer = [0]*64
        self.mpu_data = {'roll': 0, 'pitch': 0, 'yaw': 0}
        
        self.thread = threading.Thread(target=self._read_mpu6050)
        self.thread.daemon = True
        self.thread.start()
    
    def _read_mpu6050(self):
        while True:
            try:
                if self.mpu.isreadyFIFO(self.packet_size):
                    self.FIFO_buffer = self.mpu.get_FIFO_bytes(self.packet_size)
                    
                    q = self.mpu.DMP_get_quaternion_int16(self.FIFO_buffer)
                    grav = self.mpu.DMP_get_gravity(q)
                    roll_pitch_yaw = self.mpu.DMP_get_euler_roll_pitch_yaw(q)
                    
                    self.mpu_data['roll'] = roll_pitch_yaw.x
                    self.mpu_data['pitch'] = roll_pitch_yaw.y
                    self.mpu_data['yaw'] = roll_pitch_yaw.z
                    time.sleep(0.02)
            except OSError as e:
                time.sleep(0.05)
    
    def get_euler_angles(self):
        return self.mpu_data


if __name__ == '__main__':
    from config import format_value

    mpu_wrapper = MPU6050Wrapper()
    while True:
        angles = mpu_wrapper.get_euler_angles()
        print(f'\rroll: {format_value(angles["roll"], 2)} pitch: {format_value(angles["pitch"], 2)} yaw: {format_value(angles["yaw"], 2)}', end='')
        time.sleep(0.1)
