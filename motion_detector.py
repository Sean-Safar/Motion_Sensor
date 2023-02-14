# import board
# import busio
# import adafruit_mpu6050
# 
# i2c = busio.I2C(board.SCL, board.SDA)
# mpu = adafruit_mpu6050.MPU6050(is2)
# 
# while True:
#     print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2"%(mpu.acceleration))
#     print("Gyro X:%.2f, Y: %.2f, Z: %.2f degrees/s"%(mpu.gyro))
#     print("Temperature: %.2f C"%mpu.temperature)
#     print("")
#     time.sleep(1)

import machine
from machine import Pin, Timer, I2C
import esp32
import network
import socket
import urequests
import json
import ustruct
from neopixel import NeoPixel

red_led = Pin(13, Pin.OUT)
red_led.value(1)
power_led = Pin(2, Pin.OUT)
power_led.value(1)
neo_pin = Pin(0, Pin.OUT)
np = NeoPixel(neo_pin,8)
np[0] = (0,0,0)
np.write()
tim_1 = Timer(0)
tim_2 = Timer(1)
tim_3 = Timer(2)
flag = 0


#Credits: Andrew Lynn (former ECE40862 student) and Vijay R.
class MPU:
    # Static MPU memory addresses
    ACC_X = 0x3B
    ACC_Y = 0x3D
    ACC_Z = 0x3F
    TEMP = 0x41
    GYRO_X = 0x43
    GYRO_Y = 0x45
    GYRO_Z = 0x47

    def acceleration(self):
        self.i2c.start()
        acc_x = self.i2c.readfrom_mem(self.addr, MPU.ACC_X, 2)
        acc_y = self.i2c.readfrom_mem(self.addr, MPU.ACC_Y, 2)
        acc_z = self.i2c.readfrom_mem(self.addr, MPU.ACC_Z, 2)
        self.i2c.stop()

        # Accelerometer by default is set to 2g sensitivity setting
        # 1g = 9.81 m/s^2 = 16384 according to mpu datasheet
        acc_x = self.__bytes_to_int(acc_x) / 16384 * 9.81
        acc_y = self.__bytes_to_int(acc_y) / 16384 * 9.81
        acc_z = self.__bytes_to_int(acc_z) / 16384 * 9.81
        
        return acc_x, acc_y, acc_z

    def temperature(self):
        self.i2c.start()
        temp = self.i2c.readfrom_mem(self.addr, self.TEMP, 2)
        self.i2c.stop()
        temp = self.__bytes_to_int(temp)
        return self.__celsius_to_fahrenheit(temp / 340 + 36.53)

    @staticmethod
    def __celsius_to_fahrenheit(temp):
        return temp * 9 / 5 + 32
    
    @staticmethod
    def __bytes_to_int(data):
        # Int range of any register: [-32768, +32767]
        # Must determine signing of int
        if not data[0] & 0x80:
            return data[0] << 8 | data[1]
        return -(((data[0] ^ 0xFF) << 8) | (data[1] ^ 0xFF) + 1)

    def __init__(self, i2c):
        # Init MPU
        self.i2c = i2c
        self.addr = i2c.scan()[0]
        self.i2c.start()
        self.i2c.writeto(0x68, bytearray([107,0]))
        self.i2c.stop()
        print('Initialized MPU6050.')

    # Gyro values will be updated every 100ms after creation of MPU object
        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        self.pitch_offset = 0
        self.roll_offset = 0
        self.yaw_offset = 0

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    ssid = ""
    pswrd = ""
    if wlan.isconnected():
        wlan.disconnect
    if not wlan.isconnected():
        wlan.connect(ssid,pswrd)
    while not wlan.isconnected():
        pass
    print("Connected to ")
    print("IP Address: {}".format(wlan.ifconfig()[0]))
    
def mean():
    global m_x, m_y, m_z
    m_x = 0
    m_y = 0
    m_z = 0
    
    for s in range(50):
        x,y,z = mpu.acceleration()
        m_x = m_x + x
        m_y = m_y + y
        m_z = m_z + z
    m_x /= 50
    m_y /= 50
    m_z /= 50
    m_x = m_x
    m_y = m_y
    m_z = m_z
    
    
    print('Calibration complete!')
    print('Averages: x{} | y{} | z{}'.format(m_x,m_y,m_z))

def thinkspeak(self):
    global flag
    response = urequests.get("https://api.thingspeak.com/channels/1984999/fields/1.json?api_key=S5LG1EMRM02MNMWH&results=2")
    parsed = response.json()
    state = parsed['feeds'][-1]['field1']
    
    print(state)
    if state == '1':
        flag = 1    
    elif state == '0':
        flag = 0
    
def sensor_check(self):
    global real_x, real_y, real_z
    real_x = 0
    real_y = 0
    real_z = 0
    
    if flag == 1:
        x,y,z = mpu.acceleration()
        real_x = x - m_x
        real_y = y - m_y
        real_z = z - m_z

def motion_activate(self):
    if flag == 1:
        
        #if abs(real_x) > 0.15 or abs(real_y) > 0.15 or abs(real_z) > 0.15:
        if red_led.value() == 1:
            
            data = {"value1": real_x , "value2": real_y , "value3": real_z}
            urequests.post("https://maker.ifttt.com/trigger/motion_detected/with/key/dfUjOL8RNfNrp0Dz88hmiB", json=data)

i2c = I2C(scl=Pin(14), sda=Pin(22))
mpu = MPU(i2c)

print(mpu.acceleration())
print(mpu.temperature())

if __name__ == "__main__":
    connect_wifi()
    mean()
    
    tim_1.init(mode=Timer.PERIODIC, period=100, callback=sensor_check)
    tim_2.init(mode=Timer.PERIODIC, period=30000, callback=thinkspeak)
    tim_3.init(mode=Timer.PERIODIC, period=1000, callback=motion_activate)
    
    while(True):
        
        if flag == 1:
            
            np[0] = (0,50,0)
            np.write()
            
            while abs(real_x) > 0.15 or abs(real_y) > 0.15 or abs(real_z) > 0.15:
                
                red_led.value(1)
            
            red_led.value(0)
            
        elif flag == 0:
            
            np[0] = (0,0,0)
            np.write()
            red_led.value(0)
