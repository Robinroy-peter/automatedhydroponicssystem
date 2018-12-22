#!/usr/bin/python
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
#import cv2
import RPi.GPIO as GPIO
import Adafruit_DHT
import os
import smbus
import time
from time import sleep 

# Set sensor type : Options are DHT11,DHT22 or AM2302


# Load Raspberry Pi Drivers
os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')
DEVICE     = 0x23 # Default device I2C address

POWER_DOWN = 0x00 # No active state
POWER_ON   = 0x01 # Power on
RESET      = 0x07 # Reset data register value

# Start measurement at 4lx resolution. Time typically 16ms.
CONTINUOUS_LOW_RES_MODE = 0x13
# Start measurement at 1lx resolution. Time typically 120ms
CONTINUOUS_HIGH_RES_MODE_1 = 0x10
# Start measurement at 0.5lx resolution. Time typically 120ms
CONTINUOUS_HIGH_RES_MODE_2 = 0x11
# Start measurement at 1lx resolution. Time typically 120ms
# Device is automatically set to Power Down after measurement.
ONE_TIME_HIGH_RES_MODE_1 = 0x20
# Start measurement at 0.5lx resolution. Time typically 120ms
# Device is automatically set to Power Down after measurement.
ONE_TIME_HIGH_RES_MODE_2 = 0x21
# Start measurement at 1lx resolution. Time typically 120ms
# Device is automatically set to Power Down after measurement.
ONE_TIME_LOW_RES_MODE = 0x23

#bus = smbus.SMBus(0) # Rev 1 Pi uses 0
bus = smbus.SMBus(1)  # Rev 2 Pi uses 1
# Define data file for temperature sensors
temp_sensor_1 = '/sys/bus/w1/devices/28-0417b14e9fff/w1_slave'
#gpio = 4
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.IN)
GPIO.setup(18, GPIO.OUT)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(26, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
input_value = GPIO.input(17)


def convertToNumber(data):
  	result=(data[1] + (256 * data[0])) / 1.2
  	return (result)

def readLight(addr=DEVICE):
  	data = bus.read_i2c_block_data(addr,ONE_TIME_HIGH_RES_MODE_1)
	return convertToNumber(data)


def readIntensity():
	lightLevel=readLight()
	print("Light Level : " + format(lightLevel,'.2f') + " lx")
 	time.sleep(0.5)

def read_temp_raw(temp_sensor):
#Read the 2 raw lines of data from the temperature sensor
    f = open(temp_sensor, 'r')
    lines = f.readlines()
    f.close()
    return lines


def read_temp(temp_sensor):
# Check the Temp Sensor file for errors and convert to Celcius or Fahrenheit
    lines = read_temp_raw(temp_sensor)
    while lines[0].strip()[-3:] != 'YES':
        sleep(0.2)
        lines = read_temp_raw(temp_sensor)
    temp_result = lines[1].find('t=')
    if temp_result != -1:
        temp_string = lines[1][temp_result + 2:]
        # Use line below for Celsius
        temp = float(temp_string) / 1000.0
        #Uncomment line below for Fahrenheit
        #temp = ((float(temp_string) / 1000.0) * (9.0 / 5.0)) + 32
        return temp

def waterTemp():
	print "liquid Temperature = ", read_temp(temp_sensor_1)

def roomTemp():
	sensor = Adafruit_DHT.DHT11
	humidity, temperature = Adafruit_DHT.read_retry(sensor, 23)
	if humidity is not None and temperature is not None:
		print('room Temp={0:0.1f}*C  Humidity={1:0.1f}%'.format(temperature, humidity))
	else:
		print('Failed to get reading. Try again!')

def camara():
	time.sleep(0.1)
	camera.capture(rawCapture, format="bgr")
	image = rawCapture.array
	#cv2.imwrite('test.png',image)
	#cv2.imshow("Image", image)
	#cv2.waitKey(10)
	print("------------------------------------------\n")

def motor():
	x = int(input("bubbler need to on/off (1/0): ")) 
	if x==1 :
		GPIO.output(18, GPIO.HIGH)
	else:
		GPIO.output(18, GPIO.LOW)
	print("------------------------------------------\n")

def lightmiddle():
	x = int(input("middle light need to on/off (1/0): ")) 
	if x==1 :
		GPIO.output(26, GPIO.HIGH)
	else:
		GPIO.output(26, GPIO.LOW)
	print("------------------------------------------\n")

def lighttop():
	x = int(input("top light need to on/off (1/0): ")) 
	if x==1 :
		GPIO.output(16, GPIO.HIGH)
	else:
		GPIO.output(16, GPIO.LOW)
	print("------------------------------------------\n")

def lightmain():
	x = int(input("main light need to on/off (1/0): ")) 
	if x==1 :
		GPIO.output(12, GPIO.HIGH)
	else:
		GPIO.output(12, GPIO.LOW)
	print("------------------------------------------\n")

def allsensors():
	roomTemp()
	waterTemp()
	readIntensity()
def alllights():
	x = int(input("all lights need to on/off (1/0): ")) 
	if x==1 :
		GPIO.output(26, GPIO.HIGH)
		GPIO.output(16, GPIO.HIGH)
		GPIO.output(12, GPIO.HIGH)
	else:
		GPIO.output(26, GPIO.LOW)
		GPIO.output(16, GPIO.LOW)
		GPIO.output(12, GPIO.LOW)
	print("------------------------------------------\n")
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
rawCapture = PiRGBArray(camera)

while True:
	print("------------------------------------------")
	print("choose the action need to do\n\n")
	print(" 1 => take image \n")
	print(" 2 => control bubbler\n")
	print(" 3 => read room temperature\n")
	print(" 4 => control middle light\n")
	print(" 5 => control top light\n")
	print(" 6 => read liquid temperature\n")
	print(" 7 => read light intensity\n")
	print(" 8 => control main light\n")
	print(" 9 => read all sensor readings\n")
	print(" 10 => control all lights \n")
	x = int(input("Please enter Action number: ")) 
	if x==1:
		camara()
	elif x==2:
		motor()
	elif x==3:
		roomTemp()
	elif x==4:
		lightmiddle()
	elif x==5:
		lighttop()
	elif x==6:
		waterTemp()
	elif x==7:
		readIntensity()
	elif x==8:
		lightmain()
	elif x==9:
		allsensors()
	elif x==10:
		alllights()
	pass

