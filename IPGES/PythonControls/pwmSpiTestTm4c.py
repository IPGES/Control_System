#import spidev
import math
import time
import serial
from serialPort import serial_ports
'''import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT)'''

#Choose Port
print("These are all the available ports:")
print(serial_ports())
portNum = input("Choose a port: ")
print("You chose: ", portNum)
ser = serial.Serial(port=portNum, baudrate=115200, timeout=10) #need to set time
ser.flushInput()
ser.flushOutput()


#p = GPIO.PWM(12, 50)    #channel 12, frequency 50Hz
#p = GPIO.PWM(12, 5000)
#p.start(0)

#SPI Setup
'''
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 976000'''

#x = input("Enter a duty cycle: ")
dc = 2
#p.ChangeDutyCycle(0)
#Increase/Decrease duty cycle one at a time
def adjust_dc(val):
    global dc
    increment = (1) if (val > dc) else (-1)     #this is python's nasty ternary operator
    #Run from current duty cycle to desired duty cycle in increments of +-1
    for x in range(dc, val + increment, increment):
        print(x)
        p.ChangeDutyCycle(x)
        time.sleep(0.1)
    dc = val

#Write to potentiometer for Solar Output
def write_pot(Pot):
    msb = Pot >> 8
    lsb = Pot & 0xFF
    spi.xfer([msb, lsb])

#Write SPI to tm4c -- replaces write_pot
def write_spi(val):
    temp = str(val)
    if val < 10:
        temp = '00' + str(temp)
        print("Temp1: ", temp)
    if ((val >= 10) & (val <= 99)):
        temp = '0' + str(temp)
        print("Temp2: ", temp)
    print("SPI value being written: ", temp)
    str1 = ('PV ' + str(temp) + '\n')
    ser.write(str1.encode())

def write_dc(val):
    global dc
    '''increment = (1) if (val > dc) else (-1)
    for x in range(dc, val + increment, increment):
        print("Duty cycle changing: Currently:  ", x)
        temp = x
        if temp < 10:
            temp = '0' + str(x)
        str1 = ('PWM ' + str(temp) + '\n')
        ser.write(str1.encode())
        time.sleep(0.3)
    dc = val'''
    if val >= 100:
        val = 99
    if val <= 0:
        val = 1
    increment = (1) if (val > dc) else (-1)
    for x in range(dc, val + increment, increment):
        print("Duty cycle changing: Currently:  ", x)
        temp = str(x)
        if x< 10:
            temp = '0' + str(x)
        temp = '0' + temp
        str1 = ('Wind ' + str(temp) + '\n')
        print(str1, '\n')
        ser.write(str1.encode())
        #time.sleep(1)#for islanded
        time.sleep(.3)
    dc = val

try:
        while 1:
                #SPI Code - next 6 lines
                '''x = int(input("Enter an SPI (between 0 and 180 typically): "))
                print("Current SPI: " + str(int(x)))
                write_spi(int(round(x)))
                print('before SPI sleep')
                time.sleep(1)
                print('after sleep')'''
                #Duty cycle code - next two lines
                x = input("Enter a duty cycle: ")
                write_dc(int(x))
                #adjust_dc(int(x))

except KeyboardInterrupt:
        pass
adjust_dc(2)
p.stop()
GPIO.cleanup()
