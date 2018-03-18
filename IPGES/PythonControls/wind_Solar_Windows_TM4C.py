
import os
import serial
from serialPort import serial_ports
import pandas as pd
import numpy as np
from matplotlib import pyplot
#import spidev
import time
import math
'''import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT)'''

'''#Raspberry Pi PWM Pin setup
p = GPIO.PWM(12, 50)    #channel 12, frequency 50Hz
p.start(0)'''
dc = 0

#Choose Port
print("These are all the available ports:")
print(serial_ports())
portNum = input("Choose a port: ")
print("You chose: ", portNum)
ser = serial.Serial(port=portNum, baudrate=115200, timeout=10) #need to set time
ser.flushInput()
ser.flushOutput()


#Set entries per day -- we only simulate one day (could change if you increase this)
#WAS       144 entries per day @ 1 per 10 minutes
#Currently 288 Entries per day @ 1 per 5  minutes
entries_per_day = 288
m = input("Enter Month (1-12): ")
d = int(input("Enter Day (1-30): "))

#Set excel -- df = data file
#file = '/Users/bstotmeister/Documents/Sp18/Senior Design/Arduino/Solar_Insolation_1_test.xlsx'
# Use m in the filename so we don't load everything -- just the month
# Hopefully this solution doesn't take minutes to read in
#file = '/home/pi/Documents/Full_2006_Wind_Solar_Data/' + str(m) +  '_Data.xlsx'
file = 'C:/Users/Braden/SeniorDesign/PythonSimulation/PythonSimulation/Full_2006_Wind_Solar_Data/' + str(m) +  '_Data.xlsx'
df = pd.read_excel(file)

#Set up arrays from excel file
month = df["Month"]
day = df["Day"]
year = df["Year"]
hour = df["Hour"]
minute = df["Minute"]
wind_input = df["Wind Output (MW)"]
solar_input = df["Solar Output"]
solar_SPI = df["SPI"]

# We will only get data from [day, day + entries per day]
# Arrays start at 0 ;) (but these are actually lists)
# Set up arrays to feed to generators / plot
start_point = (d-1)*entries_per_day
wind_output = [1.0] * entries_per_day
solar_output = [1.0] * entries_per_day
time_array = [1.0] * entries_per_day

'''#SPI Setup
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 976000'''

#Calculate wind scale factor for duty cycle in the selected day
#Caps the duty cycle to .95
scale_factor = 0
maxOutput = 0
maxSolar = 0
for i in range(0, entries_per_day):
    time_array[i] = float(hour[i]) + float(minute[i])/60
    wind_output[i] = wind_input[start_point + i]
    solar_output[i] = solar_input[start_point + i]
    if maxOutput < wind_input[start_point + i]:
       maxOutput = wind_input[start_point + i]
    if maxSolar < solar_input[start_point + i]:
       maxSolar = solar_input[start_point + i]
scale_factor = maxOutput * 1.05

print("Before plot")
# #Set up plot
pyplot.figure()
windplot = pyplot.plot(time_array, wind_output, label='wind', c='b')
solarplot = pyplot.plot(time_array, solar_output, label='solar', c='r')
pyplot.axis([0.0,24.0,-2, maxSolar*1.1])
pyplot.ylabel('Megawatts')
pyplot.xlabel('Time of Day (Hours)')
pyplot.legend()
#pyplot.legend([windplot, solarplot], ['Wind', 'Solar'])
#pyplot.legend(handles=windplot)
plot_title = 'Daily Renewables Output on {}/{}/2006'
pyplot.title(plot_title.format(m,d))
pyplot.grid()
pyplot.ion()
pyplot.show()
pyplot.draw()
pyplot.pause(0.01)
print("After plot")

#Write to potentiometer for Solar Output
def write_pot(Pot):
    Pot = 128 - Pot
    msb = Pot >> 8
    lsb = Pot & 0xFF
    spi.xfer([msb, lsb])


#Increase/Decrease duty cycle one at a time
def adjust_dc(val):
    global dc
    increment = (1) if (val > dc) else (-1)     #this is python's nasty ternary if
    #Run from current duty cycle to desired duty cycle in increments of +-1
    for x in range(dc, val + increment, increment):
        #p.ChangeDutyCycle(x)
        time.sleep(0.3)
    dc = val

#write dc to tm4c -- replaces adjust_dc
def write_dc(val):
    global dc
    increment = (1) if (val > dc) else (-1)
    for x in range(dc, val + increment, increment):
        print("Duty cycle changing: Currently:  ", x)
        temp = x
        if temp < 10:
            temp = '0' + str(x)
        str1 = ('PWM ' + str(temp) + '\n')
        ser.write(str1.encode())
        #time.sleep(0.3)
    dc = val

trash = input("Waiting for any input to begin...")
#Main Loop
#if (ready == "y"):

try:
    for i in range(0, entries_per_day):
        pyplot.scatter(time_array[i], wind_output[i], c='b')
        pyplot.scatter(time_array[i], solar_output[i], c='r')
        print("Wind input: ", wind_output[i], "Solar input: ", solar_output[i])
        pyplot.draw()
        pyplot.pause(0.01)
        next_dc = int(math.floor(100*(wind_output[i]/scale_factor)))    #gets duty cycle
        write_dc(next_dc)
        ##adjust_dc(next_dc)
        #write_pot(int(round(solar_SPI[start_point + i])))
        '''
        print("Current Time: ", time_array[i])
        print("Wind Output: ", wind_output[i])
        #print("Current duty cycle: " + str(dc))
        print("Current SPI: " + str(int(round(solar_SPI[start_point + i]))))
        print('')
        time.sleep(1)'''
except KeyboardInterrupt:
    pass

write_dc(0)
#p.stop()
#GPIO.cleanup()

# #Set up plot
# pyplot.figure()
# windplot = pyplot.plot(time, wind_MW, label='wind')
# timeplot = pyplot.plot(time, time, label='time')
# pyplot.axis([0,24,-2, maxOutput*1.1])
# pyplot.ylabel('Megawatts')
# pyplot.xlabel('Time of Day (Hours)')
# pyplot.legend(handles=timeplot)
# plot_title = 'Daily Renewables Output on {}/{}/2006'
# pyplot.title(plot_title.format(m,d))
# pyplot.grid()
# pyplot.show()






