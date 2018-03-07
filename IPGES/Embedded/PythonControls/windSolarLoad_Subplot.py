
import os
import serial
from serialPort import serial_ports
import pandas as pd
import numpy as np
from matplotlib import pyplot
#import spidev
import time
import math

s_dc = 0
l_dc = 0

#Choose Port
'''
print("These are all the available ports:")
print(serial_ports())
portNum = input("Choose a port: ")
print("You chose: ", portNum)
ser = serial.Serial(port=portNum, baudrate=115200, timeout=10) #need to set time
ser.flushInput()
ser.flushOutput()'''


#Set entries per day -- we only simulate one day (could change if you increase this)
#WAS       144 entries per day @ 1 per 10 minutes
#Currently 288 Entries per day @ 1 per 5  minutes
entries_per_day = 288

#Set excel -- df = data file
file = 'C:/Users/Braden/Documents/SeniorDesign/Control_System/IPGES/Embedded/PythonControls/Solar_Wind_Load_Data/SWL_Data.xlsx'
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
load_input = df["Load_in"]

# We will only get data from [day, day + entries per day]
# Set up arrays to feed to generators / plot
start_point = 0*entries_per_day
wind_output = [1.0] * entries_per_day
solar_output = [1.0] * entries_per_day
load_output = [1.0] * entries_per_day
time_array = [1.0] * entries_per_day

'''#SPI Setup
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 976000'''

#Calculate wind scale factor for duty cycle in the selected day
#Caps the solar duty cycle to .95
scale_factor = 0
maxOutput = 0
maxSolar = 0
maxLoad = 0
for i in range(0, entries_per_day):
    time_array[i] = float(hour[i]) + float(minute[i])/60
    wind_output[i] = wind_input[start_point + i]
    solar_output[i] = solar_input[start_point + i]
    load_output[i] = load_input[start_point + i]
    if maxOutput < wind_input[start_point + i]:
       maxOutput = wind_input[start_point + i]
    if maxSolar < solar_input[start_point + i]:
       maxSolar = solar_input[start_point + i]
    if maxLoad < load_input[start_point + i]:
       maxLoad = load_input[start_point + i]
scale_factor = maxOutput * 1.05

print("Before plot")
# #Set up plot
f, (total_plot, wind_plot, solar_plot, load_plot) = pyplot.subplots(4, sharey=True)
total_plot.plot(time_array, wind_output, label='wind', c='b')
total_plot.plot(time_array, solar_output, label='solar', c='r')
total_plot.plot(time_array, load_output, label='load', c='k')
wind_plot.plot(time_array, wind_output, label='wind', c='b')
solar_plot.plot(time_array, solar_output, label='solar', c='r')
load_plot.plot(time_array, load_output, label='load', c='k')
total_plot.legend()
wind_plot.set_ylabel('Megawatts')
wind_plot.legend()
solar_plot.set_ylabel('SPI Input')
solar_plot.legend()
load_plot.set_ylabel('Duty Cycle')
load_plot.legend()
pyplot.ion()

'''
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
pyplot.title(plot_title.format('June', '26'))
pyplot.grid()
pyplot.ion()
pyplot.show()
pyplot.draw()
pyplot.pause(0.01)
print("After plot")'''


#Write to potentiometer for Solar Output
def write_pot(Pot):
    Pot = 128 - Pot
    msb = Pot >> 8
    lsb = Pot & 0xFF
    str1 = ('SPI ' + str([msb, lsb]))
    ser.write(str1.encode())
    #spi.xfer([msb, lsb])


#write dc to tm4c -- replaces adjust_dc
def write_dc(cur, next, dest, wait):
    increment = (1) if (next > cur) else (-1)
    for x in range(cur, next + increment, increment):
        print(dest + " Duty cycle changing: Currently:  ", x)
        temp = x
        if temp < 10:
            temp = '0' + str(x)
        str1 = (dest + str(temp) + '\n')
        ser.write(str1.encode())
        time.sleep(wait)
    return next


#Main Loop
#if (ready == "y"):
windWait = .3
loadWait = .1
wind_dc = 0
load_dc = 0
windDest = 'PWM '
loadDest = 'Load '
#write_dc(0, 0, windDest, .01)
#write_dc(0, 0, loadDest, .01)


try:
    for i in range(0, entries_per_day):
        #pyplot.scatter(time_array[i], wind_output[i], c='b')
        #pyplot.scatter(time_array[i], solar_output[i], c='r')
        #total_plot.scatter(time_array[i], wind_output[i], c='b')
        #total_plot.scatter(time_array[i], solar_output[i], c='r')
        #total_plot.scatter(time_array[i], load_output[i], c='k')
        wind_plot.scatter(time_array[i], wind_output[i], c='b')
        solar_plot.scatter(time_array[i], solar_output[i], c='r')
        load_plot.scatter(time_array[i], load_output[i], c='k')
        pyplot.draw()
        pyplot.pause(0.01)
        next_Wind = int(math.floor(100*(wind_output[i]/scale_factor)))    #gets duty cycle
        next_Load = int(load_output[i])
        #wind_dc = write_dc(wind_dc, next_Wind, windDest, windWait)
        #load_dc = write_dc(load_dc, next_Load, loadDest, loadWait)
        ##adjust_dc(next_dc)
        solar_spi = (int(round(solar_SPI[start_point + i])))
        print("Wind input:      ", wind_output[i], "  Solar input: ", solar_output[i])
        print("Wind duty cycle: ", next_Wind, "      Solar SPI:   ", solar_spi)
        #write_pot(int(round(solar_SPI[start_point + i])))
        '''
        print("Current Time: ", time_array[i])
        print("Wind Output: ", wind_output[i])
        print("Current load duty cycle: ", load_dc)
        print("Current SPI: " + str(int(round(solar_SPI[start_point + i]))))
        print('')
        time.sleep(1)'''
except KeyboardInterrupt:
    pass

write_dc(0)
#p.stop()
#GPIO.cleanup()







