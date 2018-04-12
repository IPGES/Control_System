
import os
import serial
from serialPort import serial_ports
import pandas as pd
import numpy as np
from matplotlib import pyplot
#import spidev
import time
import math

s_dc = 2
l_dc = 2

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
file = 'C:/Users/Braden/Documents/SeniorDesign/Control_System/IPGES/PythonControls/Solar_Wind_Load_Data/SWL_Data.xlsx'
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
#Approx real watt output
wind_watts = [1.0] * entries_per_day
wind_watts_3p = [1.0] * entries_per_day
solar_watts = [1.0] * entries_per_day
load_watts = [1.0] * entries_per_day


#Calculate wind scale factor for duty cycle in the selected day
#Caps the wind duty cycle to .98
maxWindInput = 30
maxWindOutput = 90 #90 Watts per phase
maxSolarOutput = 36 #36 Watts single phase
maxLoadOutput = 180 #180 Watts single phase


for i in range(0, entries_per_day):
    time_array[i] = float(hour[i]) + float(minute[i])/60
    wind_output[i] = 100*wind_input[start_point + i]/maxWindInput
    solar_output[i] = solar_input[start_point + i]
    load_output[i] = load_input[start_point + i]
    wind_watts[i] = float(maxWindOutput/99) * wind_output[i]
    wind_watts_3p[i] = wind_watts[i]*3      # must be balanced so *3 is correct
    solar_watts[i] = float(maxSolarOutput/128) * solar_SPI[start_point + i]*128/209
    load_watts[i] = float(maxLoadOutput/99) * load_output[i]



print("Before plot")
# #Set up plot
f, (total_plot, wind_plot, solar_plot, load_plot) = pyplot.subplots(4, sharey=False, figsize=(15,6))
total_plot.plot(time_array, wind_watts_3p, label='Wind 3 phase', c='g')
total_plot.plot(time_array, wind_watts, label='Wind', c='b')
total_plot.plot(time_array, solar_watts, label='Solar', c='r')
total_plot.plot(time_array, load_watts, label='Load', c='k')
wind_plot.plot(time_array, wind_output, label='Wind', c='b')
solar_plot.plot(time_array, solar_output, label='Solar', c='r')
load_plot.plot(time_array, load_output, label='Load', c='k')
total_plot.set_ylabel('Watts')
total_plot.set_ylim(top=270)
total_plot.legend()
wind_plot.set_ylabel('Duty Cycle')
wind_plot.set_ylim(top=100)
wind_plot.legend()
solar_plot.set_ylabel('Insolation (%)')
solar_plot.set_ylim(top=100)
solar_plot.legend()
load_plot.set_ylabel('Duty Cycle')
load_plot.set_xlabel('Time (Hours)')
load_plot.set_ylim(top=100)
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
#Write SPI to tm4c -- replaces write_pot
def write_spi(val):
    val = int((val*128)/209)
    temp = str(val)
    if val < 10:
        temp = '00' + str(temp)
    if ((val >= 10) & (val <= 90)):
        temp = '0' + str(temp)
    print("SPI value being written: ", temp)
    str1 = ('PV ' + str(temp) + '\n')
    #ser.write(str1.encode())


#write dc to tm4c -- replaces adjust_dc
def write_dc(cur, next, dest, wait):
    if((next < 87) & (dest == 'Wind ')):
        next = 87
    increment = (1) if (next > cur) else (-1)
    for x in range(cur, next + increment, increment):
        print(dest + " Duty cycle changing: Currently:  ", x)
        temp = str(x)
        if x < 10:
            temp = '0' + str(x)
        temp = '0' + temp
        str1 = (dest + str(temp) + '\n')
        #ser.write(str1.encode())
        #time.sleep(wait)
    return next


#Main Loop
#if (ready == "y"):
windWait = .4
loadWait = .1
wind_dc = 2
load_dc = 2
windDest = 'Wind '
loadDest = 'Load '
#write_dc(0, 0, windDest, .01)
#write_dc(0, 0, loadDest, .01)


try:
    for i in range(0, entries_per_day):
        #pyplot.scatter(time_array[i], wind_output[i], c='b')
        #pyplot.scatter(time_array[i], solar_output[i], c='r')
        #wind_watts =
        total_plot.scatter(time_array[i], wind_watts[i], c='b')
        total_plot.scatter(time_array[i], wind_watts_3p[i], c='g')
        total_plot.scatter(time_array[i], solar_watts[i], c='r')
        total_plot.scatter(time_array[i], load_watts[i], c='k')
        wind_plot.scatter(time_array[i], wind_output[i], c='b')
        solar_plot.scatter(time_array[i], solar_output[i], c='r')
        load_plot.scatter(time_array[i], load_output[i], c='k')
        pyplot.draw()
        pyplot.pause(0.01)
        #next_Wind = int(math.floor((wind_output[i]/maxWind)))    #gets duty cycle
        next_Wind = int(wind_output[i])
        next_Load = int(load_output[i])
        wind_dc = write_dc(wind_dc, next_Wind, windDest, windWait)
        load_dc = write_dc(load_dc, next_Load, loadDest, loadWait)
        ##adjust_dc(next_dc)
        solar_spi = (int(round(solar_SPI[start_point + i])))
        print("Wind input:      ", wind_output[i], "  Solar input: ", solar_output[i])
        print("Wind duty cycle: ", next_Wind, "      Solar SPI:   ", solar_spi)
        print("########")
        print("TIME: ", time_array[i])
        print("########")
        write_spi(int(round(solar_SPI[start_point + i])))
        '''
        print("Current Time: ", time_array[i])
        print("Wind Output: ", wind_output[i])
        print("Current load duty cycle: ", load_dc)
        print("Current SPI: " + str(int(round(solar_SPI[start_point + i]))))
        print('')
        time.sleep(1)'''
except KeyboardInterrupt:
    pass
time.sleep(15)
write_dc(2)
#p.stop()
#GPIO.cleanup()







