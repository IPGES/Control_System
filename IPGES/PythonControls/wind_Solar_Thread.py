
import os
import serial
from serialPort import serial_ports
import pandas as pd
import numpy as np
from matplotlib import pyplot
#import spidev
import time
import math
import datetime

#Write SPI to tm4c -- replaces write_pot
def write_spi(val, ser):
    temp = val
    if val < 10:
        temp = '0' + str(temp)
    if (val >= 10) & (val <= 90):
        temp = '0' + str(temp)
    print("SPI value being written: ", temp)
    str1 = ('PP ' + str(temp) + '\n')
    ser.write(str1.encode())

#write dc to tm4c -- replaces adjust_dc
def write_dc(dc, val, ser):
    increment = (1) if (val > dc) else (-1)
    for x in range(dc, val + increment, increment):
        print("Duty cycle changing: Currently:  ", x)
        temp = x
        if temp < 10:
            temp = '0' + str(x)
        str1 = ('WW ' + str(temp) + '\n')
        ser.write(str1.encode())
        time.sleep(0.3)
    return val


def write_loop(entries_per_day, wind_output, scale_factor, solar_SPI, start_point, ser):
    dc = 0
    for i in range(0, entries_per_day):
        next_dc = int(math.floor(100*(wind_output[i]/scale_factor)))    #gets duty cycle
        dc = write_dc(dc, next_dc, ser)
        write_spi(int(round(solar_SPI[start_point + i])), ser)
        '''
        print("Current Time: ", time_array[i])
        print("Wind Output: ", wind_output[i])
        print("Current SPI: " + str(int(round(solar_SPI[start_point + i]))))
        print('')
        '''

def run(ser):
    entries_per_day = 288
    m = input("Enter Month (1-12): ")
    d = int(input("Enter Day (1-30): "))
    file = 'C:/Users/Braden/SeniorDesign/PythonSimulation/PythonSimulation/Full_2006_Wind_Solar_Data/' + str(
        m) + '_Data.xlsx'
    df = pd.read_excel(file)

    # Set up arrays from excel file
    month = df["Month"]
    day = df["Day"]
    year = df["Year"]
    hour = df["Hour"]
    minute = df["Minute"]
    wind_input = df["Wind Output (MW)"]
    solar_input = df["Solar Output"]
    solar_SPI = df["SPI"]

    start_point = (d - 1) * entries_per_day
    wind_output = [1.0] * entries_per_day
    solar_output = [1.0] * entries_per_day
    time_array = [1.0] * entries_per_day

    maxOutput = 0
    maxSolar = 0
    for i in range(0, entries_per_day):
        time_array[i] = float(hour[i]) + float(minute[i]) / 60
        wind_output[i] = wind_input[start_point + i]
        solar_output[i] = solar_input[start_point + i]
        if maxOutput < wind_input[start_point + i]:
            maxOutput = wind_input[start_point + i]
        if maxSolar < solar_input[start_point + i]:
            maxSolar = solar_input[start_point + i]
    scale_factor = maxOutput * 1.05

    print("Initialization complete")
    write_loop(entries_per_day, wind_output, scale_factor, solar_SPI, start_point, ser)




