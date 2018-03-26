
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
import re
from requestExpress import post_to_express
import threading
import sys

def catch_str(ser):
    global caught_str
    with lock:
        tm4cIn = ser.readline()  # comes in as bytes and has b' as a header
    #print(str(tm4cIn))
    parsedTm4c = str(tm4cIn).rsplit('b\'')[1].rsplit('\\r\\n')[0]
    if (parsedTm4c[0] == '@'):
        caught_str = parsedTm4c
    print('Caught str: ', caught_str)

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
        if x < 10:
            temp = '0' + str(x)
        '''if x < 100:
            temp = '0' + str(temp)'''
        str1 = ('W ' + str(temp) + '\n')
        catch_str(ser)
        with lock:
            ser.write(str1.encode())
        print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA %%%%%%%%%%%%%% CUR VAL: ", temp, "DEST VAL: ", val)
        time.sleep(0.3)
    return val


def write_loop(entries_per_day, wind_output, scale_factor, solar_SPI, start_point, ser):
    dc = 0
    for i in range(0, entries_per_day):
        print('ENTERING LOOP:  @!@#$@!##!#!@#!@#!#@!@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@            -----   ', i)
        next_dc = int(math.floor(100*(wind_output[i]/scale_factor)))    #gets duty cycle
        dc = write_dc(dc, next_dc, ser)
        #write_spi(int(round(solar_SPI[start_point + i])), ser)
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


def toCloud(ser):
    while True:
        while True:
            with lock:
                tm4cIn = ser.readline() #comes in as bytes and has b' as a header
                #print(str(tm4cIn))
                parsedTm4c = str(tm4cIn).rsplit('b\'')[1].rsplit('\\r\\n')[0]
                if(parsedTm4c[0] != '@'):
                    parsedTm4c = caught_str
                #    break;
                #print(parsedTm4c)
                timeRecieved = datetime.datetime.now()
                timeValue = timeRecieved.hour * 100 + timeRecieved.minute
                #try:
                pvValue = parsedTm4c.split("\"pv\" : ")[1].split(',')[0]
                inverterValue = parsedTm4c.split("\"inverter\" : ")[1].split(',')[0]
                windValue = parsedTm4c.split("\"wind\" : ")[1].split(',')[0]
                gridValue = parsedTm4c.split("\"grid\" : ")[1].split(',')[0]
                loadValue = parsedTm4c.split("\"load\" : ")[1].split(',')[0]
                #print(timeValue, " " ,pvValue, " ", pvValue, " ", inverterValue, " ", windValue, " ", gridValue, " ", loadValue)
                if(parsedTm4c[0] == '@'):
                    post_to_express(timeValue, pvValue, inverterValue, inverterValue, gridValue, loadValue)
                    break
                '''except IndexError:
                    print("How did we get here?")
                    print("how _did_ we get here??", caught_str)'''

        print("Done")
    ser.close()

lock = threading.Lock()
caught_str = ''

if __name__ == "__main__":
    URL = "https://damp-gorge-19491.herokuapp.com"
    URL += "/tm4cInput"

    #Choose Port
    print("These are all the available ports:")
    print(serial_ports())
    portNum = input("Choose a port: ")
    print("You chose: ", portNum)


    ser = serial.Serial(port=portNum, baudrate=115200, timeout=10) #need to set time
    ser.flushInput()
    ser.flushOutput()
    try:
        t1 = threading.Thread(target=run, args=(ser,))
        t2 = threading.Thread(target=toCloud, args=(ser,))
        t1.start()
        time.sleep(10)
        t2.start()
    except KeyboardInterrupt:
        print("Interrupted")
        sys.exit(0)


