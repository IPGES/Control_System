import serial
import re
import datetime
from serialPort import serial_ports #check all ports
from requestExpress import post_to_express

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

'''
while True:
    for bit in range(0, 1):
        tm4cIn = ser.readline()
        print(str(tm4cIn))
        #print(str(tm4cIn).split('b\'')[1])
        #parsedTm4c = tm4cIn.rsplit('b\'')[1].rsplit('\r\n')[0]
        print(tm4cIn)
'''

while True:
    while True:
        tm4cIn = ser.readline() #comes in as bytes and has b' as a header
        #print(str(tm4cIn))
        parsedTm4c = str(tm4cIn).rsplit('b\'')[1].rsplit('\\r\\n')[0]
        print(parsedTm4c)
        timeRecieved = datetime.datetime.now()
        timeValue = timeRecieved.hour * 100 + timeRecieved.minute
        pvValue = parsedTm4c.split("\"pv\" : ")[1].split(',')[0]
        inverterValue = parsedTm4c.split("\"inverter\" : ")[1].split(',')[0]
        windValue = parsedTm4c.split("\"wind\" : ")[1].split(',')[0]
        gridValue = parsedTm4c.split("\"grid\" : ")[1].split(',')[0]
        loadValue = parsedTm4c.split("\"load\" : ")[1].split(',')[0]
        print(timeValue, " " ,pvValue, " ", pvValue, " ", inverterValue, " ", windValue, " ", gridValue, " ", loadValue) 
        if(parsedTm4c[0] == '@'):
            post_to_express(timeValue, pvValue, inverterValue, inverterValue, gridValue, loadValue)
            break
    print("Done")
ser.close()
