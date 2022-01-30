from time import sleep
from tkinter import *
import tkinter.messagebox
import webbrowser
import serial
import os
from tkinter import filedialog
import sys
import glob
import serial.tools.list_ports
from datetime import datetime

import requests
from tkinter import *
import tkinter.messagebox
from PIL import Image, ImageTk
import tkinter.filedialog
import ctypes  # An included library with Python install.

def Mbox(title, text, style):
    return ctypes.windll.user32.MessageBoxW(0, text, title, style)
##  Styles:
##  0 : OK
##  1 : OK | Cancel
##  2 : Abort | Retry | Ignore
##  3 : Yes | No | Cancel
##  4 : Yes | No
##  5 : Retry | Cancel
##  6 : Cancel | Try Again | Continue

def serial_ports():
    """ Lists serial port names
        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass

    return result

def processPORT():
    global hubPort, dbFile
    print("Looking for active com ports")
    print(serial_ports())
    serialPorts = serial_ports()
    if (len(serialPorts) == 0):
        print("No Active Com Ports to check")
        return False
    if (len(serialPorts) == 1):
        i = serialPorts[0]
    else:
        serialPorts = [input("Select Port Number, e.g. COM12? ")]
        print("Checking " + serialPorts[0])
    for i in serialPorts:
        print(f"Found Serial port {i}")
        print("Select file to save to in filee open dialogue window")
        ser1 = serial.Serial(str(i), 115200, timeout=2)
        checked=0
        root = Tk()
        root.filename = filedialog.asksaveasfilename(initialdir="/", title="Select file",
                                                     filetypes=(("text files", "*.txt"), ("all files", "*.*")))
        print(root.filename)
        dbFile = open(root.filename, 'w')
        sentence = input("Input single sentence file description, e.g. First test drop with OR <cr> for a blank line ")
        dbFile.write(sentence)
        dbFile.write("\n")
        print("\nProgram will end when <Resetting Trigger> is received from CAP (200ms after trigger)")
        while(True):
            # ser1.write('s'.encode())
            serialdata = str(ser1.readline())
            print((serialdata.replace("b\'","")).replace("\\r\\n\'",""))
            dateTimeObj = datetime.now()
            dbFile.write(str(dateTimeObj.year))
            dbFile.write('/')
            dbFile.write(str(dateTimeObj.month))
            dbFile.write('/')
            dbFile.write(str(dateTimeObj.day))
            dbFile.write(',')
            dbFile.write(str(dateTimeObj.hour))
            dbFile.write(':')
            dbFile.write(str(dateTimeObj.minute))
            dbFile.write(':')
            dbFile.write(str(dateTimeObj.second))
            #dbFile.write('.')
            #dbFile.write(str(dateTimeObj.microsecond))
            dbFile.write(',')
            dbFile.write((serialdata.replace("b\'","")).replace("\\r\\n\'",""))
            dbFile.write("\n")
            dbFile.flush()

            if (len(serialdata) == 0):
                print("No Data received")
            if ("Resetting Trigger" in serialdata):
                print(f"Closing connection to port {i}")
                ser1.close()
                dbFile.close()
                # Initialise hub image to nohub
                exit(1)
            checked = checked+1
    return False

processPORT()
dbFile.close()



