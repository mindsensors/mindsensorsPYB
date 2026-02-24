#!/usr/bin/env pybricks-micropython
#!/usr/bin/env pybricks-micropython

#Version 1.01
#from ev3dev.ev3 import *
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import I2CDevice
from  mindsensorsPYB import PPS58
from  mindsensorsPYB import EV3Matrix
from  mindsensorsPYB import DIST_ToF
from  mindsensorsPYB import EV3RFid
from  mindsensorsPYB import LINELEADER
from  mindsensorsPYB import TFTPACK
from  mindsensorsPYB import ABSIMU
from  mindsensorsPYB import IRThermometer
from  mindsensorsPYB import VOLT
from  mindsensorsPYB import PFMATE
from  mindsensorsPYB import SUMOEYES
from  mindsensorsPYB import EV3Light
from  mindsensorsPYB import NumericPad
from  mindsensorsPYB import MMX
import os
import sys
import time


# state constants
ON = True
OFF = False


def debug_print(*args, **kwargs):
    '''Print debug messages to stderr.

    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)


def reset_console():
    '''Resets the console to the default state'''
    print('\x1Bc', end='')


def set_cursor(state):
    '''Turn the cursor on or off'''
    if state:
        print('\x1B[?25h', end='')
    else:
        print('\x1B[?25l', end='')


def set_font(name):
    '''Sets the console font

    A full list of fonts can be found with `ls /usr/share/consolefonts`
    '''
    os.system('setfont ' + name)



def PPS58Test() :
    PPS= PPS58(Port.S1,0x18)
    debug_print(PPS.GetFirmwareVersion())
    debug_print(PPS.GetDeviceId())
    debug_print(PPS.readAbsolute())
    debug_print(PPS.readGauge())
    debug_print(PPS.readRef())
    
def DIST_ToFTest() :
    ToF= DIST_ToF(Port.S1,0x02)
    debug_print(ToF.GetFirmwareVersion())
    debug_print(ToF.GetDeviceId())
    debug_print(ToF.readToFmm())

def matrixTest() :
    matrix= EV3Matrix(Port.S1,0x22)
    debug_print(matrix.GetFirmwareVersion())
    debug_print(matrix.GetDeviceId())
    matrix.display_row(1,0x55)
    matrix.display_row(2,0xaa)
    matrix.display_row(3,0x55)
    matrix.display_row(4,0xaa)
    matrix.display_row(5,0x55)
    matrix.display_row(6,0xaa)
    matrix.display_row(7,0x55)
    #matrix.display_font(0,66,1)

def rfidTest() :
    rfid= EV3RFid(Port.S1,0x22)
    debug_print(rfid.GetFirmwareVersion())
    debug_print(rfid.GetDeviceId())
    rfid.WriteBlockString(4,"this is Ravi test")
    debug_print(rfid.readUID())
    rfid.clearUID()
    debug_print(rfid.readUID())
    debug_print(rfid.ReadBlockArray(4))
    debug_print(rfid.ReadBlockString(4))
    while True :
        debug_print(rfid.readUID())
        time.sleep(1)    

def IRThermometerTest() :
    IRT= IRThermometer(Port.S1)
    debug_print(IRT.GetFirmwareVersion())
    debug_print(IRT.GetDeviceId())
    debug_print(IRT.readAmbientCelsius())
    debug_print(IRT.readTargetCelsius())
    debug_print(IRT.readAmbientFahr())
    debug_print(IRT.readTargetFahr())
    while True :
        time.sleep(1)
        debug_print(IRT.GetDeviceId())  

def LINELEADERTest() :
    LL= LINELEADER(Port.S1,0x2)
    debug_print(LL.GetFirmwareVersion())
    debug_print(LL.GetDeviceId())     

def EV3LightTest() :
    light= EV3Light(Port.S1,0x2C)
    debug_print(light.GetFirmwareVersion())
    debug_print(light.GetDeviceId())
    for i in  range(1, 15):
        light.clear()    
        light.lightled(i,250,0,0)
        time.sleep(.1)
        light.lightled(i,0,250,0)
        time.sleep(.1)
        light.lightled(i,0,0,250)
        time.sleep(.1)
    light.clear()     

def Test() :
    pad= NumericPad(Port.S1,0xB4)
    #pad.setup()
    while(1):
        print(pad.DecodeKeys(pad.GetKeysPressed()))
        time.sleep(.1)
            
def mmxTest() :
    mmx= MMX(Port.S1)
    debug_print(mmx.GetFirmwareVersion())
    debug_print(mmx.GetDeviceId())
    debug_print(mmx.battVoltage())
    debug_print(mmx.pos(mmx.MMX_Motor_2))
    mmx.runRotations(mmx.MMX_Motor_2, 1, 80, brakeOnCompletion=True, waitForCompletion=True)
    

def main():
    #The main function of our program

    # set the console just how we want it
    #reset_console()
    #set_cursor(OFF)
    #set_font('Lat15-Terminus24x12')

    #sens = mindsensors_i2c(2,0x22)
    #rfid=EV3RFid(2)
    #bus = SMBus(4)  # bus number is input port number + 2
    #I2C_ADDRESS = 0x01  # the default I2C address of the sensor
    # print something to the screen of the device
    print('Hello World!')

    # print something to the output panel in VS Code
    #debug_print(bus.read_byte_data(I2C_ADDRESS, 0x00))
    #debug_print(sens.GetDeviceId())
    #debug_print(sens.GetVendorName())

    # wait a bit so you have time to look at the display before the program
    # exits
    #debug_print(rfid.readUID())
    #rfid.clearUID()
    #debug_print(rfid.ReadBlock(9))
    #debug_print(rfid.readUID())

 
    
    '''
    while True:
        
        time.sleep(1)
    '''
if __name__ == '__main__':
    debug_print('Testing Begin!')
    #matrixTest()
    #rfidTest()
    #IRThermometerTest()
    
    #DIST_ToFTest() 
    #LINELEADERTest() 
    mmxTest()
    debug_print('Testing Ends!')

    