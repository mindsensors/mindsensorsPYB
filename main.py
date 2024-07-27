#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.media.ev3dev import Font
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

import os
import sys
import time

def PPS58Test() :
# Create your objects here.
    ev3 = EV3Brick()
    pps58 = PPS58(Port.S1,0x18)


    # Write your program here.
    print(pps58.GetDeviceId())
    print(pps58.GetVendorName())
    print(pps58.GetFirmwareVersion())
    print(pps58.readAbsolute(),pps58.readGauge(),pps58.readRef())
    count = 0
    while count < 200 :
        count =count +1
        time.sleep(0.1) 
        print(pps58.readAbsolute(),pps58.readGauge(),pps58.readRef())

    ev3.speaker.beep()


def matrixTest() :

    # Create your objects here.
    ev3 = EV3Brick()
    matrix= EV3Matrix(Port.S1,0x22)


    print(matrix.GetFirmwareVersion())
    print(matrix.GetDeviceId())
    #matrix.display_test()
    matrix.display_blink()
    matrix.display_row(1,0x55)
    matrix.display_row(2,0xaa)
    matrix.display_row(3,0x55)
    matrix.display_row(4,0xaa)
    matrix.display_row(5,0x55)
    matrix.display_row(6,0xaa)
    matrix.display_row(7,0x55)
    matrix.display_font(0,'M',1)
    

    ev3.speaker.beep()


def DIST_ToFTest() :
    # Create your objects here.
    ev3 = EV3Brick()
    ToF= DIST_ToF(Port.S1,0x02)
    
    print(ToF.GetFirmwareVersion())
    print(ToF.GetDeviceId())
    print(ToF.readToFmm())
    print(ToF.readToFin())
    ev3.speaker.beep()



def RfidTest() :
    # Create your objects here.
    ev3 = EV3Brick()
    rfid= EV3RFid(Port.S1,0x22)
    
    print(rfid.GetFirmwareVersion())
    print(rfid.GetDeviceId())
    print(rfid.readUID())
    '''
    rfid.WriteBlockString(0x04,"this is Ravi test")
    rfid.clearUID()
    print(rfid.ReadBlockArray(4))
    print(rfid.ReadBlockArray(4))
    while True :
        
        print(rfid.readUID())
        time.sleep(1)
    '''


def LineleaderTest() :
    # Create your objects here.
    ev3 = EV3Brick()
    LL= LINELEADER(Port.S1,0x02)
    
    print(LL.GetFirmwareVersion())
    print(LL.GetDeviceId())
    print(LL.ReadRaw_Calibrated())
    print(LL.steering())
    print(LL.average())
    print(LL.steering())
    print(LL.getSetPoint())
    ev3.speaker.beep()



def TFTTest() :
    # Create your objects here.
    ev3 = EV3Brick()
    tft= TFTPACK(Port.S1)
    tft.clear_display()
    tft.splash_display()
    tft.set_font_size(1)
    tft.set_curser_xy(0,0)
    tft.draw_line([220,30],[90,200],[200,200,0])
    tft.draw_circle([50,200],50,[0,250,0],False)
    tft.draw_circle([50,200],49,[200,0,0],True)
    
    tft.draw_rectangle([100,260],50,60,0,[250,0,0],False)
    tft.draw_rectangle([101,261],48,58,100,[0,0,100],True)
    tft.draw_triangle([100,190],[190,300],[230,200],[0,250,0],True)
    ev3.speaker.beep()


def ABSIMUTest() :
# Create your objects here.
    ev3 = EV3Brick()
    imu = ABSIMU(Port.S1,0x22)


    # Write your program here.
    print(imu.GetDeviceId())
    print(imu.GetVendorName())
    print(imu.GetFirmwareVersion())
    print(imu.get_accelall(),imu. get_heading(),imu.get_gyroall())
    count = 0
    while count < 200 :
        count =count +1
        time.sleep(0.1) 
        print(imu.get_accelall(),imu. get_heading(),imu.get_gyroall())
    ev3.speaker.beep()


def IRThermometerTest() :
# Create your objects here.
    ev3 = EV3Brick()
    irt = IRThermometer(Port.S1)


    # Write your program here.
    print(irt.GetDeviceId())
    print(irt.GetVendorName())
    print(irt.GetFirmwareVersion())
    print(irt.readAmbientCelsius(),irt.readTargetCelsius(),irt.readAmbientFahr(),irt.readTargetFahr())
    count = 0
    while count < 200 :
        count =count +1
        time.sleep(0.1) 
        print(irt.readAmbientCelsius(),irt.readTargetCelsius(),irt.readAmbientFahr(),irt.readTargetFahr())

    ev3.speaker.beep()


def VoltTest() :
# Create your objects here.
    ev3 = EV3Brick()
    volt =VOLT(Port.S1)


    # Write your program here.
    print(volt.GetDeviceId())
    print(volt.GetVendorName())
    print(volt.GetFirmwareVersion())
    print(volt.get_calibrated(),volt.get_relative(),volt.get_reference())
    count = 0
    while count < 200 :
        count =count +1
        time.sleep(0.1) 
        print(volt.get_calibrated(),volt.get_relative(),volt.get_reference())

    ev3.speaker.beep()



def PFMATETest() :
# Create your objects here.
    ev3 = EV3Brick()
    pf =PFMATE(Port.S1)


    # Write your program here.
    print(pf.GetDeviceId())
    print(pf.GetVendorName())
    print(pf.GetFirmwareVersion())
    
    count = 0
    while count < 20 :
        count =count +1
        time.sleep(5) 
        pf.controlMotorA( pf.PFMATE_CHANNEL1,pf.PFMATE_FORWARD,7)
        time.sleep(5) 
        pf.controlMotorA( pf.PFMATE_CHANNEL1,pf.PFMATE_REVERSE,7)
    pf.controlMotorA( pf.PFMATE_CHANNEL1,pf.PFMATE_FORWARD,0)
    pf.controlMotorB( pf.PFMATE_CHANNEL1,pf.PFMATE_FORWARD,0)
    ev3.speaker.beep()    


def Sumotest():
    # Create your objects here.
    ev3 = EV3Brick()
    sumo = SUMOEYES(Port.S1)
    print(sumo.read())

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

def NumericPadTest() :
    pad= NumericPad(Port.S1,0xB4)
    #pad.setup()
    while(1):
        print(pad.DecodeKeys(pad.GetKeysPressed()))
        time.sleep(.1)


#PPS58Test()
#matrixTest()
#DIST_ToFTest() 
#RfidTest() 
#LineleaderTest() 
#TFTTest()
#ABSIMUTest()
#IRThermometerTest()
#VoltTest()
#PFMATETest() 
#Sumotest()
#EV3LightTest()
NumericPadTest()
