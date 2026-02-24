#!/usr/bin/env pybricks-micropython
#!/usr/bin/env pybricks-micropython

#Version 1.02


from pybricks.hubs import EV3Brick
from pybricks.iodevices import I2CDevice
from pybricks.iodevices import  AnalogSensor
from pybricks.iodevices import UARTDevice
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.media.ev3dev import Font

import os
import sys
import time

# state constants
ON = True
OFF = False

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

## mindsensors_i2c: this class provides i2c functions
#  for read and write operations.
class mindsensors_i2c():
    
    def __init__(self,port, i2c_address):
        self.port = port
        self.i2c_address=i2c_address
        self.i2c = I2CDevice(port,i2c_address>>1)


    def errMsg(self):
        print ("Error accessing 0x%02X: Check your I2C address" % self.address)
        return -1

    ## Read a string from your i2c device starting at a given location
    #  @param self The object pointer.
    #  @param reg The first register of the string to read from.
    #  @param length The length of the string.
    def readString(self, reg, length):
        return self.i2c.read(reg, length)


    ## Read an unsigned byte from your i2c device at a given location
    #  @param self The object pointer.
    #  @param reg The register to read from.
    def readByte(self, reg):
        a =self.i2c.read(reg, 1)
        return int.from_bytes(a, "little")

    ## Write a byte to your i2c device at a given location
    #  @param self The object pointer.
    #  @param reg The register to write value at.
    #  @param value Value to write.
    def writeByte(self, reg, value):
        
        self.i2c.write( reg,value)
        pass

    ## Write a command to your i2c device at a command location
    #  @param self The object pointer.
    #  @param comamnd to write .
    def issueCommand(self,  value):
       self.i2c.write( 0x42, value)
             


    ## Read a byte array from your i2c device starting at a given location
    #  @param self The object pointer.
    #  @param reg The first register in the array to read from.
    #  @param length The length of the array.
    def readArray(self, reg, length):

        return self.i2c.read(reg, length)
        
    ## Write a byte array from your i2c device starting at a given location
    #  @param self The object pointer.
    #  @param reg The first register in the array to write to.
    #  @param arr The array to write.
    def writeArray(self, reg, arr):
        return self.i2c.write(reg, bytearray(arr))
        
    ## Read a signed byte from your i2c device at a given location
    #  @param self The object pointer.
    #  @param reg The register to read from.
    def readByteSigned(self, reg):
        a = self.i2c.read(reg, 1)
        signed_a =int.from_bytes(a, "little") #ctypes.c_byte.value 
        return signed_a

    ## Write an unsigned 16 bit integer from your i2c device from a given location. little endian write integers.
    #  @param self The object pointer.
    #  @param reg The first register of the first byte of the integer to write.
    #  @param int The integer to write.
    def writeInteger(self, reg, i):        
        i = int(i)
        results = self.i2c.write(reg, [i%256, (i>>8)%256])

    ## Read a signed 16 bit integer from your i2c device from a given location. Big endian read integers .
    #  @param self The object pointer.
    #  @param reg The first register of the first byte of the integer to read.
    def readIntegerSignedBE(self, reg):
        a = self.readIntegerBE(reg)
        if a&0x8000 : a = a -65535 
        return a       
    
    ## Read a signed 16 bit integer from your i2c device from a given location. little endian read integers .
    #  @param self The object pointer.
    #  @param reg The first register of the first byte of the integer to read.
    def readIntegerSigned(self, reg):
        a = self.readInteger(reg)
        if a&0x8000 : a = a -65535 
        return a

    ## Read an unsigned 32bit integer from your i2c device from a given location. Big endian read integers.
    #  @param self The object pointer.
    #  @param reg The first register of the first byte of the integer to read.
    def readLongBE(self, reg):
        
        results = self.i2c.read(reg,4)
        return results[3] + (results[2]<<8)+(results[1]<<16)+(results[0]<<24)           
        
    ## Read an unsigned 32bit integer from your i2c device from a given location. little endian read integers.
    #  @param self The object pointer.
    #  @param reg The first register of the first byte of the integer to read.
    def readLong(self, reg):
        results = self.i2c.read(reg,4)
        return results[0] + (results[1]<<8)+(results[2]<<16)+(results[3]<<24)       

    ## Read a signed 32bit integer from your i2c device from a given location. Big endian read integers .
    #  @param self The object pointer.
    #  @param reg The first register of the first byte of the integer to read.
    def readLongSignedBE(self, reg):
        a = self.readLongBE(reg)
        if a&0x80000000 : a = a -0xFFFFFFFF
        return a    
          
    ## Read a signed 32bit integer from your i2c device from a given location. little endian read integers .
    #  @param self The object pointer.
    #  @param reg The first register of the first byte of the integer to read.
    def readLongSigned(self, reg):
        a = self.readLong(reg)
        if a&0x80000000 : a = a -0xFFFFFFFF
        return a

    ##  Read the firmware version of the i2c device
    #  @param self The object pointer.
    def GetFirmwareVersion(self):
        try:
            ver = self.readString(0x00, 8)
            return ver
        except:
            print( "Error: Could not retrieve Firmware Version" )
            print ("Check I2C address and device connection to resolve issue")
            return ""

    ## Read an unsigned 16 bit integer from your i2c device from a given location.  Big-endian read integers .
    #  @param self The object pointer.
    #  @param reg The first register of the first byte of the integer to read.
    def readIntegerBE(self, reg):        
        results = self.i2c.read( reg, 2)
        return results[1] + (results[0]<<8)
        
    ## Read an unsigned 16 bit integer from your i2c device from a given location. little endian read integers.
    #  @param self The object pointer.
    #  @param reg The first register of the first byte of the integer to read.
    def readInteger(self, reg):        
        try:
            results = self.i2c.read(reg, 2)
            return results[0] + (results[1]<<8)
        except:
            return 0    

    ##  Read the vendor name of the i2c device
    #  @param self The object pointer.
    def GetVendorName(self):
        try:
            vendor = self.readString(0x08, 8)
            return vendor
        except:
            print ("Error: Could not retrieve Vendor Name")
            print ("Check I2C address and device connection to resolve issue")
            return ""
            
    ##  Read the i2c device id
    #  @param self The object pointer.
    def GetDeviceId(self):
        try:    
            device = self.readString(0x10, 8)
            return device
        except:
            print ("Error: Could not retrieve Device ID")
            print ("Check I2C address and device connection to resolve issue")
            return ""    


## TFTPack: this class provides functions for TFTPackNx from mindsensors.com
#  for read and write operations.

class TFTPACK():
    TouchPoint =[0,0]

    ## Initialize the class with the i2c address of your TFTPack
    #  @param self The object pointer.
    #  @param TFTPack_address Address of your TFTPack.
    def __init__(self, port):

        self.port = port
        # Initialize sensor port  as a uart device
        self.ser = UARTDevice(self.port, baudrate=115200)
        time.sleep(0.1)


    def rgb_hex565(self,RGB):
            return( ((int(RGB[0] / 255 * 31) << 11) | (int(RGB[1] / 255 * 63) << 5) | (int(RGB[2] / 255 * 31))))
    


    def  print(self,data): 
        self.ser.write(str(data))
        #if len(str(data) >15:
        #    self.port.write(str(data)[:15])

    ## Writes command to TFTpack
    #  @param self The object pointer.
    #  @param command Value to write to the command register.
    def write_command(self, command):

        self.ser.write(command)
        go =1
        while go:
            inc =self.ser.read(1)
            if inc == b'\x06':
                time.sleep(0.004)
                self.TouchPoint[0] = int.from_bytes(self.ser.read(2), "little")
                self.TouchPoint[1] = int.from_bytes(self.ser.read(2), "little")
                #print(self.TouchPoint)
                go =0
                
    ## Clears the TFTPack display
    #  @param self The object pointer.
    def clear_display(self):
        self.write_command(b'\x01'+ b'\x00')

    ## Toggels the TFTPack display Backlight
    #  @param self The object pointer.
    def backlight(self,state):
        self.write_command(b'\x01'+ b'\x01'+state.to_bytes(1, 'little'))    


    def get_touch(self):
        self.write_command(b'\x01'+ b'\x02')
        return self.TouchPoint     

    ## Run demo mode of  TFTPack display
    #  @param self The object pointer.
    def run_demo(self):
        self.write_command(b'\x01' + b'\x04')


    ## invert_display mode of  TFTPack display
    #  @param self The object pointer.
    def invert_display(self):
        self.write_command(b'\x01' + b'\x12')    
        
    ## splash_display mode of  TFTPack display
    #  @param self The object pointer.
    def splash_display(self):
        self.write_command(b'\x01' + b'\x13')    

    ## set_curser at xy of  TFTPack display
    #  @param self The object pointer.
    #   @param  Xpos and Ypos
    def set_curser_xy(self,position):
        self.write_command(b'\x01' + b'\x18'+position[0].to_bytes(2, 'little')+position[1].to_bytes(2, 'little'))
  

    ## set_color of  TFTPack display text to [R,G,B]
    #  @param self The object pointer.
    #  @param RGB tuple  as [R,G,B]
    def set_color(self,Frgb,Brgb=[0,0,0]):
        self.write_command(b'\x01' + b'\x07'+self.rgb_hex565(Frgb).to_bytes(2, 'little')+self.rgb_hex565(Brgb).to_bytes(2, 'little') )
        
    ## set_font_size of  TFTPack display text 
    #  @param self The object pointer.
    #  @param size  of font
    def set_font_size(self,font,size):
        self.write_command(b'\x01' + b'\x06'+size.to_bytes(1, 'little')+font.to_bytes(1, 'little'))

        
  

    ## draw_line on  TFTPack display from start to finish with Color RGB
    #  @param self The object pointer.
    #  @param start position tuple[x1,y1]
    #  @param start position tuple[x2,y2]
    #  @param RGB tuple  as [R,G,B]
    
    def draw_line(self,start,end,RGB):
        self.write_command(b'\x01' + b'\x0C'+start[0].to_bytes(2, 'little')+start[1].to_bytes(2, 'little')+end[0].to_bytes(2, 'little')+end[1].to_bytes(2, 'little')+self.rgb_hex565(RGB).to_bytes(2, 'little'))  
   
   
    def draw_pixel(self,position,RGB):
        self.write_command(b'\x01' + b'\x10'+position[0].to_bytes(2, 'little')+position[1].to_bytes(2, 'little')+self.rgb_hex565(RGB).to_bytes(2, 'little'))  



    ## draw_circle on  TFTPack display from center of radieus with Color RGB, W/WO fill
    #  @param self The object pointer.
    #  @param center position tuple[x1,y1]
    #  @param radius position tuple[x2,y2]
    #  @param RGB tuple  as [R,G,B]
    #  @param fill bool  as True of False
    
    def draw_circle(self,center,radius,RGB,fill):
        if fill ==True: self.write_command(b'\x01' + b'\x43'+center[0].to_bytes(2, 'little')+center[1].to_bytes(2, 'little')+radius.to_bytes(1, 'little')+self.rgb_hex565(RGB).to_bytes(2, 'little'))  
        else :self.write_command(b'\x01' + b'\x03'+center[0].to_bytes(2, 'little')+center[1].to_bytes(2, 'little')+radius.to_bytes(1, 'little')+self.rgb_hex565(RGB).to_bytes(2, 'little'))  
     
    ## draw_rectangle on  TFTPack display at topleft  width and hight with curved corner of radius and with Color RGB, W/WO fill
    #  @param self The object pointer.
    #  @param center topleft tuple[x1,y1]
    #  @param  width 
    #  @param  hight 
    #  @param  radius of curvature on corners
    #  @param RGB tuple  as [R,G,B]
    #  @param fill bool  as True of False
    
    def draw_rectangle(self,topleft,width,hight,radius,RGB,fill):
        if radius ==0:
            if fill ==True: self.write_command(b'\x01' + b'\x4F'+topleft[0].to_bytes(2, 'little')+topleft[1].to_bytes(2, 'little')+width.to_bytes(2, 'little')+hight.to_bytes(2, 'little')+self.rgb_hex565(RGB).to_bytes(2, 'little'))  
            else :self.write_command(b'\x01' + b'\x0F'+topleft[0].to_bytes(2, 'little')+topleft[1].to_bytes(2, 'little')+width.to_bytes(2, 'little')+hight.to_bytes(2, 'little')+self.rgb_hex565(RGB).to_bytes(2, 'little'))  
        else :
            if fill ==True: self.write_command(b'\x01' + b'\x59'+topleft[0].to_bytes(2, 'little')+topleft[1].to_bytes(2, 'little')+width.to_bytes(2, 'little')+hight.to_bytes(2, 'little')+radius.to_bytes(1, 'little')+self.rgb_hex565(RGB).to_bytes(2, 'little'))  
            else :self.write_command(b'\x01' + b'\x19'+topleft[0].to_bytes(2, 'little')+topleft[1].to_bytes(2, 'little')+width.to_bytes(2, 'little')+hight.to_bytes(2, 'little')+radius.to_bytes(1, 'little')+self.rgb_hex565(RGB).to_bytes(2, 'little'))  

                        
    ## draw_triangle on  TFTPack display from vertex1 ,vertex2 and vertex3 with Color RGB, W/WO fill
    #  @param self The object pointer.
    #  @param vertex1 position tuple[x1,y1]
    #  @param vertex2 position tuple[x2,y2]
    #  @param vertex3 position tuple[x3,y3]
    #  @param RGB tuple  as [R,G,B]
    #  @param fill bool  as True of False
    
    def draw_triangle(self,vertex1,vertex2,vertex3,RGB,fill):
        if fill ==True: self.write_command(b'\x01' + b'\x4E'+vertex1[0].to_bytes(2, 'little')+vertex1[1].to_bytes(2, 'little')+vertex2[0].to_bytes(2, 'little')+vertex2[1].to_bytes(2, 'little')+vertex3[0].to_bytes(2, 'little')+vertex3[1].to_bytes(2, 'little')+self.rgb_hex565(RGB).to_bytes(2, 'little'))  
        else :self.write_command(b'\x01' + b'\x0E'+vertex1[0].to_bytes(2, 'little')+vertex1[1].to_bytes(2, 'little')+vertex2[0].to_bytes(2, 'little')+vertex2[1].to_bytes(2, 'little')+vertex3[0].to_bytes(2, 'little')+vertex3[1].to_bytes(2, 'little')+self.rgb_hex565(RGB).to_bytes(2, 'little'))  
    






## PPS58 : this class provides EV3Dev specific interface for the   PPS58
#  for read and write operations.
class PPS58(mindsensors_i2c):
    def __init__(self, port,i2c_address=0x18):
        mindsensors_i2c.__init__(self,port, i2c_address)

    ## Read a absolute pressure from PPS58 sensor and return   .
    #  @param self The object pointer.
    def  readAbsolute(self):
        results = self.readInteger(0x43)
        return results

    ## Read a gauge pressure from PPS58 sensor 
    #  @param self The object pointer.
    def  readGauge(self):
        
        results = self.readInteger(0x45)
        return results

    ## Read a referance pressure  from PPS58 sensor
    #  @param self The object pointer.
    def  readRef(self):
        
        results = self.readInteger(0x47)
        return results
    ## Write usnit register for reading pressure  from PPS58 sensor
    #‘p’: PSI
    #‘b’: millibar
    #‘k’: kPA
    #  @param self The object pointer.    
    def unitSelect(self,unit):
        self.writeByte(0x42,unit)


    
##  EV3Matrix : this class provides EV3Dev specific interface for the   EV3Matrix
#  for read and write operations.
class EV3Matrix(mindsensors_i2c):
    command_reg = 0x41
    
    Brightness_reg = 0x42
    Font_type_reg = 0x43
    Font_data_reg = 0x43
    Row_reg = 0x45
    Column_reg = 0x46
    Value_reg = 0x47
    Data_buff_reg = 0x48

    def __init__(self, port,i2c_address=0x22):
        mindsensors_i2c.__init__(self,port, i2c_address)  
    def display_blink(self):
        self.writeByte(0x41,'B')
       
    
    def display_test(self):
        self.writeByte(0x41,'T')   
    def display_miror(self) :
        self.writeByte(self.command_reg,'V') 
    def display_font(self, font_type = 0,font_data = ' ',brightness=2) :
        self.writeArray(self.Brightness_reg,[brightness,font_type,ord(font_data)])        
        #self.writeByte(self.Font_type_reg,font_type) 
        #self.writeByte(self.Font_data_reg,font_data) 
        self.writeByte(self.command_reg,'F')
        #self.issueCommand(ord('F')) 
    def display_pix(self, row = 0,column=0,data = 1) :
        self.writeArray(self.Row_reg,[row,column,data])        
        self.writeByte(self.command_reg,'P')
    def display_column(self,column=0,data = 0) :
        self.writeArray(self.Row_reg,[0,column,data])        
        self.writeByte(self.command_reg,'C') 
    def display_row(self,row=0,data = 0) :
        self.writeArray(self.Row_reg,[row,0,data])        
        self.writeByte(self.command_reg,'R')        



## DIST_ToF : this class provides EV3Dev specific interface for the   DIST_ToF
#  for read and write operations.
class DIST_ToF(mindsensors_i2c):
    def __init__(self, port,i2c_address=0x02):
        mindsensors_i2c.__init__(self,port, i2c_address)

    ## Read a distance from ToF sensor and return   .
    #  @param self The object pointer.
    def  readToF(self):
        results = self.readInteger(0x42)
        return results

    ## Read a distance from ToF sensor in mm 
    #  @param self The object pointer.
    def  readToFmm(self):
        
        return self.readToF()

    ## Read a distance from ToF sensor in inches
    #  @param self The object pointer.
    def  readToFin(self):
        
        return self.readToF()/25.4   


## EV3RFid : this class provides EV3Dev specific interface for the  EV3RFid
#  for read and write operations.
class EV3RFid(mindsensors_i2c):
    def __init__(self, port,i2c_address=0x22):
        mindsensors_i2c.__init__(self,port, i2c_address)

    ## Clear UID stored in device .
    #  @param self The object pointer.
    def clearUID(self):
        self.writeByte(65,'C')

    ## Read a EEPROM stored data block from RFid card and return in Array  .
    #  @param self The object pointer.
    #  @param BlockID The block id to read.first 4 block and every 4th block can not be used    
    def ReadBlockArray(self,BlockID):
        self.writeByte(0x4f,str(BlockID))
        self.writeByte(0x41,'R')
        time.sleep(2)
        return self.readArray(0x50,16)

    ## Read a EEPROM stored data block from RFid card and return in String  .
    #  @param self The object pointer.
    #  @param BlockID The block id to read.first 4 block and every 4th block can not be used  
    def ReadBlockString(self,BlockID):
        self.writeByte(0x4f,str(BlockID))
        self.writeByte(0x41,'R')
        time.sleep(2)
        return self.readString(0x50,16)

    ## Write a string in EEPROM data block of RFid card .
    #  @param self The object pointer.
    #  @param BlockID The block id to read.first 4 block and every 4th block can not be used  
    #  @param dataString The string to store
    def WriteBlockString(self,BlockID,dataString):
        count = 0
        
        
        self.writeByte(0x4f,str(BlockID))
        for elem in dataString :
            if count < 16 : 
                self.writeByte(0x50+count,str(elem))
                count = count+1
        while count < 16  :
            self.writeByte(0x50+count,b'x0')
            count = count+1

        self.writeByte(0x41,'W')
        
        return count

    ## Write a Data Array in EEPROM data block of RFid card .
    #  @param self The object pointer.
    #  @param BlockID The block id to read.first 4 block and every 4th block can not be used  
    #  @param dataString The string to store
    def WriteBlockArray(self,BlockID,data):
        count = 0
        self.writeByte(0x4f,str(BlockID))
        for elem in data :
            if count < 16 : 
                self.writeByte(0x50+count,str(elem))
                count = count+1
        while count < 16  :
            self.writeByte(0x50+count,b'x0')
            count = count+1

        self.writeByte(0x41,'W')
        
        return count

    ## Read a UID  from RFid card and return   .
    #  @param self The object pointer.
    def  readUID(self):
        results = self.readArray(0x44,4)
        return self.readLong(0x44)



    

## IRThermometer : this class provides EV3Dev specific interface for the
#  IR Thermometer sensor:
#  http://www.mindsensors.com/products/170-ir-temperature-sensor-for-ev3-or-nxt
#
class IRThermometer(mindsensors_i2c):
    ## Default I2C Address
    IRT_ADDRESS = 0x2A
    ## Command Register
    IRT_COMMAND = 0x41
    # temperature registers
    IRT_AMBIENT_CELSIUS = 0x42
    IRT_TARGET_CELSIUS = 0x44
    IRT_AMBIENT_FAHR = 0x46
    IRT_TARGET_FAHR = 0x48

    ## Initialize the class with the i2c address of your device
    #  @param self The object pointer.
    #  @param port The PiStorms bank.
    #  @param address Address of your device
    #  @remark
    def __init__(self, port,i2c_address=IRT_ADDRESS):
        mindsensors_i2c.__init__(self,port, i2c_address)
    

    def readAmbientCelsius(self):
        return (float(self.readInteger(self.IRT_AMBIENT_CELSIUS))/100)

    def readTargetCelsius(self):
        return (float(self.readInteger(self.IRT_TARGET_CELSIUS))/100)

    def readAmbientFahr(self):
        return (float(self.readInteger(self.IRT_AMBIENT_FAHR))/100)

    def readTargetFahr(self):
        return (float(self.readInteger(self.IRT_TARGET_FAHR))/100)


## LINELEADER: this class provides functions for Lineleader from mindsensors.com
#  for read and write operations.
class LINELEADER(mindsensors_i2c):

    ## Default Lineleader I2C Address
    LL_ADDRESS = 0x02
    ## Command Register
    LL_COMMAND = 0x41
    ## Steering Register. Will return a signed byte value
    LL_STEERING = 0x42
    ## Average Register. Will return a byte value
    LL_AVERAGE = 0x43
    ## Steering Register. Will return a byte value
    LL_RESULT = 0x44
    ## Setpoint Register
    LL_SETPOINT = 0x45
    ## KP Register
    LL_Kp = 0x46
    ## Ki Register
    LL_KI = 0x47
    ## Kd Register
    LL_KD = 0x48
    ## Kp factor Register
    LL_KPfactor = 0x61
    ## Ki factor Register
    LL_KIfactor = 0x62
    ## Kd factor Register
    LL_KDfactor = 0x63

    LL_CALIBRATED = 0x49
    LL_UNCALIBRATED = 0x74

    ## Initialize the class with the i2c address of your Lineleader
    #  @param self The object pointer.
    #  @param ll_address Address of your LightSensorArray.
    
    def __init__(self,  port,ll_address = LL_ADDRESS):
        #the LSA address
        mindsensors_i2c.__init__(self, port, ll_address )

    ## Writes a value to the command register
    #  @param self The object pointer.
    #  @param cmd Value to write to the command register.
    def command(self, cmd):
        self.writeByte(self.LL_COMMAND, cmd)

    ## Calibrates the white value for the LightSensorArray
    #  @param self The object pointer.
    def White_Cal(self):
        self.command(87)

    ## Calibrates the black value for the LightSensorArray
    #  @param self The object pointer.
    def Black_Cal(self):
        self.command(66)

    ## Wakes up or turns on the LEDs of the LightSensorArray
    #  @param self The object pointer.
    def Wakeup(self):
        self.command(80)

    ## Puts to sleep, or turns off the LEDs of the LightSensorArray
    #  @param self The object pointer.
    def Sleep(self):
        self.command(68)

    ## Reads the eight(8) calibrated light sensor values of the LightSensorArray
    #  @param self The object pointer.
    def ReadRaw_Calibrated(self):
        return self.readArray(self.LL_CALIBRATED, 8)
        

    ## Reads the eight(8) uncalibrated light sensor values of the LightSensorArray
    #  @param self The object pointer.
    def ReadRaw_Uncalibrated(self):
        
        s1 = self.readInteger(self.LL_UNCALIBRATED)
        s2 = self.readInteger(self.LL_UNCALIBRATED + 2)
        s3 = self.readInteger(self.LL_UNCALIBRATED + 4)
        s4 = self.readInteger(self.LL_UNCALIBRATED + 6)
        s5 = self.readInteger(self.LL_UNCALIBRATED + 8)
        s6 = self.readInteger(self.LL_UNCALIBRATED + 10)
        s7 = self.readInteger(self.LL_UNCALIBRATED + 12)
        s8 = self.readInteger(self.LL_UNCALIBRATED + 14)
        array = [s1, s2, s3, s4, s5, s6, s7, s8]
        return array
       

    ## Read the steering value from the Lineleader (add or subtract this value to the motor speed)
    #  @param self The object pointer.
    def steering(self):
    
        return self.readByteSigned(self.LL_STEERING)
       

    ## Read the average weighted value of the current line from position from the Lineleader
    #  @param self The object pointer.
    def average(self):
        return self.readByte(self.LL_AVERAGE)
        
    ## Reads the result of all 8 light sensors form the LineLeader as 1 byte (1 bit for each sensor)
    #  @param self The object pointer.
    def result(self):
        return self.readByte(self.LL_RESULT)
        
    ## Reads the eight(8) calibrated light sensor values of the LightSensorArray
    #  @param self The object pointer.
    def getSetPoint(self):
        return self.readByte(self.LL_SETPOINT)
        

    ## Reads the eight(8) calibrated light sensor values of the LightSensorArray
    #  @param self The object pointer.
    def setSetPoint(self):
        return self.writeByte(self.LL_SETPOINT)
        

    ## Write the Kp value to the Lineleader
    #  @param self The object pointer.
    def setKP(self):
        return self.writeByte(self.LL_KP)
        

    ## Write the Ki value to the Lineleader
    #  @param self The object pointer.
    def setKI(self):
        return self.writeByte(self.LL_KI)
        

    ## Write the Kd value to the Lineleader
    #  @param self The object pointer.
    def setKD(self):
        return self.writeByte(self.LL_KD)
        

    ## Write the Kp factor value to the Lineleader
    #  @param self The object pointer.
    def setKPfactor(self):
        return self.writeByte(self.LL_KPfactor)
        
    ## Write the Ki factor value to the Lineleader
    #  @param self The object pointer.
    def setKIfactor(self):
        return self.writeByte(self.LL_KIfactor)
        
    ## Write the Kd factor value to the Lineleader
    #  @param self The object pointer.
    def setKDfactor(self):
        return self.writeByte(self.LL_KDfactor)
        

    ## Read the Kp value from the Lineleader
    #  @param self The object pointer.
    def getKP(self):
        return self.readByte(self.LL_KP)
        

    ## Read the Ki value from the Lineleader
    #  @param self The object pointer.
    def getKI(self):
        return self.readByte(self.LL_KI)
        
    ## Read the Kd value from the Lineleader
    #  @param self The object pointer.
    def getKD(self):
        return self.readByte(self.LL_KD)
        
    ## Read the Kp factor value to the Lineleader
    #  @param self The object pointer.
    def getKPfactor(self):
        return self.readByte(self.LL_KPfactor)
        

    ## Read the Ki factor value to the Lineleader
    #  @param self The object pointer.
    def getKIfactor(self):
        return self.readByte(self.LL_KIfactor)
        

    ## Read the Kd factor value to the Lineleader
    #  @param self The object pointer.
    def getKDfactor(self):
        return self.readByte(self.LL_KDfactor)
        


## LSA: this class provides functions for LightSensorArray from mindsensors.com
#  for read and write operations.
class LSA(mindsensors_i2c):

    ## Default LightSensorArray I2C Address
    LSA_ADDRESS = 0x14
    ## Command Register
    LSA_COMMAND = 0x41
    ## Calibrated Register. Will return an 8 byte array
    LSA_CALIBRATED = 0x42
    ## Uncalibrated Register. Will return an 8 byte array
    LSA_UNCALIBRATED = 0x6A

    ## Initialize the class with the i2c address of your LightSensorArray
    #  @param self The object pointer.
    #  @param lsa_address Address of your LightSensorArray.
    def __init__(self,  port,lsa_address = LSA_ADDRESS):
        #the LSA address
        mindsensors_i2c.__init__(self, port, lsa_address)
 
    ## Writes a value to the command register
    #  @param self The object pointer.
    #  @param cmd Value to write to the command register.
    def command(self, cmd):
        self.writeByte(self.LSA_COMMAND, cmd)

    ## Calibrates the white value for the LightSensorArray
    #  @param self The object pointer.
    def White_Cal(self):
        self.command(87)

    ## Calibrates the black value for the LightSensorArray
    #  @param self The object pointer.
    def Black_Cal(self):
        self.command(66)

    ## Wakes up or turns on the LEDs of the LightSensorArray
    #  @param self The object pointer.
    def Wakeup(self):
        self.command(80)

    ## Puts to sleep, or turns off the LEDs of the LightSensorArray
    #  @param self The object pointer.
    def Sleep(self):
        self.command(68)

    ## Reads the eight(8) calibrated light sensor values of the LightSensorArray
    #  @param self The object pointer.
    def ReadRaw_Calibrated(self):
        return self.readArray(self.LSA_CALIBRATED, 8)
        

    ## Reads the eight(8) uncalibrated light sensor values of the LightSensorArray
    #  @param self The object pointer.
    def ReadRaw_Uncalibrated(self):
        s1 = self.readInteger(self.LSA_UNCALIBRATED)
        s2 = self.readInteger(self.LSA_UNCALIBRATED + 2)
        s3 = self.readInteger(self.LSA_UNCALIBRATED + 4)
        s4 = self.readInteger(self.LSA_UNCALIBRATED + 6)
        s5 = self.readInteger(self.LSA_UNCALIBRATED + 8)
        s6 = self.readInteger(self.LSA_UNCALIBRATED + 10)
        s7 = self.readInteger(self.LSA_UNCALIBRATED + 12)
        s8 = self.readInteger(self.LSA_UNCALIBRATED + 14)
        array = [s1, s2, s3, s4, s5, s6, s7, s8]
        return array


 ## BLOB: this class is a subclass of NXTCAM. There is no need to call this class directly.
class BLOB():

    ## Initialize the class with the parameters passed from getBlobs() in the NXTCAM() class
    #  @param self The object pointer.
    #  @param color The color of the specified tracked object.
    #  @param left The left coordinate of the specified tracked object.
    #  @param top The top coordinate of the specified tracked object.
    #  @param right The right coordinate of the specified tracked object.
    #  @param bottom The bottom coordinate of the specified tracked object.
    #  @remark
    def __init__(self, color, left, top, right, bottom):
        self.color = color
        self.left = left
        self.top = top
        self.right = right
        self.bottom = bottom

## NXTCAM: this class provides functions for models of the NXTCAM and PixyAdapter from mindsensors.com
#  for read and write operations.
class NXTCAM(mindsensors_i2c):

    ## Default NXTCAM I2C Address
    NXTCAM_ADDRESS = (0x02)
    ## Command Register
    COMMAND = 0x41
    ## Number of Tracked Ojects Register. Will return a byte (0-8)
    NumberObjects = 0x42
    ## First Register Containing Tracked Object Data. This is to be read in an array
    Color = 0x43
    ## X-axis Top Register
    X_Top = 0x44
    ## Y-axis Top Register
    Y_Top = 0x45
    ## X-axis Bottom Register
    X_Bottom = 0x46
    ## Y-axis Bottom Register
    Y_Bottom = 0x47

    ## Initialize the class with the i2c address of your NXTCAM
    #  @param self The object pointer.
    #  @param nxtcam_address Address of your AngleSensor.
    #  @remark
    def __init__(self,port, nxtcam_address = NXTCAM_ADDRESS):
        mindsensors_i2c.__init__(self, port,nxtcam_address)

    ## Writes a value to the command register
    #  @param self The object pointer.
    #  @param command Value to write to the command register.
    def command(self, command):
        self.writeByte(self.COMMAND, command)

    ## Sort the detected objects by size
    #  @param self The object pointer.
    def sortSize(self):
        self.command(65)
        

    ## Track detected colors as objects
    #  @param self The object pointer.
    def trackObject(self):
        self.command(66)
        

    ## Write to image sensor registers
    #  @param self The object pointer.
    def writeImageRegisters(self):
        self.command(67)
        

    ## Disable tracking
    #  @param self The object pointer.
    def stopTracking(self):
        self.command(68)
        

    ## Enable tracking
    #  @param self The object pointer.
    def startTracking(self):
        self.command(69)
        

    ## Get the color map from NXTCAM
    #  @param self The object pointer.
    def getColorMap(self):
        self.command(71)
        

    ## Turn on illumination
    #  @param self The object pointer.
    def illuminationOn(self):
        self.command(73)
        

    ## Read to image sensor registers
    #  @param self The object pointer.
    def readImageRegisters(self):
        self.command(72)
        
    ## Track detected colors as lines
    #  @param self The object pointer.
    def trackLine(self):
        self.command(76)
        
    ## Ping the NXTCAM
    #  @param self The object pointer.
    def ping(self):
        self.command(80)
        

    ## Reset the NXTCAM
    #  @param self The object pointer.
    def reset(self):
        self.command(82)
        
    ## Send ColorMap to NXTCAM
    #  @param self The object pointer.
    def sendColorMap(self):
        self.command(83)
       

    ## Turn off illumination
    #  @param self The object pointer.
    def illuminationOff(self):
        self.command(84)
        

    ## Sort tracked objects by color
    #  @param self The object pointer.
    def sortColor(self):
        self.command(85)
        

    ## Get the firmware version of the NXTCAM
    #  @param self The object pointer.
    def firmware(self):
        self.command(86)
        

    ## Do not sort tracked objects
    #  @param self The object pointer.
    def sortNone(self):
        self.command(88)
        

    ## Read the number of objects detected (0-8)
    #  @param self The object pointer.
    def getNumberObjects(self):
        return self.readByte(self.NumberObjects)
        

    ## Reads data of the tracked object(s)
    #  @param self The object pointer.
    #  @param blobNum The number of the tracked object.
    #  @remark
    #  To use this function in your program:
    #  @code
    #  from mindsensors  import NXTCAM
    #  ...
    #  cam = NXTCAM()
    #  cam.startTracking()
    #  cam.trackObject()
    #  b = cam.getBlobs(1)
    #  print ("Color: " + str(b.color))
    #  print ("Left: " + str(b.left))
    #  print ("Top: " + str(b.top))
    #  print ("Right: " + str(b.right))
    #  print ("Bottom: " + str(b.bottom))
    #  @endcode
    def getBlobs(self, blobNum = 1):
        data= [0,0,0,0,0]
        blobs = self.getNumberObjects()
        i = blobNum - 1
        if (blobNum > blobs):
            print ("blobNum is greater than amount of blobs tracked.")
            return 0
        else:
            #while(i < blobs):
            data[0] = color = self.readByte(self.Color + (i*5))
            data[1] = left = self.readByte(self.X_Top + (i*5))
            data[2] = top = self.readByte(self.Y_Top + (i*5))
            data[3] = right = self.readByte(self.X_Bottom + (i*5))
            data[4] = bottom = self.readByte(self.Y_Bottom + (i*5))
            return BLOB(color,left,top,right,bottom)
        

## @package mindsensors
#  This module contains classes and functions necessary for use of mindsensors.com I2C devices with Raspberry Pi

## ABSIMU: this class provides functions for models of the ABSIMU from mindsensors.com
#  for read and write operations.
class ABSIMU(mindsensors_i2c):

    ## Default ABSIMU I2C Address
    ABSIMU_ADDRESS = (0x22)
    ## Command Register
    COMMAND = 0x41
    ## X-Axis Tilt Register. Will return a signed integer reading
    TILT_X = 0x42
    ## Y-Axis Tilt Register. Will return a signed integer reading
    TILT_Y = 0x43
    ## Z-Axis Tilt Register. Will return a signed integer reading
    TILT_Z = 0x44
    ## X-Axis Accelerometer Register. Will return a signed integer reading (-1050 - 1050)
    ACCEL_X = 0x45
    ## Y-Axis Accelerometer Register. Will return a signed integer reading (-1050 - 1050)
    ACCEL_Y = 0x47
    ## Z-Axis Accelerometer Register. Will return a signed integer reading (-1050 - 1050)
    ACCEL_Z = 0x49
    ## Compass Heading Register. Will return an unsigned integer reading (0 - 360)
    CMPS = 0x4B
    ## X-Axis Magnetometer Register. Will return a signed integer reading
    MAG_X = 0x4D
    ## Y-Axis Magnetometer Register. Will return a signed integer reading
    MAG_Y = 0x4F
    ## Z-Axis Magnetometer Register. Will return a signed integer reading
    MAG_Z = 0x51
    ## X-Axis Gyroscope Register. Will return a signed integer reading
    GYRO_X = 0x53
    ## Y-Axis Gyroscope Register. Will return a signed integer reading
    GYRO_Y = 0x55
    ## Z-Axis Gyroscope Register. Will return a signed integer reading
    GYRO_Z = 0x57

    ## Initialize the class with the i2c address of your AbsoluteIMU
    #  @param self The object pointer.
    #  @param absimu_address Address of your AbsoluteIMU.
    #  @remark
    def __init__(self, port,absimu_address = ABSIMU_ADDRESS):
        mindsensors_i2c.__init__(self,port, absimu_address )

    ## Writes a value to the command register
    #  @param self The object pointer.
    #  @param command Value to write to the command register.
    def command(self, command):
        self.writeByte(self.COMMAND, command)

    ## Reads the tilt value along the x-axis
    #  @param self The object pointer.
    def get_tiltx(self):
        return self.readByteSigned(self.TILT_X)
        
    ## Reads the tilt value along the y-axis
    #  @param self The object pointer.
    def get_tilty(self):
        return self.readByteSigned(self.TILT_Y)
        
    ## Reads the tilt value along the z-axis
    #  @param self The object pointer.
    def get_tiltz(self):
        return self.readByteSigned(self.TILT_Z)
        
    ## Reads the tilt values
    #  @param self The object pointer.
    def get_tiltall(self):
        res = [(self.get_tiltx(),
                    self.get_tilty(),
                    self.get_tiltz())]
        return res
        

    ## Reads acceleromter value along the x-axis
    #  @param self The object pointer.
    def get_accelx(self):
        return self.readIntegerSigned(self.ACCEL_X)
       

    ## Reads acceleromter value along the y-axis
    #  @param self The object pointer.
    def get_accely(self):
        return self.readIntegerSigned(self.ACCEL_Y)
       

    ## Reads acceleromter value along the z-axis
    #  @param self The object pointer.
    def get_accelz(self):
        return self.readIntegerSigned(self.ACCEL_Z)
        

    ## Reads the accelerometer values
    #  @param self The object pointer.
    def get_accelall(self):
        res = [(self.get_accelx(),
                self.get_accely(),
                self.get_accelz())]
        return res
        

    ## Reads compass heading
    #  @param self The object pointer.
    def get_heading(self):
        head = self.readInteger(self.CMPS)
        while(head > 360 or head < 0):
            head = self.readInteger(self.CMPS)
        return head
        

    ## Reads magnetometer value along the x-axis
    #  @param self The object pointer.
    def get_magx(self):
        return self.readIntegerSigned(self.MAG_X)
        

    ## Reads magnetometer value along the y-axis
    #  @param self The object pointer.
    def get_magy(self):
        return self.readIntegerSigned(self.MAG_Y)
        

    ## Reads magnetometer value along the z-axis
    #  @param self The object pointer.
    def get_magz(self):
        return self.readIntegerSigned(self.MAG_Z)
        

    ## Reads the magnetometer values
    #  @param self The object pointer.
    def get_magall(self):
        res = [(self.get_magx(),
                    self.get_magy(),
                    self.get_magz())]
        return res
        

    ## Reads gyroscope value along the x-axis
    #  @param self The object pointer.
    def get_gyrox(self):
        return self.readIntegerSigned(self.GYRO_X)
        

    ## Reads gyroscope value along the y-axis
    #  @param self The object pointer.
    def get_gyroy(self):
        return self.readIntegerSigned(self.GYRO_Y)
        

    ## Reads gyroscope value along the z-axis
    #  @param self The object pointer.
    def get_gyroz(self):
        return self.readIntegerSigned(self.GYRO_Z)
        

    ## Reads the tilt values
    #  @param self The object pointer.
    def get_gyroall(self):
        res = [(self.get_gyrox(),
                    self.get_gyroy(),
                    self.get_gyroz())]
        return res
       

    ## Starts the compass calibration process
    #  @param self The object pointer.
    def start_cmpscal(self):
        self.command(67)
       

    ## Stops the compass calibration process
    #  @param self The object pointer.
    def stop_cmpscal(self):
        self.command(99)
       

    ## Sets accelerometer sensitivity to 2G
    #  @param self The object pointer.
    def accel_2G(self):
        self.command('1')
        

    ## Sets accelerometer sensitivity to 4G
    #  @param self The object pointer.
    def accel_4G(self):
        self.command('2')
        

    ## Sets accelerometer sensitivity to 8G
    #  @param self The object pointer.
    def accel_8G(self):
        self.command('3')
        

    ## Sets accelerometer sensitivity to 16G
    #  @param self The object pointer.
    def accel_16G(self):
        self.command('4')
        

## CURRENT: this class provides functions for NXTCurrentMeter from mindsensors.com
#  for read and write operations.
class CURRENT(mindsensors_i2c):

    ## Default CurrentMeter I2C Address
    CURRENT_ADDRESS = (0x28)
    ## Command Register
    COMMAND = 0x41
    ## Absolute Calibrated Current value Register. Will Return a signed integer value
    CAL = 0x43
    ## Relative Current value Register. Will Return a signed integer value
    REL = 0x45
    ## Reference Current value Register. Will Return a signed integer value
    REF = 0x47

    ## Initialize the class with the i2c address of your NXTCurrentMeter
    #  @param self The object pointer.
    #  @param current_address Address of your NXTCurrentMeter.
    def __init__(self,port, current_address = CURRENT_ADDRESS):
        #the DIST address
        mindsensors_i2c.__init__(self,port, current_address)

    ## Writes a value to the command register
    #  @param self The object pointer.
    #  @param cmd Value to write to the command register.
    def command(self, cmd):
        self.writeByte(self.COMMAND, cmd)

    ## Reads the absolute current value in milliAmps
    #  @param self The object pointer.
    def get_calibrated(self):
        return self.readIntegerSigned(self.CAL)
        
    ## Reads the relative current value in milliAmps
    #  @param self The object pointer.
    def get_relative(self):
        return self.readIntegerSigned(self.REL)
       

    ## Reads the reference current value in milliAmps
    #  @param self The object pointer.
    def get_reference(self):
        return self.readIntegerSigned(self.REF)
        

    ## Sets the reference current equal to the absolute current value
    #  @param self The object pointer.
    def set_reference(self):
        self.command(68)
        

## Volt: this class provides functions for NXTVoltMeter from mindsensors.com
#  for read and write operations.
class VOLT(mindsensors_i2c):

    ## Default VoltMeter I2C Address
    VOLT_ADDRESS = (0x26)
    ## Command Register
    COMMAND = 0x41
    ## Absolute Calibrated Voltage value Register. Will Return a signed integer value
    CAL = 0x43
    ## Relative Voltage value Register. Will Return a signed integer value
    REL = 0x45
    ## Reference Voltage value Register. Will Return a signed integer value
    REF = 0x47

    ## Initialize the class with the i2c address of your NXTVoltMeter
    #  @param self The object pointer.
    #  @param volt_address Address of your NXTVoltMeter.
    def __init__(self,port,  volt_address = VOLT_ADDRESS):
        #the DIST address
        mindsensors_i2c.__init__(self,port,  volt_address)

    ## Writes a value to the command register
    #  @param self The object pointer.
    #  @param cmd Value to write to the command register.
    def command(self, cmd):
        self.writeByte(self.COMMAND, cmd)

    ## Reads the absolute voltage of the NXTVoltMeter
    #  @param self The object pointer.
    def get_calibrated(self):
        return self.readIntegerSigned(self.CAL)
        

    ## Reads the relative voltage of the NXTVoltMeter
    #  @param self The object pointer.
    def get_relative(self):
        return self.readIntegerSigned(self.REL)
        

    ## Reads the reference voltage of the NXTVoltMeter
    #  @param self The object pointer.
    def get_reference(self):
        return self.readIntegerSigned(self.REF)
        

    ## Sets the reference voltage to the current absolute voltage value
    #  @param self The object pointer.
    def set_reference(self):
        self.command(68)


## PFMATE: this class provides motor control functions
class PFMATE(mindsensors_i2c):

    ## Default PFMate I2C Address
    PFMATE_ADDRESS = (0x48)

    ## Constants to specify Float Action
    PFMATE_FLOAT = 0
    ## Constants to specify Forward Motion
    PFMATE_FORWARD = 1
    ## Constants to specify Reverse Motion
    PFMATE_REVERSE = 2
    ## Constants to specify Brake Action
    PFMATE_BRAKE = 3
    ## Constants to specify Channel 1
    PFMATE_CHANNEL1 = 1
    ## Constants to specify Channel 2
    PFMATE_CHANNEL2 = 2
    ## Constants to specify Channel 3
    PFMATE_CHANNEL3 = 3
    ## Constants to specify Channel 4
    PFMATE_CHANNEL4 = 4

    ## Command Register
    PFMATE_COMMAND  = 0x41
    ## Channel Registewr
    PFMATE_CHANNEL  =   0x42
    ## Motor Define Register
    PFMATE_MOTORS    =   0x43
    ## Motor Operation A Register
    PFMATE_OPER_A =    0x44
    ## Motor Speed A Register
    PFMATE_SPEED_A   =   0x45
    ## Motor Operation B Register
    PFMATE_OPER_B =    0x46
    ## Motor Speed B Register
    PFMATE_SPEED_B   =   0x47

    ## Initialize the class with the i2c address of your PFMate
    #  @param self The object pointer.
    #  @param pfmate_address Address of your PFMate.
    def __init__(self,port,  pfmate_address = PFMATE_ADDRESS):
        #the PFMate address
        mindsensors_i2c.__init__(self,port,  pfmate_address)

    ## Writes a specified command on the command register of the PFMate
    #  @param self The object pointer.
    #  @param cmd The command you wish the PFMate to execute.
    def command(self, cmd):
       self.writeByte(self.PFMATE_COMMAND,cmd)

    ## Controls both motors
    #  @param self The object pointer.
    #  @param channel Communication channel to transmit to LEGO PF IR receiver (1-4).
    #  @param operationA Operation command for motor A (0 = Float, 1 = Forward, 2 = Reverse, 3 = Brake).
    #  @param speedA Speed of motor A (0-7).
    #  @param operationB Operation command for motor B (0 = Float, 1 = Forward, 2 = Reverse, 3 = Brake).
    #  @param speedB Speed of motor B (0-7).
    def controlBothMotors(self, channel, operationA, speedA, operationB, speedB):
        array = [channel, 0x00, operationA, speedA, operationB, speedB]
        self.writeArray(self.PFMATE_CHANNEL, array)
        time.sleep(.1)
        self.command('G')
        time.sleep(.1)

    ## Controls motor A
    #  @param self The object pointer.
    #  @param channel Communication channel to transmit to LEGO PF IR receiver (1-4).
    #  @param operationA Operation command for motor A (0 = Float, 1 = Forward, 2 = Reverse, 3 = Brake).
    #  @param speedA Speed of motor A (0-7).
    def controlMotorA(self, channel, operationA, speedA):
        array = [channel, 0x01, operationA, speedA]
        self.writeArray(self.PFMATE_CHANNEL, array)
        time.sleep(.1)
        self.command('G')
        time.sleep(.1)

    ## Controls motor B
    #  @param self The object pointer.
    #  @param channel Communication channel to transmit to LEGO PF IR receiver (1-4).
    #  @param operationB Operation command for motor B (0 = Float, 1 = Forward, 2 = Reverse, 3 = Brake).
    #  @param speedB Speed of motor B (0-7).
    def controlMotorB(self, channel, operationB, speedB):
        array = [channel, 0x02]
        array2 = [operationB, speedB]
        self.writeArray(self.PFMATE_CHANNEL, array)
        self.writeArray(self.PFMATE_OPER_B, array2)
        time.sleep(.1)
        self.command('G')
        time.sleep(.1)

## Sumoeyes : this class provides EV3Dev specific interface for the  Sumoeyes
#  for read and write operations.
class SUMOEYES(AnalogSensor):
    def __init__(self, port):
        self.port = port
       # self.sumo = I2CDevice(port)

    ## Read a  Sumoeyes sensor and return   .
    #  @param self The object pointer.
    def  read(self):
        results = self.voltage()
        return results

    ## Read a  Sumoeyes sensor in long range 
    #  @param self The object pointer.
    def  longrange(self):
        
        self.active()
        time.sleep(.05)
        results = self.voltage()
        return results

     ## Read a  Sumoeyes sensor in short range 
    #  @param self The object pointer.
    def  shortrange(self):
        
        self. passive()  
        time.sleep(.05)
        results = self.voltage()
        return results

  
## NXTSERVO: this class provides servo motor control functions
class NXTSERVO(mindsensors_i2c):

    ## Default NXTServo I2C Address
    NXTSERVO_ADDRESS = 0xB0
    ## Constant Voltage Multiplier
    NXTSERVO_VBATT_SCALER = 37
    ## Command Register
    NXTSERVO_COMMAND = 0x41
    ## Input Power Voltage Register
    NXTSERVO_VBATT = 0x62

    ## Initialize the class with the i2c address of your NXTServo
    #  @param self The object pointer.
    #  @param nxtservo_address Address of your NXTServo.
    def __init__(self,port, nxtservo_address = NXTSERVO_ADDRESS):
        mindsensors_i2c.__init__(self,port, nxtservo_address)
        self.command('S')

    ## Writes a specified command on the command register of the NXTServo
    #  @param self The object pointer.
    #  @param cmd The command you wish the NXTServo to execute.
    def command(self, cmd):
        self.writeByte(self.NXTSERVO_COMMAND, cmd)
        

    ## Reads NXTServo battery voltage in millivolts
    #  @param self The object pointer.
    def battVoltage(self):
        return self.readByte(self.NXTSERVO_VBATT) * self.NXTSERVO_VBATT_SCALER
        

    ## Sets the speed of a servo
    #  Has no effect on continuous rotation servos.
    #  @param self The object pointer.
    #  @param servoNumber The number of the servo to set its speed (1-8).
    #  @param speed The speed to set the servo (1-255).
    def setSpeed(self, servoNumber, speed):
        reg = 0x52 + servoNumber-1
        spd = speed % 256 # note: speed 0 is the same as speed 255
        self.writeByte(reg, spd)

    ## Sets the position of a servo
    #  @param self The object pointer.
    #  @param servoNumber The number of the servo to set its position (1-8).
    #  @param position The position to set the servo (1-255).
    def setPosition(self, servoNumber, position):
        reg = 0x5A + servoNumber-1
        pos = position % 256
        self.writeByte(reg, pos)

    ## Runs the specified servo to a specific position at a specified speed
    #  @param self The object pointer.
    #  @param servoNumber The number of the servo to move (1-8).
    #  @param position The position to set the servo (1-255).
    #  @param speed The speed to set the servo (1-255) (not used for continuous rotation servos).
    def runServo(self, servoNumber, position, speed = None):
        #self.setPosition(servoNumber, position)
        if speed: self.setSpeed(servoNumber, speed)
        self.setPosition(servoNumber, position)

    ## Store the current settings of the specified servo to initial/default settings (remembered when powered on)
    #  @param self The object pointer.
    #  @param servoNumber The number of the servo to save its settings (1-8).
    def storeInitial(self, servoNumber):
        self.command('I')
        self.command(servoNumber)

    ## Reset all servos to their default settings
    #  @param self The object pointer.
    def reset(self):
        self.command('S')

    ## Stop a specific servo
    #  This will also completely stop a continuous rotation servo, regardless of its neutral point.
    #  @param self The object pointer.
    #  @param servoNumber The number of the servo to stop (1-8).
    def stopServo(self, servoNumber):
        self.setPosition(servoNumber, 0)

    ## Sets the default neutral position of a user defined servo
    #  @param self The object pointer.
    #  @param servoNumber The number of the servo you wish to set to the default position.
    def setNeutral(self, servoNumber):
        self.command(73)
        time.sleep(0.1)
        self.command(servoNumber + 48)

    # warning: macro methods are untested

    ## Stop the onboard macro on the NXTServo
    #  @param self The object pointer.
    def haltMacro(self):
        self.command('H')

    ## Resume the onboard macro on the NXTServo
    #  @param self The object pointer.
    def resumeMacro(self):
        self.command('R')

    ## Go to a given EEPROM position
    #  This command re-initializes the macro environment
    #  @param self The object pointer.
    #  @param position The EEPROM position to go to
    def gotoEEPROM(self, position):
        self.command('G')
        self.command(position)

    ## Edit the onboard macro
    #  @param self The object pointer.
    def editMacro(self):
        self.command('E')
        self.command('m')

    ## Temporarily pause the running macro
    #  @param self The object pointer.
    def pauseMacro(self):
        self.command('P')        
        
        
        
        
## PSPNX: this class provides servo motor control functions
class PSPNX(mindsensors_i2c):

    ## Default PSPNX I2C Address
    PSPNX_ADDRESS = 0x02
    ## Command Register
    PSPNX_COMMAND = 0x41
    ## Button Set 1 Register
    PSPNX_BTN1 = 0x42
    ## Button Set 1 Register
    PSPNX_BTN2 = 0x43
    ## X-Left joystick Register
    PSPNX_LX = 0x44
    ## Y-Left joystick Register
    PSPNX_LX = 0x45
    ## X-Right joystick Register
    PSPNX_RX = 0x46
    ## Y-Right joystick Register
    PSPNX_LX = 0x47
    
    ## joystick button bitmask
    PSPNX_RA = 0x7F    #right Arrow
    PSPNX_DA = 0xbF    #down Arrow
    PSPNX_LA = 0xdF    #Left Arrow
    PSPNX_UA = 0xeF    #Up Arrow
    PSPNX_R3 = 0xFb    #R3
    PSPNX_L3 = 0xfd    #L3
    PSPNX_SQ = 0x7F    #Square
    PSPNX_X = 0xbF     #Cross
    PSPNX_CR = 0xdF    #Circle
    PSPNX_TR = 0xeF    #Triangle
    PSPNX_R1 = 0xf7    #R1
    PSPNX_L1 = 0xFb    #L1
    PSPNX_R2 = 0xFd    #R2
    PSPNX_L3 = 0xfd    #L3
    

    ## Initialize the class with the i2c address of your PSPNX
    #  @param self The object pointer.
    #  @param PSPNX_address Address of your PSPNX.
    def __init__(self,port, PSPNX_address = PSPNX_ADDRESS):
        mindsensors_i2c.__init__(self,port, PSPNX_address)
        self.command('I')

    ## Writes a specified command on the command register of the PSPNX
    #  @param self The object pointer.
    #  @param cmd The command you wish the PSPNX to execute.
    def command(self, cmd):
        self.writeByte(self.PSPNX_COMMAND, cmd)
        

    ## Reads PSPNX Button Set 1 Register
    #  @param self The object pointer.
    def Button_Set_1 (self):
        return self.readByte(self.PSPNX_BTN1)
        
    ## Reads PSPNX Button Set 2 Register
    #  @param self The object pointer.
    def Button_Set_2 (self):
        return self.readByte(self.PSPNX_BTN2)
        
    ## Reads PSPNX X-Left joystick Register
    #  @param self The object pointer.
    def X_LEFT (self):
        return self.readByte(self.PSPNX_LX)
        
    ## Reads PSPNX Y-Left joystick Register
    #  @param self The object pointer.
    def Y_LEFT (self):
        return self.readByte(self.PSPNX_LY)    
        
        
    ## Reads PSPNX X-Right joystick Register
    #  @param self The object pointer.
    def X_RIGHT (self):
        return self.readByte(self.PSPNX_RX)
        
    ## Reads PSPNX Y-Right joystick Register
    #  @param self The object pointer.
    def Y_RIGHT (self):
        return self.readByte(self.PSPNX_RY)     
        
    ## Returns if button from Set 1 is pressed 
    #  @param self The object pointer.
    def IS_PRESSED1 (self,button):
        return self.Button_Set_1()&button    
        
    ## Returns if button from Set 1 is pressed 
    #  @param self The object pointer.
    def IS_PRESSED2 (self,button):
        return self.Button_Set_2()&button               
        return self.Button_Set_2()&button    

## EV3Light : this class provides EV3Dev specific interface for the
#  EV3Light sensor:
#  
#
class EV3Light(mindsensors_i2c):
    ## Default I2C Address
    EV3Light_ADDRESS = 0x2A
    ## Command Register
    EV3Light_COMMAND = 0x41 
    EV3Light_RGB = 0x42
    EV3Light_Update = 0x46
    EV3Light_LED_Index = 0x46
    

    ## Initialize the class with the i2c address of your device
    #  @param self The object pointer.
    #  @param port The Ev3 port.
    #  @param address Address of your device
    #  @remark
    def __init__(self, port,i2c_address=EV3Light_ADDRESS):
        mindsensors_i2c.__init__(self,port, i2c_address)
    
    ## Writes a specified command on the command register of the EV3Light
    #  @param self The object pointer.
    #  @param cmd The command you wish the EV3Light to execute.
    def command(self, cmd):
       self.writeByte(self.EV3Light_COMMAND,cmd)

    ## Controls single LED
    #  @param self The object pointer.
    #  @param led id or mode, if led ==0 the mode is V1
    #  @param R Red intensity.
    #  @param G Green intensity.
    #  @param B Blue intensity.
    
    def lightled(self, led, R,G,B):
       
        if led==0 :  self.command('S')
        else :self.command('I') 
        time.sleep(.1)
        array = [R,G,B,led,0x1]
        self.writeArray(self.EV3Light_RGB, array)
        time.sleep(.1)
        
    def clear(self):
        self.lightled(0,0,0,0)   


## Numericpad : this class provides EV3Dev specific interface for the
#  Numericpad sensor:
#  
#
class NumericPad(mindsensors_i2c):
    ## Default I2C Address
    NumericPad_ADDRESS = 0xB4
    ## status Register
    NumericPad_KEY_STATUS_REG = 0x00
    
    

    ## Initialize the class with the i2c address of your device
    #  @param self The object pointer.
    #  @param port The PiStorms bank.
    #  @param address Address of your device
    #  @remark
    def __init__(self, port,i2c_address=NumericPad_ADDRESS):
        mindsensors_i2c.__init__(self,port, i2c_address)
        self.setup()
    

    ## Setup NumericPad
     #  @param self The object pointer.
    
    def setup(self):
       
        array = [0x0F, 0x0A, 0x0F, 0x0A, 0x0F, 0x0A, 0x0F, 0x0A, 0x0F]
        self.writeArray(0x41, array)
        array = [ 0x0A, 0x0F, 0x0A, 0x0F, 0x0A, 0x0F, 0x0A, 0x0F]
        self.writeArray(0x4A, array)
        array = [  0x0A, 0x0F, 0x0A, 0x0F, 0x0A, 0x0F, 0x0A, 0x0F]
        self.writeArray(0x52, array)
        array = [  0x0b, 0x20, 0x0C]
        self.writeArray(0x5C, array)
        array = [ 0x01, 0x01, 0x00, 0x00, 0x01, 0x01, 0xFF, 0x02]
        self.writeArray(0x2B, array)
        array = [ 0x0b]
        self.writeArray(0x7B, array)
        array = [  0x9C, 0x65, 0x8C]
        self.writeArray(0x7D, array)
        time.sleep(.1)

    def  GetKeysPressed(self)  :
          return self.readInteger(0x00)& 0x0FFF
        
    

    def DecodeKeys(self,KeyBits) :        
        keyMap = [ '4', '1', '7', '*', '5', '2', '8', '0', '3', '6', '9', '#' ]
        keypressed=[]
        if KeyBits&0x0800:keypressed.append("4")
        if KeyBits&0x0400:keypressed.append("1")
        if KeyBits&0x0200:keypressed.append("7")
        if KeyBits&0x0100:keypressed.append("*")
        if KeyBits&0x080:keypressed.append("5")
        if KeyBits&0x040:keypressed.append("2")
        if KeyBits&0x020:keypressed.append("8")
        if KeyBits&0x010:keypressed.append("0")
        if KeyBits&0x008:keypressed.append("3")
        if KeyBits&0x004:keypressed.append("6")
        if KeyBits&0x002:keypressed.append("9")
        if KeyBits&0x001:keypressed.append("#")
        return     keypressed
        
## MMX: this class provides motor control functions for use with NXTMMX

class MMX(mindsensors_i2c):

    ## Default NXTMMX I2C Address
    MMX_ADDRESS = (0x06)
    ## Constant Voltage Multipler
    MMX_VOLTAGE_MULTIPLIER = 37

    ## Constant to specify Motor 1
    MMX_Motor_1        =        0x01
    ## Constant to specify Motor 2
    MMX_Motor_2        =        0x02
    ## Constant to specify both Motors
    MMX_Motor_Both     =        0x03

    ## Constant to specify Float Action
    MMX_Next_Action_Float   =   0x00
    ## Constant to specify Brake Action
    MMX_Next_Action_Brake   =   0x01
    ## Constant to specify Hold Action
    MMX_Next_Action_BrakeHold = 0x02

    ## Constant to specify Forward Motion
    MMX_Direction_Forward   =   0x01
    ## Constant to specify Reverse Motion
    MMX_Direction_Reverse   =   0x00

    ## Constant to specify Relative Encoder Position
    MMX_Move_Relative = 0x01
    ## Constant to specify Absolute Encoder Position
    MMX_Move_Absolute = 0x00

    ## Constant to Wait for Next Action
    MMX_Completion_Wait_For   =  0x01
    ## Constant to NOT Wait for Next Action
    MMX_Completion_Dont_Wait  = 0x00

    ## Constant for commonly used Full Speed value
    MMX_Speed_Full = 90
    ## Constant for commonly used Moderate Speed value
    MMX_Speed_Medium = 60
    ## Constant for commonly used Slow Speed value
    MMX_Speed_Slow  = 25

    ## Constant to specify Speed bit of Motor Control byte
    MMX_CONTROL_SPEED  =    0x01
    ## Constant to specify Ramp bit of Motor Control byte
    MMX_CONTROL_RAMP   =    0x02
    ## Constant to specify Relative bit of Motor Control byte
    MMX_CONTROL_RELATIVE =  0x04
    ## Constant to specify Tacho bit of Motor Control byte
    MMX_CONTROL_TACHO  =    0x08
    ## Constant to specify Brake bit of Motor Control byte
    MMX_CONTROL_BRK    =    0x10
    ## Constant to specify On bit of Motor Control byte
    MMX_CONTROL_ON     =    0x20
    ## Constant to specify Time bit of Motor Control byte
    MMX_CONTROL_TIME  =     0x40
    ## Constant to specify Go bit of Motor Control byte
    MMX_CONTROL_GO   =      0x80

    ## Command Register
    MMX_COMMAND = 0x41
    ## Motor 1 Encoder Target Register
    MMX_SETPT_M1  =   0x42
    ## Motor 1 Speed Register
    MMX_SPEED_M1  =   0x46
    ## Motor 1 Time Register
    MMX_TIME_M1  =    0x47
    ## Motor 1 Motor Command B Register
    MMX_CMD_B_M1  =   0x48
    ## Motor 1 Motor Command A Register
    MMX_CMD_A_M1  =   0x49
    ## Motor 2 Encoder Target Register
    MMX_SETPT_M2  =   0x4A
    ## Motor 2 Speed Register
    MMX_SPEED_M2  =   0x4E
    ## Motor 2 Time Register
    MMX_TIME_M2   =   0x4F
    ## Motor 1 Motor Command B Register
    MMX_CMD_B_M2  =   0x50
    ## Motor 1 Motor Command A Register
    MMX_CMD_A_M2  =   0x51
    ## Motor 1 Encoder Position Register. Will return a long singed integer value
    MMX_POSITION_M1 = 0x62
    ## Motor 2 Encoder Position Register. Will return a long singed integer value
    MMX_POSITION_M2 = 0x66
    ## Motor 1 Status Register. Will return a byte value
    MMX_STATUS_M1   = 0x72
    ## Motor 2 Status Register. Will return a byte value
    MMX_STATUS_M2   = 0x73
    ## Motor 1 Tasks Register. Will return a byte value
    MMX_TASKS_M1    = 0x76
    ## Motor 2 Tasks Register. Will return a byte value
    MMX_TASKS_M2   =  0x77
    ## Position Kp Register
    MMX_P_Kp  =  0x7A
    ## Position Ki Register
    MMX_P_Ki  =  0x7C
    ## Position Kd Register
    MMX_P_Kd  =  0x7E
    ## Speed Kp Register
    MMX_S_Kp  =  0x80
    ## Speed Ki Register
    MMX_S_Ki  =  0x82
    ## Speed Kd Register
    MMX_S_Kd  =  0x84
    ## Pass Count Register
    MMX_PASSCOUNT  =  0x86
    ## Pass Tolerance Register
    MMX_PASSTOLERANCE  =  0x87

    ## Initialize the class with the i2c address of your NXTMMX
    #  @param self The object pointer.
    #  @param mmx_address Address of your NXTMMX.
    def __init__(self, port, mmx_address = MMX_ADDRESS):
        #the NXTMMX address
        mindsensors_i2c.__init__(self,port, mmx_address)

    ## Writes a specified command on the command register of the NXTMMX
    #  @param self The object pointer.
    #  @param cmd The command you wish the NXTMMX to execute.
    def command(self, cmd):
       self.writeByte(self.MMX_COMMAND, int(cmd))

    ## Reset the both motor encoders of the NXTMMX
    #  @param self The object pointer.
    def resPos(self):
       self.command(82)

    ## Reads the battery voltage
    #  @param self The object pointer.
    def battVoltage(self):
        try:
            return self.readByte(self.MMX_COMMAND) * self.MMX_VOLTAGE_MULTIPLIER #37
        except:
            print ("Error: Could not read voltage")
            return ""

    ## Reads the encoder position of the specified motor
    #  @param self The object pointer.
    #  @param motor_number Number of the motor you wish to read.
    def pos(self, motor_number):
        try:
            if motor_number == 1 :
                return self.readLongSigned(self.MMX_POSITION_M1)
            if motor_number == 2 :
                return self.readLongSigned(self.MMX_POSITION_M2)
        except:
            print ("Error: Could not read encoder position")
            return ""

    ## Run the motor(s) at a set speed for an unlimited duration
    #  @param self The object pointer.
    #  @param motor_number Number of the motor(s) you wish to turn.
    #  @param speed The speed at which you wish to turn the motor(s).
    def setSpeed( self, motor_number, speed ):

        ctrl = 0
        ctrl |= self.MMX_CONTROL_SPEED
        ctrl |= self.MMX_CONTROL_BRK

        if ( motor_number != self.MMX_Motor_Both ):
            ctrl |= self.MMX_CONTROL_GO
        if ( (motor_number & 0x01) != 0 ):
            array = [speed, 0, 0, ctrl]
            self.writeArray( self.MMX_SPEED_M1, array)
        if ( (motor_number & 0x02) != 0 ):
            array = [speed, 0, 0, ctrl]
            self.writeArray( self.MMX_SPEED_M2, array)
        if ( motor_number == self.MMX_Motor_Both ) :
            self.writeByte(self.MMX_COMMAND, 83)

    ### @cond Doxygen_ignore_this
    ## Stops the specified motor(s)
    #  @param self The object pointer.
    #  @param motor_number Number of the motor(s) you wish to stop.
    #  @param next_action How you wish to stop the motor(s).
    def MMX_Stop( self, motor_number, next_action ):

        if ( next_action == self.MMX_Next_Action_Brake or next_action == self.MMX_Next_Action_BrakeHold ):
            if (motor_number == self.MMX_Motor_1):
                self.writeByte(self.MMX_COMMAND, 65)
            if (motor_number == self.MMX_Motor_2):
                self.writeByte(self.MMX_COMMAND, 66)
            if (motor_number == self.MMX_Motor_Both):
                self.writeByte(self.MMX_COMMAND, 67)
        else:
            if (motor_number == self.MMX_Motor_1):
                self.writeByte(self.MMX_COMMAND, 97)
            if (motor_number == self.MMX_Motor_2):
                self.writeByte(self.MMX_COMMAND, 98)
            if (motor_number == self.MMX_Motor_Both):
                self.writeByte(self.MMX_COMMAND, 99)

    def status(self, motor_number):
        if (motor_number == 1):
            return self.readByte(self.MMX_STATUS_M1)
        if (motor_number == 2):
            return self.readByte(self.MMX_STATUS_M2)

    def statusBit(self, motor_number, bitno = 0):
        return (self.status(motor_number) >> bitno) & 1
    ### @endcond

    ## Stop the motor with abruptly with brake
    #  @param self The object pointer.
    #  @param motor_number Number of the motor(s) you wish to brake.
    def brake(self, motor_number):
        self.MMX_Stop(motor_number, self.MMX_Next_Action_Brake)

    ## Stop the motor smoothly with float
    #  @param self The object pointer.
    #  @param motor_number Number of the motor(s) you wish to float.
    def float(self, motor_number):
        self.MMX_Stop(motor_number, self.MMX_Next_Action_Float)

    ## Stop the motor abruptly and hold the current position
    #  @param self The object pointer.
    #  @param motor_number Number of the motor(s) you wish to hold.
    def hold(self, motor_number):
        self.MMX_Stop(motor_number, self.MMX_Next_Action_BrakeHold)

    ## Check if the motor is running
    #  @param self The object pointer.
    #  @param motor_number Number of the motor(s) you wish to check.
    def isBusy(self, motor_number):
        return self.statusBit(motor_number, 0) == 1 or self.statusBit(motor_number, 1) == 1 or self.statusBit(motor_number, 3) == 1 or self.statusBit(motor_number, 6) == 1

    ## Wait until the motor is no longer running
    #  @param self The object pointer.
    #  @param motor_number Number of the motor(s) you wish to wait for.
    #  @param timeout The timeout value as a factor of 10ms.
    def waitUntilNotBusy(self, motor_number, timeout=-1):
        while(self.isBusy(motor_number)):
            time.sleep(.01)
            timeout -= 1
            if(timeout == 0):
                return 1
            if(timeout <-5):
                timeout = -1
            pass
        return 0

    ## Check if the motor is stalled
    #  @param motor_number Number of the motor(s) you wish to check.
    #  @param self The object pointer.
    def isStalled(self, motor_number):
        return self.statusBit(motor_number, 7) == 1

    ## Check if the motor is overloaded
    #  @param self The object pointer.
    def isOverloaded(self):
        return self.statusBit(motor_number, 5) == 1

    ## Run the motor for a specific time in seconds
    #  @param self The object pointer.
    #  @param motor_number Number of the motor(s) you wish to turn.
    #  @param secs The number of seconds to run the motor.
    #  @param speed The speed at which to turn the motor.
    #  @param brakeOnCompletion Choose to brake or float the motor upon completion with True (brake) or False (float).
    #  @param waitForCompletion Wait until the motor is finished running before continuing the program.
    def runSecs( self, motor_number, secs, speed, brakeOnCompletion = False, waitForCompletion = False ):
        ctrl = 0
        ctrl |= self.MMX_CONTROL_SPEED
        ctrl |= self.MMX_CONTROL_TIME

        if ( brakeOnCompletion == True ):
            ctrl |= self.MMX_CONTROL_BRK
        if ( motor_number != self.MMX_Motor_Both ):
            ctrl |= self.MMX_CONTROL_GO
        if ( (motor_number & 0x01) != 0 ):
            array = [speed, secs, 0, ctrl]
            self.writeArray( self.MMX_SPEED_M1, array)
        if ( (motor_number & 0x02) != 0 ) :
            array = [speed, secs, 0, ctrl]
            self.writeArray( self.MMX_SPEED_M2, array)
        if ( motor_number == self.MMX_Motor_Both ) :
            self.writeByte(self.MMX_COMMAND, 83)
        if ( waitForCompletion == True ):
            time.sleep(0.050)  # this delay is required for the status byte to be available for reading.
            self.MMX_WaitUntilTimeDone(motor_number)

    ### @cond
    ## Waits until the specified time for the motor(s) to run is completed
    #  @param self The object pointer.
    #  @param motor_number Number of the motor(s) to wait for.
    def MMX_WaitUntilTimeDone(self,motor_number):
        while self.MMX_IsTimeDone(motor_number) != True:
            time.sleep(0.050)

    ## Checks to ensure the specified time for the motor(s) to run is completed.
    #  @param self The object pointer.
    #  @param motor_number Number of the motor(s) to check.
    def  MMX_IsTimeDone(self, motor_number):
        if ( motor_number == self.MMX_Motor_1 ):
            result = self.readByte(self.MMX_STATUS_M1)
            # look for the time bit to be zero.
            if (( result & 0x40 ) == 0 ):
                return True
        elif ( motor_number == self.MMX_Motor_2 ) :
            result = self.readByte(self.MMX_STATUS_M2)
            # look for the time bit to be zero.
            if (( result & 0x40 ) == 0 ):
                return True
        elif ( motor_number == self.MMX_Motor_Both ):
            result = self.readByte(self.MMX_STATUS_M1)
            result2 = self.readByte(self.MMX_STATUS_M2)
            # look for both time bits to be zero
            if (((result & 0x40) == 0) &((result2 & 0x40) == 0) ):
                return True
        else :
            return False
    ### @endcond

    ## Run the motor for a specific amount of degrees
    #  @param self The object pointer.
    #  @param motor_number Number of the motor(s) you wish to turn.
    #  @param degs The number of degrees to run the motor(s).
    #  @param speed The speed at which to turn the motor(s).
    #  @param brakeOnCompletion Choose to brake or float the motor upon completion with True (brake) or False (float).
    #  @param holdOnCompletion Choose to hold the motor position upon completion with True (hold) or False (release).
    #  @param waitForCompletion Tells the program when to handle the next line of code.
    def runDegs(self, motor_number, degs, speed, brakeOnCompletion = False, holdOnCompletion = False, waitForCompletion = False):
        ctrl = 0
        ctrl |= self.MMX_CONTROL_SPEED
        ctrl |= self.MMX_CONTROL_TACHO
        ctrl |= self.MMX_CONTROL_RELATIVE

        d = degs
        t4 = int(d/0x1000000)
        t3 = int((d%0x1000000)/0x10000)
        t2 = int(((d%0x1000000)%0x10000)/0x100)
        t1 = int(((d%0x1000000)%0x10000)%0x100)

        if ( brakeOnCompletion == True ):
            ctrl |= self.MMX_CONTROL_BRK
        if ( holdOnCompletion == True ):
            ctrl |= self.MMX_CONTROL_BRK
            ctrl |= self.MMX_CONTROL_ON
        if ( motor_number != self.MMX_Motor_Both ):
            ctrl |= self.MMX_CONTROL_GO
        if ( (motor_number & 0x01) != 0 ):
            array = [t1, t2, t3, t4, speed, 0, 0, ctrl]
            self.writeArray(self.MMX_SETPT_M1, array)
        if ( (motor_number & 0x02) != 0 ) :
            array = [t1, t2, t3, t4, speed, 0, 0, ctrl]
            self.writeArray(self.MMX_SETPT_M2, array)
        if ( motor_number == self.MMX_Motor_Both ) :
            self.writeByte(self.MMX_COMMAND, 83)
        if ( waitForCompletion == True ):
            time.sleep(0.050)  # this delay is required for the status byte to be available for reading.
            self.MMX_WaitUntilTachoDone(motor_number)

    ## Run the motor for a specific amount of rotations
    #  @param self The object pointer.
    #  @param motor_number Number of the motor(s) you wish to turn.
    #  @param rotations The number of rotations to run the motor(s).
    #  @param speed The speed at which to turn the motor(s).
    #  @param brakeOnCompletion Choose to brake or float the motor upon completion with True (brake) or False (float).
    #  @param holdOnCompletion Choose to hold the motor position upon completion with True (hold) or False (release).
    #  @param waitForCompletion Tells the program when to handle the next line of code.
    def runRotations(self, motor_number, rotations, speed, brakeOnCompletion = False, holdOnCompletion = False, waitForCompletion = False):
        ctrl = 0
        ctrl |= self.MMX_CONTROL_SPEED
        ctrl |= self.MMX_CONTROL_TACHO
        ctrl |= self.MMX_CONTROL_RELATIVE

        d = rotations * 360

        t4 = int(d/0x1000000)
        t3 = int((d%0x1000000)/0x10000)
        t2 = int(((d%0x1000000)%0x10000)/0x100)
        t1 = int(((d%0x1000000)%0x10000)%0x100)

        if ( brakeOnCompletion == True ):
            ctrl |= self.MMX_CONTROL_BRK
        if ( holdOnCompletion == True ):
            ctrl |= self.MMX_CONTROL_BRK
            ctrl |= self.MMX_CONTROL_ON
        if ( motor_number != self.MMX_Motor_Both ):
            ctrl |= self.MMX_CONTROL_GO
        if ( (motor_number & 0x01) != 0 ):
            array = [t1, t2, t3, t4, speed, 0, 0, ctrl]
            print(array)
            self.writeArray(self.MMX_SETPT_M1, array)
        if ( (motor_number & 0x02) != 0 ) :
            array = [t1, t2, t3, t4, speed, 0, 0, ctrl]
            print(array)
            self.writeArray(self.MMX_SETPT_M2, array)
        if ( motor_number == self.MMX_Motor_Both ) :
            self.writeByte(self.MMX_COMMAND, 83)
        if ( waitForCompletion == True ):
            time.sleep(0.050)  # this delay is required for the status byte to be available for reading.
            self.MMX_WaitUntilTachoDone(motor_number)

    ## Run the motor for a specific amount of rotations
    #  @param self The object pointer.
    #  @param motor_number Number of the motor(s) you wish to turn.
    #  @param pos The encoder value to which to run the motor(s).
    #  @param speed The speed at which to turn the motor(s).
    #  @param brakeOnCompletion Choose to brake or float the motor upon completion with True (brake) or False (float).
    #  @param holdOnCompletion Choose to hold the motor position upon completion with True (hold) or False (release).
    #  @param waitForCompletion Tells the program when to handle the next line of code.
    def runEncoderPos(self, motor_number, pos, speed, brakeOnCompletion = False, holdOnCompletion = False, waitForCompletion = False):
        ctrl = 0
        ctrl |= self.MMX_CONTROL_SPEED
        ctrl |= self.MMX_CONTROL_TACHO
        d = pos

        t4 = int(d/0x1000000)
        t3 = int((d%0x1000000)/0x10000)
        t2 = int(((d%0x1000000)%0x10000)/0x100)
        t1 = int(((d%0x1000000)%0x10000)%0x100)

        if ( brakeOnCompletion == True ):
            ctrl |= self.MMX_CONTROL_BRK
        if ( holdOnCompletion == True ):
            ctrl |= self.MMX_CONTROL_BRK
            ctrl |= self.MMX_CONTROL_ON
        if ( motor_number != self.MMX_Motor_Both ):
            ctrl |= self.MMX_CONTROL_GO
        if ( (motor_number & 0x01) != 0 ):
            array = [t1, t2, t3, t4, speed, 0, 0, ctrl]
            self.writeArray(self.MMX_SETPT_M1, array)
        if ( (motor_number & 0x02) != 0 ):
            array = [t1, t2, t3, t4, speed, 0, 0, ctrl]
            self.writeArray(self.MMX_SETPT_M2, array)
        if ( motor_number == self.MMX_Motor_Both ) :
            self.writeByte(self.MMX_COMMAND, 83)
        if ( waitForCompletion == True ):
            time.sleep(0.050)  # this delay is required for the status byte to be available for reading.
            self.MMX_WaitUntilTachoDone(motor_number)

    ### @cond
    ## Waits until the specified tacheomter count for the motor(s) to run is reached.
    #  @param self The object pointer.
    #  @param motor_number Number of the motor(s) to wait for.
    def MMX_WaitUntilTachoDone(self,motor_number):
        while self.MMX_IsTachoDone(motor_number) != True:
            time.sleep(0.050)

    ## Checks to ensure the specified tacheomter count for the motor(s) to run is reached.
    #  @param self The object pointer.
    #  @param motor_number Number of the motor(s) to check.
    def  MMX_IsTachoDone(self, motor_number):
        if ( motor_number == self.MMX_Motor_1 ):
            result = self.readByte(self.MMX_STATUS_M1)
            # look for the time bit to be zero.
            if (( result & 0x08 ) == 0 ):
                return True
        elif ( motor_number == self.MMX_Motor_2 ) :
            result = self.readByte(self.MMX_STATUS_M2)
            # look for the time bit to be zero.
            if (( result & 0x08 ) == 0 ):
                return True
        elif ( motor_number == self.MMX_Motor_Both ):
            result = self.readByte(self.MMX_STATUS_M1)
            result2 = self.readByte(self.MMX_STATUS_M2)
            # look for both time bits to be zero
            if (((result & 0x08) == 0) & ((result2 & 0x08) == 0) ):
                return True
        else :
            return False
    ### @endcond

    ## Writes user specified values to the PID control registers
    #  @param self The object pointer.
    #  @param Kp_tacho Proportional-gain of the encoder position of the motor.
    #  @param Ki_tacho Integral-gain of the encoder position of the motor.
    #  @param Kd_tacho Derivative-gain of the encoder position of the motor.
    #  @param Kp_speed Proportional-gain of the speed of the motor.
    #  @param Ki_speed Integral-gain of the speed of the motor.
    #  @param Kd_speed Derivative-gain of the speed of the motor.
    #  @param passcount The number of times the encoder reading should be within tolerance.
    #  @param tolerance The tolerance (in ticks) for encoder positioning .
    def SetPerformanceParameters(self, Kp_tacho, Ki_tacho, Kd_tacho, Kp_speed, Ki_speed, Kd_speed, passcount, tolerance):
        Kp_t1 = Kp_tacho%0x100
        Kp_t2 = Kp_tacho/0x100
        Ki_t1 = Ki_tacho%0x100
        Ki_t2 = Ki_tacho/0x100
        Kd_t1 = Kd_tacho%0x100
        Kd_t2 = Kd_tacho/0x100
        Kp_s1 = Kp_speed%0x100
        Kp_s2 = Kp_speed/0x100
        Ki_s1 = Ki_speed%0x100
        Ki_s2 = Ki_speed/0x100
        Kd_s1 = Kd_speed%0x100
        Kd_s2 = Kd_speed/0x100
        print ("Kp_t1: " + str(Kp_t1))
        print ("Kp_t2: " + str(Kp_t2))
        print ("Ki_t1: " + str(Ki_t1))
        print ("Ki_t2: " + str(Ki_t2))
        print ("Kd_t1: " + str(Kd_t1))
        print ("Kd_t2: " + str(Kd_t2))
        print ("Kp_s1: " + str(Kp_s1))
        print ("Kp_s2: " + str(Kp_s2))
        print ("Ki_s1: " + str(Ki_s1))
        print ("Ki_s2: " + str(Ki_s2))
        print ("Kd_s1: " + str(Kd_s1))
        print ("Kd_s2: " + str(Kd_s2))
        passcount = passcount
        tolerance = tolerance
        array = [Kp_t1 , Kp_t2 , Ki_t1, Ki_t2, Kd_t1, Kd_t2, Kp_s1, Kp_s2, Ki_s1, Ki_s2, Kd_s1, Kd_s2, passcount, tolerance]
        self.writeArray(self.MMX_P_Kp, array)

    ## Reads the values of the PID control registers
    #  @param self The object pointer.
    def ReadPerformanceParameters(self):
        try:
            print ("Pkp: " + str(self.readInteger(self.MMX_P_Kp)))
            print ("Pki: " + str(self.readInteger(self.MMX_P_Ki)))
            print ("Pkd: " + str(self.readInteger(self.MMX_P_Kd)))
            print ("Skp: " + str(self.readInteger(self.MMX_S_Kp)))
            print ("Ski: " + str(self.readInteger(self.MMX_S_Ki)))
            print ("Skd: " + str(self.readInteger(self.MMX_S_Kd)))
            print ("Passcount: " + str(self.MMX_PASSCOUNT))
            print ("Tolerance: " + str(self.MMX_PASSTOLERANCE))
        except:
            print ("Error: Could not read PID values")
            return ""


        