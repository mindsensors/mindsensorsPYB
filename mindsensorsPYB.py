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
    #  print "Color: " + str(b.color)
    #  print "Left: " + str(b.left)
    #  print "Top: " + str(b.top)
    #  print "Right: " + str(b.right)
    #  print "Bottom: " + str(b.bottom)
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
