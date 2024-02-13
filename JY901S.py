# coding: UTF-8
import time
import threading
import serial
import math
import os

class WitProtocolResolver():
    TempBytes=[]        
    PackSize = 11        
    gyroRange = 2000.0   
    accRange = 16.0     
    angleRange = 180.0 
    TempFindValues=[]

    def setConfig(self, deviceModel):
        pass

    def sendData(self, sendData, deviceModel):
        success_bytes = deviceModel.serialPort.write(sendData)
        
    def passiveReceiveData(self, data, deviceModel):
        global TempBytes
        for val in data:
            self.TempBytes.append(val)
            if (self.TempBytes[0]!=0x55):                   
                del self.TempBytes[0]                       
                continue
            if (len(self.TempBytes)>1):
                if (((self.TempBytes[1] - 0x50 >=0 and self.TempBytes[1] - 0x50 <=11) or self.TempBytes[1]==0x5f)==False):   
                    del self.TempBytes[0]                   
                    continue
            if (len(self.TempBytes) == self.PackSize):      
                CheckSum = 0                                
                for i in range(0,self.PackSize-1):
                    CheckSum+=self.TempBytes[i]
                if (CheckSum&0xff==self.TempBytes[self.PackSize-1]):  
                    if (self.TempBytes[1] == 0x50):                   
                        self.get_chiptime(self.TempBytes, deviceModel) 
                        # print("Chiptime: ", deviceModel.getDeviceData("Chiptime"))
                    elif (self.TempBytes[1]==0x51):                    
                        self.get_acc(self.TempBytes,deviceModel)  
                        # print("Acceleration: ", deviceModel.getDeviceData("accX"),deviceModel.getDeviceData("accY"),deviceModel.getDeviceData("accZ"))     
                    elif(self.TempBytes[1]==0x52):                    
                        self.get_gyro(self.TempBytes,deviceModel) 
                        # print("Angular velocity: ", deviceModel.getDeviceData("gyroX"),deviceModel.getDeviceData("gyroY"),deviceModel.getDeviceData("gyroZ"))    
                    elif(self.TempBytes[1]==0x53):                    
                        self.get_angle(self.TempBytes,deviceModel)  
                        # print("Angle: ", deviceModel.getDeviceData("angleX"),deviceModel.getDeviceData("angleY"),deviceModel.getDeviceData("angleZ"))  
                    elif(self.TempBytes[1]==0x54):                    
                        self.get_mag(self.TempBytes, deviceModel)   
                        self.calculate_heading(deviceModel)  
                        deviceModel.dataProcessor.onUpdate(deviceModel)
                        # print("Magnetic field: ", deviceModel.getDeviceData("magX"),deviceModel.getDeviceData("magY"),deviceModel.getDeviceData("magZ")) 
                    elif(self.TempBytes[1]==0x57):                    
                        self.get_lonlat(self.TempBytes, deviceModel)     
                        deviceModel.dataProcessor.onUpdate(deviceModel)
                        # print("Longitude: ", deviceModel.getDeviceData("lon"), " Latitude: ", deviceModel.getDeviceData("lat"))
                    elif(self.TempBytes[1]==0x58):                    
                        self.get_gps(self.TempBytes, deviceModel)     
                        deviceModel.dataProcessor.onUpdate(deviceModel) 
                        # print("Height: ", deviceModel.getDeviceData("Height"), " Yaw: ", deviceModel.getDeviceData("Yaw"), " Speed: ", deviceModel.getDeviceData("Speed"))
                    elif(self.TempBytes[1]==0x59):                    
                        self.get_four_elements(self.TempBytes, deviceModel)     
                        deviceModel.dataProcessor.onUpdate(deviceModel) 
                        # print("Quaternion: ", deviceModel.getDeviceData("q1"),deviceModel.getDeviceData("q2"),deviceModel.getDeviceData("q3"),deviceModel.getDeviceData("q4"))
                    elif(self.TempBytes[1]==0x5f):           
                        self.get_find(self.TempBytes,deviceModel)
                        # print("Find: ", self.TempFindValues)
                    self.TempBytes=[]                        
                else:                                        
                    del self.TempBytes[0]                    

    def get_readbytes(self,regAddr):
        return [0xff, 0xaa,0x27, regAddr & 0xff, regAddr >> 8]

    def get_writebytes(self,regAddr,sValue):
        return [0xff, 0xaa, regAddr, sValue & 0xff, sValue >> 8]

    def get_acc(self,datahex, deviceModel):
        axl = datahex[2]
        axh = datahex[3]
        ayl = datahex[4]
        ayh = datahex[5]
        azl = datahex[6]
        azh = datahex[7]

        tempVal = (datahex[9] << 8 | datahex[8])
        acc_x = (axh << 8 | axl) / 32768.0 * self.accRange * 9.8
        acc_y = (ayh << 8 | ayl) / 32768.0 * self.accRange * 9.8
        acc_z = (azh << 8 | azl) / 32768.0 * self.accRange * 9.8
        # if acc_x >= self.accRange:
        #     acc_x -= 2 * self.accRange
        # if acc_y >= self.accRange:
        #     acc_y -= 2 * self.accRange
        # if acc_z >= self.accRange:
        #     acc_z -= 2 * self.accRange

        deviceModel.setDeviceData("accX", round(acc_x, 4))    
        deviceModel.setDeviceData("accY", round(acc_y, 4))   
        deviceModel.setDeviceData("accZ", round(acc_z, 4))
        temperature = round(tempVal / 100.0, 2)                                           
        deviceModel.setDeviceData("temperature", temperature)                           
        
    def get_gyro(self,datahex, deviceModel):
        wxl = datahex[2]
        wxh = datahex[3]
        wyl = datahex[4]
        wyh = datahex[5]
        wzl = datahex[6]
        wzh = datahex[7]

        gyro_x = (wxh << 8 | wxl) / 32768.0 * self.gyroRange
        gyro_y = (wyh << 8 | wyl) / 32768.0 * self.gyroRange
        gyro_z = (wzh << 8 | wzl) / 32768.0 * self.gyroRange
        if gyro_x >= self.gyroRange:
            gyro_x -= 2 * self.gyroRange
        if gyro_y >= self.gyroRange:
            gyro_y -= 2 * self.gyroRange
        if gyro_z >= self.gyroRange:
            gyro_z -= 2 * self.gyroRange

        deviceModel.setDeviceData("gyroX", round(gyro_x, 4))
        deviceModel.setDeviceData("gyroY", round(gyro_y, 4))
        deviceModel.setDeviceData("gyroZ", round(gyro_z, 4))

    def get_angle(self,datahex, deviceModel):
        rxl = datahex[2]
        rxh = datahex[3]
        ryl = datahex[4]
        ryh = datahex[5]
        rzl = datahex[6]
        rzh = datahex[7]

        angle_x = (rxh << 8 | rxl) / 32768.0 * self.angleRange
        angle_y = (ryh << 8 | ryl) / 32768.0 * self.angleRange
        angle_z = (rzh << 8 | rzl) / 32768.0 * self.angleRange
        if angle_x >= self.angleRange:
            angle_x -= 2 * self.angleRange
        if angle_y >= self.angleRange:
            angle_y -= 2 * self.angleRange
        if angle_z >= self.angleRange:
            angle_z -= 2 * self.angleRange

        deviceModel.setDeviceData("angleX", round(angle_x, 3)) 
        deviceModel.setDeviceData("angleY", round(angle_y, 3))
        deviceModel.setDeviceData("angleZ", round((angle_z + 360) % 360, 3))
        
    def get_mag(self,datahex, deviceModel):
        _x = deviceModel.get_int(bytes([datahex[2],datahex[3]]))
        _y = deviceModel.get_int(bytes([datahex[4],datahex[5]]))
        _z = deviceModel.get_int(bytes([datahex[6],datahex[7]]))
        deviceModel.setDeviceData("magX", round(_x, 0))
        deviceModel.setDeviceData("magY", round(_y, 0))
        deviceModel.setDeviceData("magZ", round(_z, 0))
        
    def calculate_heading(self, deviceModel):
        try:
            # Get magnetometer and accelerometer data
            mag_x = deviceModel.getDeviceData("magX")
            mag_y = deviceModel.getDeviceData("magY")
            acc_x = deviceModel.getDeviceData("accX")
            acc_y = deviceModel.getDeviceData("accY")
            acc_z = deviceModel.getDeviceData("accZ")
            # Calculate tilt-compensated heading
            tilt_heading = math.atan2(mag_y, mag_x)
            pitch = math.atan2(acc_x, math.sqrt(acc_y**2 + acc_z**2))
            roll = math.atan2(-acc_y, acc_z)
            # Compensate for tilt
            heading = tilt_heading + roll  # Adjust for roll
            # Optionally, you can also adjust for pitch if needed:
            # heading -= pitch
            # Convert heading to degrees
            heading_degrees = round(math.degrees(heading), 2)
            heading = (heading_degrees + 360) % 360
            # Update device model with heading data
            deviceModel.setDeviceData("heading", heading)
            return heading
        except Exception as e:
            print(e)

    def get_lonlat(self,datahex, deviceModel):

        lon = deviceModel.get_unint(bytes([datahex[2],datahex[3],datahex[4],datahex[5]]))
        lat = deviceModel.get_unint(bytes([datahex[6],datahex[7],datahex[8],datahex[9]]))
        #(lon / 10000000 + ((double)(lon % 10000000) / 1e5 / 60.0)).ToString("f8")
        tlon = lon / 10000000.0
        tlat = lat / 10000000.0
        deviceModel.setDeviceData("lon", round(tlon, 8))  
        deviceModel.setDeviceData("lat", round(tlat, 8))   

    def get_gps(self,datahex, deviceModel):
        Height = deviceModel.get_int(bytes([datahex[2],datahex[3]])) / 10.0   
        Yaw = deviceModel.get_int(bytes([datahex[4],datahex[5]])) / 100.0   
        Speed = deviceModel.get_unint(bytes([datahex[6],datahex[7],datahex[8],datahex[9]])) / 1e3       

        deviceModel.setDeviceData("Height", round(Height, 3))   
        deviceModel.setDeviceData("Yaw", round(Yaw, 2))  
        deviceModel.setDeviceData("Speed", round(Speed, 3))   

    def get_four_elements(self,datahex, deviceModel):
        q1 = deviceModel.get_int(bytes([datahex[2],datahex[3]])) / 32768.0
        q2 = deviceModel.get_int(bytes([datahex[4],datahex[5]])) / 32768.0
        q3 = deviceModel.get_int(bytes([datahex[6],datahex[7]])) / 32768.0
        q4 = deviceModel.get_int(bytes([datahex[8],datahex[9]])) / 32768.0

        deviceModel.setDeviceData("q1", round(q1, 5))   
        deviceModel.setDeviceData("q2", round(q2, 5))  
        deviceModel.setDeviceData("q3", round(q3, 5)) 
        deviceModel.setDeviceData("q4", round(q4, 5))

    def get_chiptime(self,datahex, deviceModel):
        tempVals = []    
        for i in range(0,4):
            tIndex = 2 + i * 2
            tempVals.append(datahex[tIndex+1] << 8 | datahex[tIndex])

        _year = 2000 + (tempVals[0] & 0xff)      
        _moth = ((tempVals[0] >> 8) & 0xff)      
        _day = (tempVals[1] & 0xff)              
        _hour = ((tempVals[1] >> 8) & 0xff)      
        _minute = (tempVals[2] & 0xff)           
        _second = ((tempVals[2] >> 8) & 0xff)    
        _millisecond = tempVals[3]              
        deviceModel.setDeviceData("Chiptime",
                                  str(_year) + "-" + str(_moth) + "-" + str(_day) + " " + str(_hour) + ":" + str(
                                      _minute) + ":" + str(_second) + "." + str(_millisecond))  # 设备模型芯片时间赋值

    def readReg(self, regAddr,regCount, deviceModel):
        tempResults = []                      
        readCount = int(regCount/4)           
        if (regCount % 4>0):
            readCount+=1
        for n in range(0,readCount):
            self.TempFindValues = []  
            tempBytes = self.get_readbytes(regAddr + n * 4)
            print("Write Command: ", [hex(byte) for byte in tempBytes])            
            success_bytes = deviceModel.serialPort.write(tempBytes)  
            print("success_bytes: ", success_bytes)
            for i in range(0,20): 
                time.sleep(0.05)  
                if (len(self.TempFindValues)>0):   
                    for j in range(0,len(self.TempFindValues)):
                        if (len(tempResults) < regCount):
                            tempResults.append(self.TempFindValues[j])
                        else:
                            break
                    break
        return tempResults

    def writeReg(self, regAddr,sValue, deviceModel):
        tempBytes = self.get_writebytes(regAddr,sValue)                  
        success_bytes = deviceModel.serialPort.write(tempBytes) 
                 
    def unlock(self, deviceModel):
        tempBytes = self.get_writebytes(0x69, 0xb588)                    
        success_bytes = deviceModel.serialPort.write(tempBytes)          

    def save(self, deviceModel):
        tempBytes = self.get_writebytes(0x00, 0x00)                      
        success_bytes = deviceModel.serialPort.write(tempBytes)          

    def AccelerationCalibration(self,deviceModel):
        self.unlock(deviceModel)                                         
        time.sleep(0.1)                                                  
        tempBytes = self.get_writebytes(0x01, 0x01)                      
        success_bytes = deviceModel.serialPort.write(tempBytes)          
        time.sleep(5.5)                                                  

    def BeginFiledCalibration(self,deviceModel):
        self.unlock(deviceModel)                                         
        time.sleep(0.1)                                                  
        tempBytes = self.get_writebytes(0x01, 0x07)                      
        success_bytes = deviceModel.serialPort.write(tempBytes)          

    def EndFiledCalibration(self,deviceModel):
        self.unlock(deviceModel)                                         
        time.sleep(0.1)                                                  
        self.save(deviceModel)                                          

    def get_find(self,datahex, deviceModel):
        t0l = datahex[2]
        t0h = datahex[3]
        t1l = datahex[4]
        t1h = datahex[5]
        t2l = datahex[6]
        t2h = datahex[7]
        t3l = datahex[8]
        t3h = datahex[9]

        val0 = (t0h << 8 | t0l)
        val1 = (t1h << 8 | t1l)
        val2 = (t2h << 8 | t2l)
        val3 = (t3h << 8 | t3l)
        self.TempFindValues.extend([val0,val1,val2,val3])

class JY901SDataProcessor():
    onVarChanged = []

    def onOpen(self, deviceModel):
        pass

    def onClose(self):
        pass

    @staticmethod
    def onUpdate(*args):
        for fun in JY901SDataProcessor.onVarChanged:
            fun(*args)

class SerialConfig:
    # Port
    portName = '/dev/ttyUSB_witmotion'

    # Baud rate
    baud = 9600

class Witmotion:
    # Device Name
    deviceName = "My Device"

    # Device ID
    ADDR = 0x50

    # Device Data Dictionary
    deviceData = {}

    # Is Open
    isOpen = False

    # Serial Port
    serialPort = None

    # Serial Configuration
    serialConfig = SerialConfig()

    # Update Trigger
    dataUpdateListener = "51_0"

    # Data Processor
    dataProcessor = None

    # Protocol Resolver
    protocolResolver = None

    def __init__(self, deviceName, protocolResolver, dataProcessor, dataUpdateListener):
        print("Initializing device model")
        self.deviceName = deviceName
        self.protocolResolver = protocolResolver
        self.dataProcessor = dataProcessor
        self.dataUpdateListener = dataUpdateListener
        # _thread.start_new_thread(self.readDataTh, ("Data-Received-Thread", 10, ))

    def setDeviceData(self, key, value):
        """
        Set device data
        :param key: Data key
        :param value: Data value
        :return: None
        """
        self.deviceData[key] = value

    def getDeviceData(self, key):
        """
        Get device data
        :param key: Data key
        :return: Data value if exists, otherwise None
        """
        if key in self.deviceData:
            return self.deviceData[key]
        else:
            return None

    def removeDeviceData(self, key):
        """
        Remove device data
        :param key: Data key
        :return: None
        """
        del self.deviceData[key]

    def readDataTh(self, threadName, delay):
        """
        Read data thread
        :return: None
        """
        print("Starting " + threadName)
        while True:
            # If the serial port is open
            if self.isOpen:
                try:
                    tlen = self.serialPort.inWaiting()
                    if tlen > 0:
                        data = self.serialPort.read(tlen)
                        self.onDataReceived(data)
                except Exception as ex:
                    print(ex)
            else:
                time.sleep(0.1)
                print("Paused")
                break

    def openDevice(self):
        """
        Open the device
        :return: None
        """

        # Close the port first
        self.closeDevice()
        try:
            self.serialPort = serial.Serial(self.serialConfig.portName, self.serialConfig.baud, timeout=0.5)
            self.isOpen = True
            t = threading.Thread(target=self.readDataTh, args=("Data-Received-Thread", 10,))  # Start a thread to receive data
            t.start()
        except serial.SerialException:
            print("Failed to open " + self.serialConfig.portName + " at " + str(self.serialConfig.baud) + " baud")

    def closeDevice(self):
        """
        Close the device
        :return: None
        """
        if self.serialPort is not None:
            self.serialPort.close()
            print("Port closed")
        self.isOpen = False
        print("Device closed")

    def onDataReceived(self, data):
        """
        When data is received
        :param data: Received data
        :return: None
        """
        if self.protocolResolver is not None:
            self.protocolResolver.passiveReceiveData(data, self)

    def get_int(self, dataBytes):
        """
        Convert bytes to signed integer = C# BitConverter.ToInt16
        :param dataBytes: Byte array
        :return: Converted integer
        """
        return int.from_bytes(dataBytes, "little", signed=True)

    def get_unint(self, dataBytes):
        """
        Convert bytes to unsigned integer
        :param dataBytes: Byte array
        :return: Converted integer
        """
        return int.from_bytes(dataBytes, "little")

    def sendData(self, data):
        """
        Send data
        :return: Success flag
        """
        if self.protocolResolver is not None:
            self.protocolResolver.sendData(data, self)

    def readReg(self, regAddr, regCount):
        """
        Read register
        :param regAddr: Register address
        :param regCount: Number of registers
        :return: Read data
        """
        if self.protocolResolver is not None:
            return self.protocolResolver.readReg(regAddr, regCount, self)
        else:
            return None

    def writeReg(self, regAddr, sValue):
        """
        Write to register
        :param regAddr: Register address
        :param sValue: Value to write
        :return: None
        """
        if self.protocolResolver is not None:
            self.protocolResolver.writeReg(regAddr, sValue, self)

    def unlock(self):
        """
        Unlock
        :return: None
        """
        if self.protocolResolver is not None:
            self.protocolResolver.unlock(self)

    def save(self):
        """
        Save
        :return: None
        """
        if self.protocolResolver is not None:
            self.protocolResolver.save(self)

    def AccelerationCalibration(self):
        """
        Acceleration calibration
        :return: None
        """
        if self.protocolResolver is not None:
            self.protocolResolver.AccelerationCalibration(self)

    def BeginFiledCalibration(self):
        """
        Start field calibration
        :return: None
        """
        if self.protocolResolver is not None:
            self.protocolResolver.BeginFiledCalibration(self)

    def EndFiledCalibration(self):
        """
        End field calibration
        :return: None
        """
        if self.protocolResolver is not None:
            self.protocolResolver.EndFiledCalibration(self)

    def sendProtocolData(self, data):
        """
        Send data with protocol
        :return: None
        """
        if self.protocolResolver is not None:
            self.protocolResolver.sendData(data)

def readConfig(device, regAddr, regCount):
    """
    Example of reading configuration information
    :param device: Device model
    :return:
    """
    tVals = device.readReg(regAddr, regCount)
    time.sleep(0.1)
    if (len(tVals) > 0):
        print("Resultx: ", [bin(int(hex(byte), 16)) for byte in tVals])
        print("Resultx: ", [hex(byte) for byte in tVals])
        print("Resulty: ", tVals)
    else:
        print("No response")
    # tVals = device.readReg(0x23, 2)
    # if (len(tVals) > 0):
    #     print("Resultz: " + str(tVals))
    # else:
    #     print("No response")

def setConfig(device):
    """
    Example setting configuration information
    :param device: Device model
    :return:
    """
    device.unlock()
    time.sleep(0.1)
    device.writeReg(0x03, 0x0B)
    time.sleep(0.1)
    device.writeReg(0x23, 0)
    time.sleep(0.1)
    device.writeReg(0x24, 0)
    time.sleep(0.1)
    device.save()

def AccelerationCalibration(device):
    """
    Acceleration calibration
    :param device: Device model
    :return:
    """
    device.AccelerationCalibration()
    print("Acceleration calibration finished")

def FiledCalibration(device):
    """
    Magnetic field calibration
    :param device: Device model
    :return:
    """
    device.BeginFiledCalibration()
    if input("Please rotate slowly around the XYZ axis. End calibration after three full rotations (Y/N)?").lower() == "y":
        device.EndFiledCalibration()
        print("Magnetic field calibration finished")

def onUpdate(deviceModel):
    """
    Data update event
    :param deviceModel: Device model
    :return:
    """
    try:
        # Clear the terminal screen
        # os.system('cls' if os.name == 'nt' else 'clear')
        print("Temperature: {:.2f}".format(deviceModel.getDeviceData("temperature")))
        print("Acceleration: {:.4f}, {:.4f}, {:.4f}".format(deviceModel.getDeviceData("accX"),deviceModel.getDeviceData("accY"),deviceModel.getDeviceData("accZ")))
        print("Angular velocity: {:.4f}, {:.4f}, {:.4f}".format(deviceModel.getDeviceData("gyroX"),deviceModel.getDeviceData("gyroY"),deviceModel.getDeviceData("gyroZ")))
        print("Angle: {:.4f}, {:.4f}, {:.4f}".format(deviceModel.getDeviceData("angleX"),deviceModel.getDeviceData("angleY"),deviceModel.getDeviceData("angleZ")))
        print("Magnetic field: {:.0f}, {:.0f}, {:.0f}".format(deviceModel.getDeviceData("magX"),deviceModel.getDeviceData("magY"),deviceModel.getDeviceData("magZ")))
        print("Heading: {:.2f}".format(deviceModel.getDeviceData("heading")))
    except Exception as e:
        print(e)


if __name__ == '__main__':
    compass = Witmotion(
        "MSRS",
        WitProtocolResolver(),
        JY901SDataProcessor(),
        "51_0"
    )

    compass.serialConfig.portName = "/dev/ttyUSB_witmotion"           
    compass.serialConfig.baud = 9600                     
    compass.openDevice()
    setConfig(compass)  
    print("#" * 50)                                 
    readConfig(compass, 0x01, 1)
    print("#" * 50)                                 
    readConfig(compass, 0x02, 3)
    print("#" * 50)                                 
    readConfig(compass, 0x03, 1)
    print("#" * 50)                                 
    readConfig(compass, 0x04, 1)
    print("#" * 50)                                 
    readConfig(compass, 0x1F, 1)
    print("#" * 50)                                 
    readConfig(compass, 0x5F, 1)
    print("#" * 50)                                 
    readConfig(compass, 0x23, 2)
    print("#" * 50)                                 
    print(compass.deviceData)
    FiledCalibration(compass)                            
    compass.dataProcessor.onVarChanged.append(onUpdate)                             
    input()
    compass.closeDevice()
                                     
