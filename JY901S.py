# coding: UTF-8
"""
Test file
"""
import time
import datetime
import platform
import struct
import device_model as deviceModel
from jy901s_dataProcessor import JY901SDataProcessor
from wit_protocol_resolver import WitProtocolResolver

welcome = """
Welcome to the Wit-Motoin sample program
"""
_writeF = None
_IsWriteF = False

def readConfig(device):
    """
    Example of reading configuration information
    :param device: Device model
    :return:
    """
    tVals = device.readReg(0x02, 3)
    if (len(tVals) > 0):
        print("Result: " + str(tVals))
    else:
        print("No response")
    tVals = device.readReg(0x23, 2)
    if (len(tVals) > 0):
        print("Result: " + str(tVals))
    else:
        print("No response")

def setConfig(device):
    """
    Example setting configuration information
    :param device: Device model
    :return:
    """
    device.unlock()
    time.sleep(0.1)
    device.writeReg(0x03, 6)
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
    print("Chip time:" + str(deviceModel.getDeviceData("Chiptime"))
         , " Temperature:" + str(deviceModel.getDeviceData("temperature"))
         , " Acceleration:" + str(deviceModel.getDeviceData("accX")) +","+  str(deviceModel.getDeviceData("accY")) +","+ str(deviceModel.getDeviceData("accZ"))
         , " Angular velocity:" + str(deviceModel.getDeviceData("gyroX")) +","+ str(deviceModel.getDeviceData("gyroY")) +","+ str(deviceModel.getDeviceData("gyroZ"))
         , " Angle:" + str(deviceModel.getDeviceData("angleX")) +","+ str(deviceModel.getDeviceData("angleY")) +","+ str(deviceModel.getDeviceData("angleZ"))
        , " Magnetic field:" + str(deviceModel.getDeviceData("magX")) +","+ str(deviceModel.getDeviceData("magY"))+","+ str(deviceModel.getDeviceData("magZ"))
        , " Longitude:" + str(deviceModel.getDeviceData("lon")) + " Latitude:" + str(deviceModel.getDeviceData("lat"))
        , " Heading angle:" + str(deviceModel.getDeviceData("Yaw")) + " Ground speed:" + str(deviceModel.getDeviceData("Speed"))
         , " Quaternion:" + str(deviceModel.getDeviceData("q1")) + "," + str(deviceModel.getDeviceData("q2")) + "," + str(deviceModel.getDeviceData("q3"))+ "," + str(deviceModel.getDeviceData("q4"))
          )
    if (_IsWriteF):    
        Tempstr = " " + str(deviceModel.getDeviceData("Chiptime"))
        Tempstr += "\t"+str(deviceModel.getDeviceData("accX")) + "\t"+str(deviceModel.getDeviceData("accY"))+"\t"+ str(deviceModel.getDeviceData("accZ"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("gyroX")) +"\t"+ str(deviceModel.getDeviceData("gyroY")) +"\t"+ str(deviceModel.getDeviceData("gyroZ"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("angleX")) +"\t" + str(deviceModel.getDeviceData("angleY")) +"\t"+ str(deviceModel.getDeviceData("angleZ"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("temperature"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("magX")) +"\t" + str(deviceModel.getDeviceData("magY")) +"\t"+ str(deviceModel.getDeviceData("magZ"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("lon")) + "\t" + str(deviceModel.getDeviceData("lat"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("Yaw")) + "\t" + str(deviceModel.getDeviceData("Speed"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("q1")) + "\t" + str(deviceModel.getDeviceData("q2"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("q3")) + "\t" + str(deviceModel.getDeviceData("q4"))
        Tempstr += "\r\n"
        _writeF.write(Tempstr)

def startRecord():
    """
    Start recording data
    :return:
    """
    global _writeF
    global _IsWriteF
    _writeF = open(str(datetime.datetime.now().strftime('%Y%m%d%H%M%S')) + ".txt", "w")
    _IsWriteF = True
    Tempstr = "Chiptime"
    Tempstr +=  "\tax(g)\tay(g)\taz(g)"
    Tempstr += "\twx(deg/s)\twy(deg/s)\twz(deg/s)"
    Tempstr += "\tAngleX(deg)\tAngleY(deg)\tAngleZ(deg)"
    Tempstr += "\tT(°)"
    Tempstr += "\tmagx\tmagy\tmagz"
    Tempstr += "\tlon\tlat"
    Tempstr += "\tYaw\tSpeed"
    Tempstr += "\tq1\tq2\tq3\tq4"
    Tempstr += "\r\n"
    _writeF.write(Tempstr)
    print("Start recording data", Tempstr)

def endRecord():
    """
    End record data
    :return:
    """
    global _writeF
    global _IsWriteF
    _IsWriteF = False             
    _writeF.close()               
    print("End record data")

if __name__ == '__main__':

    print(welcome)
    device = deviceModel.DeviceModel(
        "My JY901",
        WitProtocolResolver(),
        JY901SDataProcessor(),
        "51_0"
    )


    device.serialConfig.portName = "/dev/ttyUSB_witmotion"           
    device.serialConfig.baud = 9600                     
    device.openDevice()                                 
    readConfig(device)                                  
    device.dataProcessor.onVarChanged.append(onUpdate)  

    startRecord()                                       
    input()
    device.closeDevice()
    endRecord()                                        
