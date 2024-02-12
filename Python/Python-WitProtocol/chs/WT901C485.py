# coding: UTF-8
"""
    Test file
"""
import time
import datetime
import platform
import threading
import lib.device_model as deviceModel
from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
from lib.protocol_resolver.roles.protocol_485_resolver import Protocol485Resolver

welcome = """
Welcome to the Wit-Motion sample program
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
    Tempstr += "\tax(g)\tay(g)\taz(g)"
    Tempstr += "\twx(deg/s)\twy(deg/s)\twz(deg/s)"
    Tempstr += "\tAngleX(deg)\tAngleY(deg)\tAngleZ(deg)"
    Tempstr += "\tT(°)"
    Tempstr += "\tmagx\tmagy\tmagz"
    Tempstr += "\r\n"
    _writeF.write(Tempstr)
    print("Start recording data")

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
          )
    if (_IsWriteF):
        Tempstr = " " + str(deviceModel.getDeviceData("Chiptime"))
        Tempstr += "\t"+str(deviceModel.getDeviceData("accX")) + "\t"+str(deviceModel.getDeviceData("accY"))+"\t"+ str(deviceModel.getDeviceData("accZ"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("gyroX")) +"\t"+ str(deviceModel.getDeviceData("gyroY")) +"\t"+ str(deviceModel.getDeviceData("gyroZ"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("angleX")) +"\t" + str(deviceModel.getDeviceData("angleY")) +"\t"+ str(deviceModel.getDeviceData("angleZ"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("temperature"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("magX")) +"\t" + str(deviceModel.getDeviceData("magY")) +"\t"+ str(deviceModel.getDeviceData("magZ"))
        Tempstr += "\r\n"
        _writeF.write(Tempstr)

def LoopReadThead(device):
    """
    Cyclic read data
    :param device:
    :return:
    """
    while(True):
        device.readReg(0x30, 41)

if __name__ == '__main__':

    print(welcome)
    device = deviceModel.DeviceModel(
        "My JY901",
        Protocol485Resolver(),
        JY901SDataProcessor(),
        "51_0"
    )
    device.ADDR = 0x50
    if (platform.system().lower() == 'linux'):
        device.serialConfig.portName = "/dev/ttyUSB0"
    else:
        device.serialConfig.portName = "COM82"
    device.serialConfig.baud = 9600
    device.openDevice()
    readConfig(device)
    device.dataProcessor.onVarChanged.append(onUpdate)

    startRecord()
    t = threading.Thread(target=LoopReadThead, args=(device,))
    t.start()

    input()
    device.closeDevice()
    endRecord()
