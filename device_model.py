# coding: UTF-8
import threading
import _thread
import time
import struct
import serial
from serial import SerialException

'''
    Serial Configuration
'''


class SerialConfig:
    # Port
    portName = '/dev/ttyUSB_witmotion'

    # Baud rate
    baud = 9600

'''
Device Model
'''


class DeviceModel:
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
    dataUpdateListener = ""

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
        except SerialException:
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
