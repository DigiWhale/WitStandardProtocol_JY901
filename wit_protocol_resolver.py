# coding: UTF-8
import time
from i_protocol_resolver import IProtocolResolver

"""
    Wit Protocol Resolver
"""


class WitProtocolResolver(IProtocolResolver):
    TempBytes = []         # Temporary data list
    PackSize = 11        # Size of one packet of data
    gyroRange = 2000.0   # Gyroscope range
    accRange = 16.0      # Acceleration range
    angleRange = 180.0   # Angle range
    TempFindValues = []    # Data returned when reading specific registers

    def setConfig(self, deviceModel):
        pass

    def sendData(self, sendData, deviceModel):
        success_bytes = deviceModel.serialPort.write(sendData)

    def passiveReceiveData(self, data, deviceModel):
        """
        Process received data
        :param data: Serial data
        :param deviceModel: Device model
        :return:
        """
        global TempBytes
        for val in data:
            self.TempBytes.append(val)
            if self.TempBytes[0] != 0x55:  # Not starting with identifier 0x55
                del self.TempBytes[0]  # Remove the first byte
                continue
            if len(self.TempBytes) > 1:
                if not (0x50 <= self.TempBytes[1] - 0x50 <= 11) or self.TempBytes[1] == 0x5f:  # Second byte value not in the range 0x50~0x5a or not equal to 0x5f
                    del self.TempBytes[0]  # Remove the first byte
                    continue
            if len(self.TempBytes) == self.PackSize:  # Indicates the size of one packet of data
                CheckSum = 0  # Sum check
                for i in range(0, self.PackSize - 1):
                    CheckSum += self.TempBytes[i]
                if CheckSum & 0xff == self.TempBytes[self.PackSize - 1]:  # Check passed
                    if self.TempBytes[1] == 0x50:  # Chip time packet
                        self.get_chiptime(self.TempBytes, deviceModel)  # Calculate chip time data
                    elif self.TempBytes[1] == 0x51:  # Acceleration packet
                        self.get_acc(self.TempBytes, deviceModel)  # Calculate acceleration data
                    elif self.TempBytes[1] == 0x52:  # Gyroscope packet
                        self.get_gyro(self.TempBytes, deviceModel)  # Calculate gyroscope data
                    elif self.TempBytes[1] == 0x53:  # Angle packet
                        self.get_angle(self.TempBytes, deviceModel)  # Calculate angle data
                    elif self.TempBytes[1] == 0x54:  # Magnetic field packet
                        self.get_mag(self.TempBytes, deviceModel)  # Calculate magnetic field data
                        deviceModel.dataProcessor.onUpdate(deviceModel)  # Trigger data update event
                    elif self.TempBytes[1] == 0x57:  # Longitude and latitude packet
                        self.get_lonlat(self.TempBytes, deviceModel)  # Calculate longitude and latitude data
                        deviceModel.dataProcessor.onUpdate(deviceModel)  # Trigger data update event
                    elif self.TempBytes[1] == 0x58:  # GPS packet
                        self.get_gps(self.TempBytes, deviceModel)  # Calculate GPS data
                        deviceModel.dataProcessor.onUpdate(deviceModel)  # Trigger data update event
                    elif self.TempBytes[1] == 0x59:  # Quaternion packet
                        self.get_four_elements(self.TempBytes, deviceModel)  # Calculate quaternion data
                        deviceModel.dataProcessor.onUpdate(deviceModel)  # Trigger data update event
                    elif self.TempBytes[1] == 0x5f:  # Return reading of specified register
                        self.get_find(self.TempBytes, deviceModel)
                    self.TempBytes = []  # Clear data
                else:  # Check failed
                    del self.TempBytes[0]  # Remove the first byte

    def get_readbytes(self, regAddr):
        """
        Get the read instruction
        :param regAddr: Register address
        :return:
        """
        return [0xff, 0xaa, 0x27, regAddr & 0xff, regAddr >> 8]

    def get_writebytes(self, regAddr, sValue):
        """
        Get the write instruction
        :param regAddr: Register address
        :param sValue: Value to write
        :return:
        """
        return [0xff, 0xaa, regAddr, sValue & 0xff, sValue >> 8]

    def get_acc(self, datahex, deviceModel):
        """
        Acceleration and temperature calculation
        :param datahex: Raw data packet
        :param deviceModel: Device model
        :return:
        """
        axl = datahex[2]
        axh = datahex[3]
        ayl = datahex[4]
        ayh = datahex[5]
        azl = datahex[6]
        azh = datahex[7]

        tempVal = (datahex[9] << 8 | datahex[8])
        acc_x = (axh << 8 | axl) / 32768.0 * self.accRange
        acc_y = (ayh << 8 | ayl) / 32768.0 * self.accRange
        acc_z = (azh << 8 | azl) / 32768.0 * self.accRange
        if acc_x >= self.accRange:
            acc_x -= 2 * self.accRange
        if acc_y >= self.accRange:
            acc_y -= 2 * self.accRange
        if acc_z >= self.accRange:
            acc_z -= 2 * self.accRange

        deviceModel.setDeviceData("accX", round(acc_x, 4))  # Assign acceleration X to device model
        deviceModel.setDeviceData("accY", round(acc_y, 4))  # Assign acceleration Y to device model
        deviceModel.setDeviceData("accZ", round(acc_z, 4))  # Assign acceleration Z to device model
        temperature = round(tempVal / 100.0, 2)  # Calculate temperature and keep two decimal places
        deviceModel.setDeviceData("temperature", temperature)  # Assign temperature to device model

    def get_gyro(self, datahex, deviceModel):
        """
        Gyroscope calculation
        :param datahex: Raw data packet
        :param deviceModel: Device model
        :return:
        """
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

        deviceModel.setDeviceData("gyroX", round(gyro_x, 4))  # Assign gyroscope X to device model
        deviceModel.setDeviceData("gyroY", round(gyro_y, 4))  # Assign gyroscope Y to device model
        deviceModel.setDeviceData("gyroZ", round(gyro_z, 4))  # Assign gyroscope Z to device model

    def get_angle(self, datahex, deviceModel):
        """
        Angle calculation
        :param datahex: Raw data packet
        :param deviceModel: Device model
        :return:
        """
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

        deviceModel.setDeviceData("angleX", round(angle_x, 3))  # Assign angle X to device model
        deviceModel.setDeviceData("angleY", round(angle_y, 3))  # Assign angle Y to device model
        deviceModel.setDeviceData("angleZ", round(angle_z, 3))  # Assign angle Z to device model

    def get_mag(self, datahex, deviceModel):
        """
        Magnetic field calculation
        :param datahex: Raw data packet
        :param deviceModel: Device model
        :return:
        """
        _x = deviceModel.get_int(bytes([datahex[2], datahex[3]]))
        _y = deviceModel.get_int(bytes([datahex[4], datahex[5]]))
        _z = deviceModel.get_int(bytes([datahex[6], datahex[7]]))

        deviceModel.setDeviceData("magX", round(_x, 0))  # Assign magnetic field X to device model
        deviceModel.setDeviceData("magY", round(_y, 0))  # Assign magnetic field Y to device model
        deviceModel.setDeviceData("magZ", round(_z, 0))  # Assign magnetic field Z to device model

    def get_lonlat(self, datahex, deviceModel):
        """
        Longitude and latitude calculation
        :param datahex: Raw data packet
        :param deviceModel: Device model
        :return:
        """

        lon = deviceModel.get_unint(bytes([datahex[2], datahex[3], datahex[4], datahex[5]]))
        lat = deviceModel.get_unint(bytes([datahex[6], datahex[7], datahex[8], datahex[9]]))
        tlon = lon / 10000000.0
        tlat = lat / 10000000.0
        deviceModel.setDeviceData("lon", round(tlon, 8))  # Assign longitude to device model
        deviceModel.setDeviceData("lat", round(tlat, 8))  # Assign latitude to device model

    def get_gps(self, datahex, deviceModel):
        """
        GPS calculation
        :param datahex: Raw data packet
        :param deviceModel: Device model
        :return:
        """
        Height = deviceModel.get_int(bytes([datahex[2], datahex[3]])) / 10.0  # Height
        Yaw = deviceModel.get_int(bytes([datahex[4], datahex[5]])) / 100.0  # Yaw angle
        Speed = deviceModel.get_unint(
            bytes([datahex[6], datahex[7], datahex[8], datahex[9]])) / 1e3  # Nautical miles

        deviceModel.setDeviceData("Height", round(Height, 3))  # Assign height to device model
        deviceModel.setDeviceData("Yaw", round(Yaw, 2))  # Assign yaw angle to device model
        deviceModel.setDeviceData("Speed", round(Speed, 3))  # Assign speed to device model

    def get_four_elements(self, datahex, deviceModel):
        """
        Quaternion calculation
        :param datahex: Raw data packet
        :param deviceModel: Device model
        :return:
        """
        q1 = deviceModel.get_int(bytes([datahex[2], datahex[3]])) / 32768.0
        q2 = deviceModel.get_int(bytes([datahex[4], datahex[5]])) / 32768.0
        q3 = deviceModel.get_int(bytes([datahex[6], datahex[7]])) / 32768.0
        q4 = deviceModel.get_int(bytes([datahex[8], datahex[9]])) / 32768.0

        deviceModel.setDeviceData("q1", round(q1, 5))  # Assign element 1 to device model
        deviceModel.setDeviceData("q2", round(q2, 5))  # Assign element 2 to device model
        deviceModel.setDeviceData("q3", round(q3, 5))  # Assign element 3 to device model
        deviceModel.setDeviceData("q4", round(q4, 5))  # Assign element 4 to device model

    def get_chiptime(self, datahex, deviceModel):
        """
        Chip time calculation
        :param datahex: Raw data packet
        :param deviceModel: Device model
        :return:
        """
        tempVals = []  # Temporary calculation data
        for i in range(0, 4):
            tIndex = 2 + i * 2
            tempVals.append(datahex[tIndex + 1] << 8 | datahex[tIndex])

        _year = 2000 + (tempVals[0] & 0xff)  # Year
        _moth = ((tempVals[0] >> 8) & 0xff)  # Month
        _day = (tempVals[1] & 0xff)  # Day
        _hour = ((tempVals[1] >> 8) & 0xff)  # Hour
        _minute = (tempVals[2] & 0xff)  # Minute
        _second = ((tempVals[2] >> 8) & 0xff)  # Second
        _millisecond = tempVals[3]  # Millisecond
        deviceModel.setDeviceData("Chiptime",
                                  str(_year) + "-" + str(_moth) + "-" + str(_day) + " " + str(_hour) + ":" + str(
                                      _minute) + ":" + str(_second) + "." + str(_millisecond))  # Assign chip time to device model

    def readReg(self, regAddr, regCount, deviceModel):
        """
        Read register
        :param regAddr: Register address
        :param regCount: Number of registers
        :param deviceModel: Device model
        :return:
        """
        print("Read register: " + str(regAddr) + " " + str(regCount))
        tempResults = []  # Return data
        readCount = int(regCount / 4)  # Get the number of reads based on the number of registers
        if (regCount % 4 > 0):
            readCount += 1
        for n in range(0, readCount):
            self.TempFindValues = []  # Clear data
            tempBytes = self.get_readbytes(regAddr + n * 4)  # Get read instruction
            print("Read register: " + str(tempBytes))
            success_bytes = deviceModel.serialPort.write(tempBytes)  # Write data
            for i in range(0, 20):  # Set timeout 1 second
                time.sleep(0.05)  # Sleep 50 milliseconds
                if (len(self.TempFindValues) > 0):  # The value of the specified register has been returned
                    for j in range(0, len(self.TempFindValues)):
                        if (len(tempResults) < regCount):
                            tempResults.append(self.TempFindValues[j])
                        else:
                            break
                    break
        return tempResults

    def writeReg(self, regAddr, sValue, deviceModel):
        """
        Write register
        :param regAddr: Register address
        :param sValue: Value to be written
        :param deviceModel: Device model
        :return:
        """
        tempBytes = self.get_writebytes(regAddr, sValue)  # Get write instruction
        success_bytes = deviceModel.serialPort.write(tempBytes)  # Write register

    def unlock(self, deviceModel):
        """
        Unlock
        :return:
        """
        tempBytes = self.get_writebytes(0x69, 0xb588)  # Get write instruction
        success_bytes = deviceModel.serialPort.write(tempBytes)  # Write register

    def save(self, deviceModel):
        """
        Save
        :param deviceModel: Device model
        :return:
        """
        tempBytes = self.get_writebytes(0x00, 0x00)  # Get write instruction
        success_bytes = deviceModel.serialPort.write(tempBytes)  # Write register

    def AccelerationCalibration(self, deviceModel):
        """
        Acceleration calibration
        :param deviceModel: Device model
        :return:
        """
        self.unlock(deviceModel)  # Unlock
        time.sleep(0.1)  # Sleep 100 milliseconds
        tempBytes = self.get_writebytes(0x01, 0x01)  # Get write instruction
        success_bytes = deviceModel.serialPort.write(tempBytes)  # Write register
        time.sleep(5.5)  # Sleep 5500 milliseconds

    def BeginFiledCalibration(self, deviceModel):
        """
        Start field calibration
        :param deviceModel: Device model
        :return:
        """
        self.unlock(deviceModel)  # Unlock
        time.sleep(0.1)  # Sleep 100 milliseconds
        tempBytes = self.get_writebytes(0x01, 0x07)  # Get write instruction for magnetic field calibration
        success_bytes = deviceModel.serialPort.write(tempBytes)  # Write register

    def EndFiledCalibration(self, deviceModel):
        """
        End field calibration
        :param deviceModel: Device model
        :return:
        """
        self.unlock(deviceModel)  # Unlock
        time.sleep(0.1)  # Sleep 100 milliseconds
        self.save(deviceModel)  # Save

    def get_find(self, datahex, deviceModel):
        """
        Read specified register calculation
        :param datahex: Raw data packet
        :param deviceModel: Device model
        :return:
        """
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
        self.TempFindValues.extend([val0, val1, val2, val3])
