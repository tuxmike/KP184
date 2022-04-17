#!/usr/bin/env python3

'''
Python Wrapper to control Kunkin Electronic Load KP184 via RS485 / Serial Interface

Connect Electronic Load via KP184 / serial USB adapter. Use serial port as first parameter of class constructor.

Class can be used as:
1. Class instance: kp = KP184(port, address) Then call some method, e.g. voltageRead = kp.readCV()
2. main(), included in this file, e.g.: ./kp184.py /dev/ttyUSB0 writeLoadOnOff 1
3. Deriving by coustom class, see kp184Example.py

Based on protocol description from https://www.eevblog.com/forum/testgear/serial-to-usb-on-kunkin-kp184-electronic-load/msg3147906/#msg3147906
Note that some registers from the documentation are partially not functional, see code comments - remarks appreciated.

@Author: M.Reithmeier (https://github.com/tuxmike/)

MIT License

Copyright (c) 2022 M.Reithmeier

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''

import sys
import serial
import time
from struct import pack, unpack

class KP184:
    
    def __init__(self, rs485port, address):
        self.openPort(rs485port)
        self.address = int(address)
        self.debug = False

    def openPort(self, rs485port):
        self.serial = serial.Serial()
        self.serial.port = rs485port
        self.serial.baudrate = 9600
        self.serial.stopbits = 1
        self.serial.bytesize = 8
        self.serial.parity = 'N'
        self.serial.timeout = 0.2
        try:
            self.serial.open()
        except serial.SerialException:
            print(f"Could not open port {rs485port}")
    
    def request(self, cmd):
        try:
            self.sendRequest(cmd)
            return self.receiveResponse(cmd)
        except serial.SerialException:
            print(f"RS485 Error: {sys.exc_info()}")

    def sendRequest(self, cmd):
        framedata = pack('!B', self.address) + cmd.packPayload()
        framedata += pack('!H', self.crc16(framedata))
        if self.debug:
            print(framedata.hex())
        self.serial.write(framedata)

    def receiveResponse(self, cmd):
        framedata = self.serial.read(cmd.getFrameRespFrameLen())
        if self.debug:
            print(framedata.hex())
        if len(framedata) < 4:
            raise serial.SerialException("Received incomplete data")
        crc = unpack('!H', framedata[-2:])[0]
        
        if self.crc16(framedata[:-2]) != crc:
            print(f"crcin:{crc:x} crcexp:{self.crc16(framedata[:-2]):x}")
            raise serial.SerialException("CRC error")
        return cmd.unpackPayload(framedata[1:-2])

    def crc16(self, data):
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                crc = ((crc >> 1) ^ 0xA001
                    if (crc & 0x0001)
                    else crc >> 1)
        return ((crc << 8) & 0xFF00) | ((crc >> 8) & 0x00FF)

    #####   Methods   #####
    def readVersion(self):
        cmd = Version_Read()
        return self.request(cmd)

    def readKeySoundOnOff(self):
        cmd = KeySoundOnOff_Read()
        return self.request(cmd)
    
    def writeKeySoundOnOff(self, onOff: bool):
        cmd = KeySoundOnOff_Write(onOff)
        return self.request(cmd)

    def readKeyLockOnOff(self):
        cmd = KeyLockOnOff_Read()
        return self.request(cmd)

    def writeKeyLockOnOff(self, onOff: bool):
        cmd = KeyLockOnOff_Write(onOff)
        return self.request(cmd)

    def readOverTempAlarm(self):
        cmd = OverTempAlarm_Read()
        return self.request(cmd)

    def readVsensePortOnOff(self):
        cmd = VsensePortOnOff_Read()
        return self.request(cmd)
    
    def writeVsensePortOnOff(self, onOff: bool):
        cmd = VsensePortOnOff_Write(onOff)
        return self.request(cmd)

    def readLoadOnOff(self):
        cmd = LoadOnOff_Read()
        return self.request(cmd)

    def writeLoadOnOff(self, onOff: bool):
        cmd = LoadOnOff_Write(onOff)
        return self.request(cmd)

    def readMode(self):
        cmd = Mode_Read()
        return self.request(cmd)

    def writeMode(self, mode: str): # "CV", "CC", "CR", "CW"
        cmd = Mode_Write(mode)
        return self.request(cmd)

    def readCV(self):
        cmd = CV_Read()
        return self.request(cmd)

    def writeCV(self, voltage: float):
        cmd = CV_Write(voltage)
        return self.request(cmd)

    def readCC(self):
        cmd = CC_Read()
        return self.request(cmd)

    def writeCC(self, current: float):
        cmd = CC_Write(current)
        return self.request(cmd)

    def readCR(self):
        cmd = CR_Read()
        return self.request(cmd)

    def writeCR(self, resistance: float):
        cmd = CR_Write(resistance)
        return self.request(cmd)

    def readCW(self):
        cmd = CW_Read()
        return self.request(cmd)

    def writeCW(self, power: float):
        cmd = CW_Write(power)
        return self.request(cmd)

    def readVoltage(self):
        cmd = Voltage_Read()
        return self.request(cmd)

    def readPower(self):
        cmd = Power_Read()
        return self.request(cmd)

    def readCurrent(self):
        cmd = Current_Read()
        return self.request(cmd)

    def readVoltageCurrent(self):
        cmd = VoltageCurrent_Read()
        return self.request(cmd)

    def readOnloadLvlV(self):
        cmd = OnloadLvlV_Read()
        return self.request(cmd)

    def writeOnloadLvlV(self, voltage: float):
        cmd = OnloadLvlV_Write(voltage)
        return self.request(cmd)

    def readSlewRateUp(self):
        cmd = SlewRateUp_Read()
        return self.request(cmd)

    def readSlewRateDown(self):
        cmd = SlewRateDown_Read()
        return self.request(cmd)

    def writeSlewRateUp(self, aMsUp: float):
        cmd = SlewRateUp_Write(aMsUp)
        return self.request(cmd)

    def writeSlewRateDown(self, aMsDown: float):
        cmd = SlewRateDown_Write(aMsDown)
        return self.request(cmd)

    def writePassFailOut(self, fail: bool):
        cmd = PassFailOut_Write(fail)
        return self.request(cmd)

    def readDynaTestOnOff(self):
        cmd = DynaTestOnOff_Read()
        return self.request(cmd)

    def writeDynaTestOnOff(self, onOff: bool):
        cmd = DynaTestOnOff_Write(onOff)
        return self.request(cmd)

    def readDynaACurrent(self):
        cmd = DynaACurrent_Read()
        return self.request(cmd)

    def writeDynaACurrent(self, current: float):
        cmd = DynaACurrent_Write(current)
        return self.request(cmd)

    def readDynaATime(self):
        cmd = DynaATime_Read()
        return self.request(cmd)

    def writeDynaATime(self, current: float):
        cmd = DynaATime_Write(current)
        return self.request(cmd)

    def readDynaBCurrent(self):
        cmd = DynaBCurrent_Read()
        return self.request(cmd)

    def writeDynaBCurrent(self, current: float):
        cmd = DynaBCurrent_Write(current)
        return self.request(cmd)

    def readDynaBTime(self):
        cmd = DynaBTime_Read()
        return self.request(cmd)

    def writeDynaBTime(self, current: float):
        cmd = DynaBTime_Write(current)
        return self.request(cmd)

    def writeDynaAB(self, currentA: float, currentB: float, timeA: float, timeB: float):
        cmd = DynaAB_Write(currentA, currentB, timeA, timeB)
        return self.request(cmd)

    def readBattTestOnOff(self):
        cmd = BattTestOnOff_Read()
        return self.request(cmd)

    def writeBattTestOnOff(self, onOff: bool):
        cmd = BattTestOnOff_Write(onOff)
        return self.request(cmd)

    def readBattTestEndVoltage(self):
        cmd = BattTestEndVoltage_Read()
        return self.request(cmd)

    def writeBattTestEndVoltage(self, voltage: float):
        cmd = BattTestEndVoltage_Write(voltage)
        return self.request(cmd)

    def readBattTestCapacityResult(self):
        cmd = BattTestCapacityResult_Read()
        return self.request(cmd)

    def readBattTestHalfCurrOnOff(self):
        cmd = BattTestHalfCurrOnOff_Read()
        return self.request(cmd)

    def writeBattTestHalfCurrOnOff(self, onOff: bool):
        cmd = BattTestHalfCurrOnOff_Write(onOff)
        return self.request(cmd)

    def readBattTestUnit(self):
        cmd = BattTestUnit_Read()
        return self.request(cmd)

    def writeBattTestUnit(self, unit: str): # "WH" or "AH"
        cmd = BattTestUnit_Write(unit)
        return self.request(cmd)

    def readBattTestSignal(self):
        cmd = BattTestSignal_Read()
        return self.request(cmd)

    def writeBattTestEndSignal(self, signal: int): # 0:buzz once, 1: cont buzz, 2: level out
        cmd = BattTestSignal_Write(signal)
        return self.request(cmd)

    def readBattTestAll(self):
        cmd = BattTestAll_Read()
        return self.request(cmd)

    def readCompareOnOff(self):
        cmd = CompareOnOff_Read()
        return self.request(cmd)

    def writeCompareOnOff(self, onOff: bool):
        cmd = CompareOnOff_Write(onOff)
        return self.request(cmd)

    def readCompareVoltageHigh(self):
        cmd = CompareVoltageHigh_Read()
        return self.request(cmd)

    def writeCompareVoltageHigh(self, voltage: float):
        cmd = CompareVoltageHigh_Write(voltage)
        return self.request(cmd)

    def readCompareVoltageLow(self):
        cmd = CompareVoltageLow_Read()
        return self.request(cmd)

    def writeCompareVoltageLow(self, voltage: float):
        cmd = CompareVoltageLow_Write(voltage)
        return self.request(cmd)
    
    def readCompareCurrentHigh(self):
        cmd = CompareCurrentHigh_Read()
        return self.request(cmd)

    def writeCompareCurrentHigh(self, current: float):
        cmd = CompareCurrentHigh_Write(current)
        return self.request(cmd)

    def readCompareCurrentLow(self):
        cmd = CompareCurrentLow_Read()
        return self.request(cmd)

    def writeCompareCurrentLow(self, current: float):
        cmd = CompareCurrentLow_Write(current)
        return self.request(cmd)

    def readCompareFailAlarm(self):
        cmd = CompareFailAlarm_Read()
        return self.request(cmd)

    def writeCompareFailAlarm(self, alarm: int): # 0: buzz, 1: LevelOut
        cmd = CompareFailAlarm_Write(alarm)
        return self.request(cmd)

class PK184Cmd:

    def __init__(self): # just for reference, not called
        self.funcCode = 0
        self.respHdrAndCRCLen = 0
        self.respDataLen = 0
        self.register = 0x0000

    def encode(self):
        return b""

    def decode(self, data):
        return ""

    def packPayload(self):
        payload = pack('!BH', self.funcCode, self.register)
        payload += self.encode()
        return payload
    
    def unpackPayload(self, payload):
        funcCode, register = unpack('!BH', payload[0:3])
        return self.decode(payload[2:])

    def getFrameRespFrameLen(self):
        return self.respHdrAndCRCLen + self.respDataLen

class PK184ReadCmd(PK184Cmd):
    def __init__(self):
        self.funcCode = 0x03
        self.respHdrAndCRCLen = 5 # devaddr, func, numofbytes, crcH, crcL
        self.respDataLen = 4 # default, 1 reg = 4 byte

    def packPayload(self):
        payload = pack('!BHH', self.funcCode, self.register, self.respDataLen)
        payload += self.encode()
        return payload

class PK184WriteCmd(PK184Cmd):
    def __init__(self):
        self.funcCode = 0x06
        self.respHdrAndCRCLen = 9 # devaddr, func, startregisterH, startregisterL, numofregistersH, numofregistersL, numofbytes, crcH, crcL
        self.respDataLen = 0 # default

    def packPayload(self):
        payload = pack('!BHHB', self.funcCode, self.register, 0x0001, 0x04) # default: 1 reg = 4 byte
        payload += self.encode()
        return payload

class PK184WriteMCmd(PK184Cmd):
    def __init__(self):
        self.funcCode = 0x10
        self.respHdrAndCRCLen = 9 # devaddr, func, startregisterH, startregisterL, numofregistersH, numofregistersL, numofbytes, crcH, crcL
        self.respDataLen = 0 # default

####    Commands    ####  
class Version_Read(PK184ReadCmd):
    def __init__(self):
        super().__init__()
        self.register = 0x0001

    def decode(self, data):
        self.model, self.sw, self.hw, self.mandata = unpack('!BBBB', data)
        return (self.model, self.sw, self.hw, self.mandata)

class KeySoundOnOff_Read(PK184ReadCmd):
    def __init__(self):
        super().__init__()
        self.register = 0x0100

    def decode(self, data):
        self.onOff = bool(unpack('!I', data)[0] & 0x01)
        return self.onOff

class KeySoundOnOff_Write(PK184WriteCmd):
    def __init__(self, onOff: bool):
        super().__init__()
        self.register = 0x0100
        self.onOff = onOff
    
    def encode(self):
        return pack('!I', int(self.onOff))

class KeyLockOnOff_Read(PK184ReadCmd):
    def __init__(self):
        super().__init__()
        self.register = 0x0102

    def decode(self, data):
        self.onOff = (unpack('!I', (data)[0] & 0x01) ^ 1) # negated
        return self.onOff

class KeyLockOnOff_Write(PK184WriteCmd):
    def __init__(self, onOff: bool):
        super().__init__()
        self.register = 0x0102
        self.onOff = onOff
    
    def encode(self):
        return pack('!I', int(self.onOff) ^ 1) # negated

class OverTempAlarm_Read(PK184ReadCmd):
    def __init__(self):
        super().__init__()
        self.register = 0x0108

    def decode(self, data):
        self.onOff = bool(unpack('!I', data)[0] & 0x01)
        return self.onOff

class VsensePortOnOff_Read(PK184ReadCmd):
    def __init__(self):
        super().__init__()
        self.register = 0x010A

    def decode(self, data):
        self.onOff = bool(unpack('!I', data)[0] & 0x01)
        return self.onOff

class VsensePortOnOff_Write(PK184WriteCmd):
    def __init__(self, onOff: bool):
        super().__init__()
        self.register = 0x010A
        self.onOff = onOff
    
    def encode(self):
        return pack('!I', int(self.onOff))

class LoadOnOff_Read(PK184ReadCmd):
    def __init__(self):
        super().__init__()
        self.register = 0x010E

    def decode(self, data):
        self.onOff = bool(unpack('!I', data)[0] & 0x01)
        return self.onOff

class LoadOnOff_Write(PK184WriteCmd):
    def __init__(self, onOff: bool):
        super().__init__()
        self.register = 0x010E
        self.onOff = onOff
    
    def encode(self):
        return pack('!I', int(self.onOff))

class Mode_Read(PK184ReadCmd):
    def __init__(self):
        super().__init__()
        self.register = 0x0110

    def decode(self, data):
        modeStr = ["CV", "CC", "CR", "CW"]
        self.mode = modeStr[ unpack('!I', data)[0] & 0x03 ]
        return self.mode

class Mode_Write(PK184WriteCmd):
    def __init__(self, mode: str):
        super().__init__()
        self.register = 0x0110
        modeStr = ["CV", "CC", "CR", "CW"]
        self.mode = modeStr.index(mode)
    
    def encode(self):
        return pack('!I', self.mode)

class CV_Read(PK184ReadCmd):
    def __init__(self):
        super().__init__()
        self.register = 0x0112

    def decode(self, data):
        self.cv = unpack('!I', data)[0] / 1000.0
        return self.cv

class CV_Write(PK184WriteCmd):
    def __init__(self, cv: float):
        super().__init__()
        self.register = 0x0112
        self.cv = cv
    
    def encode(self):
        cv = max(0, min(150000, round( float(self.cv)*1000.0 ) ))
        return pack('!I', cv)

class CC_Read(PK184ReadCmd):
    def __init__(self):
        super().__init__()
        self.register = 0x0116

    def decode(self, data):
        self.cc = unpack('!I', data)[0] / 1000.0
        return self.cc

class CC_Write(PK184WriteCmd):
    def __init__(self, cc: float):
        super().__init__()
        self.register = 0x0116
        self.cc = cc
    
    def encode(self):
        cc = max(0, min(30000, round( float(self.cc)*1000.0 ) ))
        return pack('!I', cc)

class CR_Read(PK184ReadCmd):
    def __init__(self):
        super().__init__()
        self.register = 0x011A

    def decode(self, data):
        self.cr = unpack('!I', data)[0] / 10.0
        return self.cr

class CR_Write(PK184WriteCmd):
    def __init__(self, cr: float):
        super().__init__()
        self.register = 0x011A
        self.cr = cr
    
    def encode(self):
        cr = max(0, min(99998, round( float(self.cr)*10.0 ) ))
        return pack('!I', cr)

class CW_Read(PK184ReadCmd):
    def __init__(self):
        super().__init__()
        self.register = 0x011E

    def decode(self, data):
        self.cw = unpack('!I', data)[0] / 100.0
        return self.cw

class CW_Write(PK184WriteCmd):
    def __init__(self, cw: float):
        super().__init__()
        self.register = 0x011E
        self.cw = cw
    
    def encode(self):
        cw = max(0, min(2500, round( float(self.cw*100) ) ))
        return pack('!I', cw)

class Voltage_Read(PK184ReadCmd):
    def __init__(self):
        super().__init__()
        self.register = 0x0122

    def decode(self, data):
        self.voltage = unpack('!I', data)[0] / 1000.0
        return self.voltage

class Power_Read(PK184ReadCmd):
    def __init__(self):
        super().__init__()
        self.register = 0x0124

    def decode(self, data):
        self.power = unpack('!I', data)[0] / 10.0
        return self.power

class Current_Read(PK184ReadCmd):
    def __init__(self):
        super().__init__()
        self.register = 0x0126

    def decode(self, data):
        self.current = unpack('!I', data)[0] / 1000.0
        return self.current

class VoltageCurrent_Read(PK184ReadCmd):
    def __init__(self):
        super().__init__()
        self.register = 0x0300 # special command
        self.respDataLen = 18 # @override

    def decode(self, data):
        modeStr = ["CV", "CC", "CR", "CW"]
        self.onOff = bool(data[0] & 0x01)
        self.mode = modeStr[(data[0]>>1) & 0x03]
        voltageCurrent = unpack('!II', b'\x00' + data[2:5] + b'\x00' + data[5:8])
        self.voltage = voltageCurrent[0] / 1000.0
        self.current = voltageCurrent[1] / 1000.0
        return (self.onOff, self.mode, self.voltage, self.current)

class OnloadLvlV_Write(PK184WriteCmd):
    def __init__(self, onloadLvlV: str):
        super().__init__()
        self.register = 0x012A
        self.onloadLvlV = onloadLvlV
    
    def encode(self):
        onloadLvlV = max(0, min(150000, round( float(self.onloadLvlV)*1000.0 ) ))
        return pack('!I', onloadLvlV)

class OnloadLvlV_Read(PK184ReadCmd):
    def __init__(self):
        super().__init__()
        self.register = 0x012A

    def decode(self, data):
        self.onloadLvlV = unpack('!I', data)[0] / 1000.0
        return self.onloadLvlV

class SlewRateUp_Write(PK184WriteCmd):
    def __init__(self, aMsUp: float):
        super().__init__()
        self.register = 0x012C
        self.aMsUp = aMsUp
    
    def encode(self):
        aMsUp = max(0, min(4000, round( float(self.aMsUp)*10.0 ) ))
        return pack('!I', aMsUp)

class SlewRateDown_Write(PK184WriteCmd):
    def __init__(self, aMsDown: float):
        super().__init__()
        self.register = 0x012E
        self.aMsDown = aMsDown
    
    def encode(self):
        aMsDown = max(0, min(4000, round( float(self.aMsDown)*10.0 ) ))
        return pack('!I', aMsDown)

class SlewRateUp_Read(PK184ReadCmd):
    def __init__(self):
        super().__init__()
        self.register = 0x012C

    def decode(self, data):
        self.aMsUp = unpack('!I', data)[0] / 10.0
        return self.aMsUp

class SlewRateDown_Read(PK184ReadCmd):
    def __init__(self):
        super().__init__()
        self.register = 0x012E

    def decode(self, data):
        self.aMsDown = unpack('!I', data)[0] / 10.0
        return self.aMsDown

class PassFailOut_Write(PK184WriteCmd):
    def __init__(self, fail: bool):
        super().__init__()
        self.register = 0x0131
        self.fail = fail
    
    def encode(self):
        return pack('!I', int(self.fail))

class DynaTestOnOff_Write(PK184WriteCmd):
    def __init__(self, onOff: bool):
        super().__init__()
        self.register = 0x0132
        self.onOff = onOff
    
    def encode(self):
        return pack('!I', int(self.onOff))

class DynaTestOnOff_Read(PK184ReadCmd):
    def __init__(self):
        super().__init__()
        self.register = 0x0132

    def decode(self, data):
        self.onOff = bool(unpack('!I', data)[0] & 0x01)
        return self.onOff

class DynaACurrent_Read(PK184ReadCmd): #todo: reg 0x0134-0x140 not working?
    def __init__(self):
        super().__init__()
        self.register = 0x0134

    def decode(self, data):
        self.current = unpack('!I', data)[0] / 1000.0
        return self.current

class DynaACurrent_Write(PK184WriteCmd): #todo: reg 0x0134-0x140 not working?
    def __init__(self, current: float):
        super().__init__()
        self.register = 0x0134
        self.current = current
    
    def encode(self):
        current = max(0, min(40000, round( float(self.current)*1000.0 ) ))
        return pack('!I', current)

class DynaATime_Read(PK184ReadCmd): #todo: reg 0x0134-0x140 not working?
    def __init__(self):
        super().__init__()
        self.register = 0x0138

    def decode(self, data):
        self.current = unpack('!I', data)[0] / 1000.0
        return self.current

class DynaATime_Write(PK184WriteCmd): #todo: reg 0x0134-0x140 not working?
    def __init__(self, time: float):
        super().__init__()
        self.register = 0x0138
        self.time = time
    
    def encode(self):
        time = max(0, round( float(self.time)*1000.0 ) )
        return pack('!I', time)

class DynaBCurrent_Read(PK184ReadCmd): #todo: reg 0x0134-0x140 not working?
    def __init__(self):
        super().__init__()
        self.register = 0x013C

    def decode(self, data):
        self.current = unpack('!I', data)[0] / 1000.0
        return self.current

class DynaBCurrent_Write(PK184WriteCmd): #todo: reg 0x0134-0x140 not working?
    def __init__(self, current: float):
        super().__init__()
        self.register = 0x013C
        self.current = current
    
    def encode(self):
        current = max(0, min(40000, round( float(self.current)*1000.0 ) ))
        return pack('!I', current)

class DynaBTime_Read(PK184ReadCmd): #todo: reg 0x0134-0x140 not working?
    def __init__(self):
        super().__init__()
        self.register = 0x0140

    def decode(self, data):
        self.current = unpack('!I', data)[0] / 1000.0
        return self.current

class DynaBTime_Write(PK184WriteCmd): #todo: reg 0x0134-0x140 not working?
    def __init__(self, time: float):
        super().__init__()
        self.register = 0x0140
        self.time = time
    
    def encode(self):
        time = max(0, round( float(self.time)*1000.0 ) )
        return pack('!I', time)

class DynaBTime_Read(PK184ReadCmd):  #todo: reg 0x0134-0x140 not working?
    def __init__(self):
        super().__init__()
        self.register = 0x0140

    def decode(self, data):
        self.current = unpack('!I', data)[0] / 1000.0
        return self.current

class DynaBTime_Write(PK184WriteCmd):  #todo: reg 0x0134-0x140 not working?
    def __init__(self, time: float):
        super().__init__()
        self.register = 0x0140
        self.time = time
    
    def encode(self):
        time = max(0, round( float(self.time)*1000.0 ) )
        return pack('!I', time)

class DynaAB_Write(PK184WriteMCmd): #todo: reg 0x0134-0x140 not working?
    def __init__(self, currentA: float, currentB: float, timeA: float, timeB: float):
        super().__init__()
        self.register = 0x0134
        self.currentA = currentA
        self.currentB = currentB
        self.timeA = timeA
        self.timeB = timeB
    
    def encode(self):
        currentA = max(0, min(40000, round( float(self.currentA)*1000.0 ) ))
        currentB = max(0, min(40000, round( float(self.currentB)*1000.0 ) ))
        timeA = max(0, round( float(self.timeA)*1000.0 ) )
        timeB = max(0, round( float(self.timeB)*1000.0 ) )
        return pack('!HBIIII', 4, 16, currentA, currentB, timeA, timeB)

class BattTestOnOff_Write(PK184WriteCmd):
    def __init__(self, onOff: bool):
        super().__init__()
        self.register = 0x0144
        self.onOff = onOff
    
    def encode(self):
        return pack('!I', int(self.onOff))

class BattTestOnOff_Read(PK184ReadCmd):
    def __init__(self):
        super().__init__()
        self.register = 0x0144

    def decode(self, data):
        self.onOff = bool(unpack('!I', data)[0] & 0x01)
        return self.onOff

class BattTestEndVoltage_Read(PK184ReadCmd):
    def __init__(self):
        super().__init__()
        self.register = 0x0146

    def decode(self, data):
        self.voltage = unpack('!I', data)[0] / 1000.0
        return self.voltage

class BattTestEndVoltage_Write(PK184WriteCmd):
    def __init__(self, voltage: float):
        super().__init__()
        self.register = 0x0146
        self.voltage = voltage
    
    def encode(self):
        voltage = max(0,  min(150000, round( float(self.voltage)*1000.0 ) ))
        return pack('!I', voltage)

class BattTestCapacityResult_Read(PK184ReadCmd):
    def __init__(self):
        super().__init__()
        self.register = 0x0148

    def decode(self, data):
        self.capacity = unpack('!I', data)[0] / 1000.0
        return self.capacity

class BattTestHalfCurrOnOff_Read(PK184ReadCmd):
    def __init__(self):
        super().__init__()
        self.register = 0x014A

    def decode(self, data):
        self.onOff = bool(unpack('!I', data)[0] & 0x01)
        return self.onOff

class BattTestHalfCurrOnOff_Write(PK184WriteCmd): #todo: reg 0x014A is not writeable?
    def __init__(self, onOff: bool):
        super().__init__()
        self.register = 0x014A
        self.onOff = onOff
    
    def encode(self):
        return pack('!I', int(self.onOff))

class BattTestUnit_Read(PK184ReadCmd):
    def __init__(self):
        super().__init__()
        self.register = 0x014C

    def decode(self, data):
        self.unit = "WH" if (unpack('!I', data)[0] & 0x01) == 1 else "AH"
        return self.unit

class BattTestUnit_Write(PK184WriteCmd):
    def __init__(self, unit: str):
        super().__init__()
        self.register = 0x014C
        self.unit = 0x01 if unit.upper() == "WH" else 0x00
    
    def encode(self):
        return pack('!I', int(self.unit))

class BattTestSignal_Read(PK184ReadCmd):
    def __init__(self):
        super().__init__()
        self.register = 0x014E

    def decode(self, data):
        self.signal = unpack('!I', data)[0] & 0x03
        return self.signal

class BattTestSignal_Write(PK184WriteCmd):
    def __init__(self, signal: int):
        super().__init__()
        self.register = 0x014E
        self.signal = signal
    
    def encode(self):
        return pack('!I', int(self.signal) & 0x03)

class BattTestAll_Read(PK184ReadCmd):
    def __init__(self):
        super().__init__()
        self.register = 0x0301 # special command
        self.respDataLen = 32 # @override

    def decode(self, data):
        modeStr = ["CV", "CC", "CR", "CW"]
        self.onOff = bool(data[0] & 0x01)
        self.mode = modeStr[(data[0]>>1) & 0x03]
        voltageCurrent = unpack('!II', b'\x00' + data[2:5] + b'\x00' + data[5:8])
        self.voltage = voltageCurrent[0] / 1000.0
        self.current = voltageCurrent[1] / 1000.0
        
        battTest = unpack('!BBBIII', data[18:21] + b'\x00' + data[21:24] + data[24:32])
        self.halfCurrentEnabled = bool(battTest[0] & 0x01)
        self.unit = battTest[1]
        self.signal = battTest[2]
        self.endofchargeVoltage = battTest[3] / 1000.0
        self.capacityAh = battTest[4] / 1000.0
        self.capacityWh = battTest[5] / 1000.0
        return (self.onOff, self.mode, self.halfCurrentEnabled, self.unit, self.signal, self.endofchargeVoltage, self.capacityAh, self.capacityWh)

class CompareOnOff_Write(PK184WriteCmd): # todo: write to reg 0x0180 not working?
    def __init__(self, onOff: bool):
        super().__init__()
        self.register = 0x0180
        self.onOff = onOff
    
    def encode(self):
        return pack('!I', int(self.onOff))

class CompareOnOff_Read(PK184ReadCmd):
    def __init__(self):
        super().__init__()
        self.register = 0x0180

    def decode(self, data):
        self.onOff = unpack('!I', data)[0] & 0x03
        return self.onOff

class CompareVoltageHigh_Read(PK184ReadCmd):
    def __init__(self):
        super().__init__()
        self.register = 0x0182

    def decode(self, data):
        self.voltage = unpack('!I', data)[0] / 1000.0
        return self.voltage

class CompareVoltageHigh_Write(PK184WriteCmd):
    def __init__(self, voltage: float):
        super().__init__()
        self.register = 0x0182
        self.voltage = voltage
    
    def encode(self):
        voltage = max(0,  min(150000, round( float(self.voltage)*1000.0 ) ))
        return pack('!I', voltage)

class CompareVoltageLow_Read(PK184ReadCmd):
    def __init__(self):
        super().__init__()
        self.register = 0x0184

    def decode(self, data):
        self.voltage = unpack('!I', data)[0] / 1000.0
        return self.voltage

class CompareVoltageLow_Write(PK184WriteCmd):
    def __init__(self, voltage: float):
        super().__init__()
        self.register = 0x0184
        self.voltage = voltage
    
    def encode(self):
        voltage = max(0,  min(150000, round( float(self.voltage)*1000.0 ) ))
        return pack('!I', voltage)

class CompareCurrentHigh_Read(PK184ReadCmd):
    def __init__(self):
        super().__init__()
        self.register = 0x0186

    def decode(self, data):
        self.current = unpack('!I', data)[0] / 1000.0
        return self.current

class CompareCurrentHigh_Write(PK184WriteCmd):
    def __init__(self, current: float):
        super().__init__()
        self.register = 0x0186
        self.current = current
    
    def encode(self):
        current = max(0, min(40000, round( float(self.current)*1000.0 ) ))
        return pack('!I', current)

class CompareCurrentLow_Read(PK184ReadCmd):
    def __init__(self):
        super().__init__()
        self.register = 0x0188

    def decode(self, data):
        self.current = unpack('!I', data)[0] / 1000.0
        return self.current

class CompareCurrentLow_Write(PK184WriteCmd):
    def __init__(self, current: float):
        super().__init__()
        self.register = 0x0188
        self.current = current
    
    def encode(self):
        current = max(0, min(40000, round( float(self.current)*1000.0 ) ))
        return pack('!I', current)

class CompareFailAlarm_Read(PK184ReadCmd):
    def __init__(self):
        super().__init__()
        self.register = 0x018A

    def decode(self, data):
        self.alarm = unpack('!I', data)[0] & 0x03
        return self.alarm

class CompareFailAlarm_Write(PK184WriteCmd):
    def __init__(self, alarm: int):
        super().__init__()
        self.register = 0x018A
        self.alarm = alarm
    
    def encode(self):
        return pack('!I', int(self.alarm) & 0x03)

def main():

    try:
        if len(sys.argv) < 3:
            printHelp()
            exit()
        kp = KP184(sys.argv[1], 1) # rs485port, address=1

        try:
            method_to_call = getattr(kp, sys.argv[2])
        except AttributeError:
            print(f"Unknown cmd {sys.argv[2]}")
            exit()

        result = method_to_call(*sys.argv[3:])
        print(result)

    except KeyboardInterrupt:
        exit()

def printHelp():
    print("kp184.py cmd [value]")
    print("Commands:")
    for cmdname in filter(lambda name: name.startswith(('read', 'write')), dir(KP184)):
        print(f" - {cmdname}")

if __name__ == "__main__":
    main()