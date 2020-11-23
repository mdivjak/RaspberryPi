import numpy as np
from smbus2 import SMBus
from enum import Enum

class BME280OS(Enum):
    SKIP = 1
    OS1 = 2
    OS2 = 3
    OS4 = 4
    OS8 = 5
    OS16 = 6

class BME280Mode(Enum):
    SLEEP = 1
    FORCED = 2
    NORMAL = 3

class BME280Standby(Enum):
    STANDBY0_5 = 1
    STANDBY62_5 = 2
    STANDBY125 = 3
    STANDBY250 = 4
    STANDBY500 = 5
    STANDBY1000 = 6
    STANDBY10 = 7
    STANDBY20 = 8

class BME280Filter(Enum):
    COEFFOFF = 1
    COEFF2 = 2
    COEFF4 = 3
    COEFF8 = 4
    COEFF16 = 5


class BME280:
    def __init__(self, sensorAddress):
        self.sensor_addr =  sensorAddress
        self.bus = SMBus(1)

        # chip id number; equals to 0x60
        self.id_addr = 0xD1
        # if 0xB6 at this address, chip resets
        self.reset_addr = 0xE0
        # humidity data acquisition options
        # changes effective after a write operation to ctrl_meas
        # bit 2,1,0 osrs_h[2..0] oversampling of humidity data
        # 000           skipped (output se to 0x8000)
        # 001           oversampling x 1
        # 010           oversampling x 2
        # 011           oversampling x 4
        # 100           oversampling x 8
        # 101, others   oversampling x 16
        self.ctrl_hum_addr = 0xF2
        # register indicates status of device
        # bit 3 - measuring[0]
        #   1 - when conversion is running
        #   0 - when results have been transferred to the data regs
        # bit 0 - im_update[0]
        #   1 - when NVM data are being copied to image registers
        #   0 - when copying is done
        self.status_addr = 0xF3
        # pressure and temperature data acquisition options
        # needs to be written after ctrl_hum for changes to be effective
        # bit 7,6,5 - osrs_t[2..0] oversampling of temp
        # bit 4,3,2 - osrs_p[2..0] oversampling of press
        # oversampling table is the same as for humidity
        # bit 1,0 - mode[1..0] controls sensor mode
        #   00          sleep mode
        #   01 or 10    forced mode
        #   11          normal mode
        self.ctrl_meas_addr = 0xF4
        # sets rate, filter and interface options
        # bit 7,6,5 - t_sb[2..0] controls inactive duration in normal mode
        # 000 0.5 ms
        # 001 62.5 ms
        # 010 125 ms
        # 011 250 ms
        # 100 500 ms
        # 101 1000 ms
        # 110 10 ms
        # 111 20 ms
        # bit 4,3,2 - filter[2..0]
        # 000         filter off
        # 001         2
        # 010         4
        # 011         8
        # 100, others 16
        # bit 0 - spi3w_en enables 3-wire SPI interface when set to 1
        self.config_addr = 0xF5
        # 0xF7 press_msb[7..0] raw pressure measurement bits 19..12
        # 0xF8 press_lsb[7..0] raw pressure measurement bits 11..4
        # 0xF9 bits 7..4 press_xlsb[3..0] raw pressure measurement bits 3..0
        self.press_addr = 0xF7
        self.press_len = 3
        # 0xFA temp_msb[7..0] raw temperature measurement bits 19..12
        # 0xFB temp_lsb[7..0] raw temperature measurement bits 11..4
        # 0xFC bits 7..4 temp_xlsb[3..0] raw temperature measurement bits 3..0
        self.temp_addr = 0xFA
        self.temp_len = 3
        # 0xFD hum_msb[7..0] raw humidity measurement bits 15..8
        # 0xFE hum_lsb[7..0] raw humidity measurement bits 7..0
        self.hum_addr = 0xFD
        self.hum_len = 2

        self.dig_temp_addr = 0x88
        self.dig_temp_len = 6

        self.ctrl_hum_val = 0x00
        self.status_val = 0x00
        self.ctrl_meas_val = 0x00
        self.config_val = 0x00
        self.press_val = 0x00
        self.temp_val = 0x00
        self.hum_val = 0x00

        self.dig_T1 = 0x00
        self.dig_T2 = 0x00
        self.dig_T3 = 0x00

        self.dig_P1 = 0x00
        self.dig_P2 = 0x00
        self.dig_P3 = 0x00
        self.dig_P4 = 0x00
        self.dig_P5 = 0x00
        self.dig_P6 = 0x00
        self.dig_P7 = 0x00
        self.dig_P8 = 0x00
        self.dig_P9 = 0x00

        self.dig_H1 = 0x00
        self.dig_H2 = 0x00
        self.dig_H3 = 0x00
        self.dig_H4 = 0x00
        self.dig_H5 = 0x00

        self.adc_T = 0
        self.T = 0
    
    def setMode(self, mode):
        if mode == BME280Mode.SLEEP:
            self.ctrl_meas_val &= ~0 << 2
        elif mode == BME280Mode.FORCED:
            self.ctrl_meas_val &= ~0 << 2
            self.ctrl_meas_val |= 0b01
        elif mode == BME280Mode.NORMAL:
            self.ctrl_meas_val &= ~0 << 2
            self.ctrl_meas_val |= 0b11
        else:
            raise NameError("Invalid sensor mode")

    def setOsrsHum(self, osrs):
        if osrs == BME280OS.SKIP:
            self.ctrl_hum_val &= ~0 << 3
            self.ctrl_hum_val |= 0b000
        elif osrs == BME280OS.OS1:
            self.ctrl_hum_val &= ~0 << 3
            self.ctrl_hum_val |= 0b001
        elif osrs == BME280OS.OS2:
            self.ctrl_hum_val &= ~0 << 3
            self.ctrl_hum_val |= 0b010
        elif osrs == BME280OS.OS4:
            self.ctrl_hum_val &= ~0 << 3
            self.ctrl_hum_val |= 0b011
        elif osrs == BME280OS.OS8:
            self.ctrl_hum_val &= ~0 << 3
            self.ctrl_hum_val |= 0b100
        elif osrs == BME280OS.OS16:
            self.ctrl_hum_val &= ~0 << 3
            self.ctrl_hum_val |= 0b101
        else:
            raise NameError("Invalid Oversampling value")

    def setOsrsTemp(self, osrs):
        if osrs == BME280OS.SKIP:
            self.ctrl_meas_val &= (~0 << 8) | (0b00011111)
            self.ctrl_meas_val |= 0b00011111
        elif osrs == BME280OS.OS1:
            self.ctrl_meas_val &= (~0 << 8) | (0b00011111)
            self.ctrl_meas_val |= 0b00111111
        elif osrs == BME280OS.OS2:
            self.ctrl_meas_val &= (~0 << 8) | (0b00011111)
            self.ctrl_meas_val |= 0b01011111
        elif osrs == BME280OS.OS4:
            self.ctrl_meas_val &= (~0 << 8) | (0b00011111)
            self.ctrl_meas_val |= 0b01111111
        elif osrs == BME280OS.OS8:
            self.ctrl_meas_val &= (~0 << 8) | (0b00011111)
            self.ctrl_meas_val |= 0b10011111
        elif osrs == BME280OS.OS16:
            self.ctrl_meas_val &= (~0 << 8) | (0b00011111)
            self.ctrl_meas_val |= 0b10111111
        else:
            raise NameError("Invalid Oversampling value")

    def setOsrsPress(self, osrs):
        if osrs == BME280OS.SKIP:
            self.ctrl_meas_val &= (~0 << 5) | (0b11100011)
            self.ctrl_meas_val |= 0b11100011
        elif osrs == BME280OS.OS1:
            self.ctrl_meas_val &= (~0 << 5) | (0b11100011)
            self.ctrl_meas_val |= 0b11100111
        elif osrs == BME280OS.OS2:
            self.ctrl_meas_val &= (~0 << 5) | (0b11100011)
            self.ctrl_meas_val |= 0b11101011
        elif osrs == BME280OS.OS4:
            self.ctrl_meas_val &= (~0 << 5) | (0b11100011)
            self.ctrl_meas_val |= 0b11101111
        elif osrs == BME280OS.OS8:
            self.ctrl_meas_val &= (~0 << 5) | (0b11100011)
            self.ctrl_meas_val |= 0b11110011
        elif osrs == BME280OS.OS16:
            self.ctrl_meas_val &= (~0 << 5) | (0b11100011)
            self.ctrl_meas_val |= 0b11110111
        else:
            raise NameError("Invalid Oversampling value")
            
    def setRate(self, rate):
        if rate == BME280Standby.STANDBY0_5:
            self.config_val &= (~0 << 8) | 0b00011111
        elif rate == BME280Standby.STANDBY62_5:
            self.config_val &= (~0 << 8) | 0b00011111
        elif rate == BME280Standby.STANDBY125:
            self.config_val &= (~0 << 8) | 0b00011111
        elif rate == BME280Standby.STANDBY250:
            self.config_val &= (~0 << 8) | 0b00011111
        elif rate == BME280Standby.STANDBY500:
            self.config_val &= (~0 << 8) | 0b00011111
        elif rate == BME280Standby.STANDBY1000:
            self.config_val &= (~0 << 8) | 0b00011111
        elif rate == BME280Standby.STANDBY10:
            self.config_val &= (~0 << 8) | 0b00011111
        elif rate == BME280Standby.STANDBY20:
            self.config_val &= (~0 << 8) | 0b00011111
        else:
            raise NameError("Invalid Standby rate")
    
    def setFilterCoeff(self, coeff):
        if coeff == BME280Filter.COEFFOFF:
            self.config_val &= (~0 << 5) | 0b11100011
        elif coeff == BME280Filter.COEFF2:
            self.config_val &= (~0 << 5) | 0b11100111
        elif coeff == BME280Filter.COEFF4:
            self.config_val &= (~0 << 5) | 0b11101011
        elif coeff == BME280Filter.COEFF8:
            self.config_val &= (~0 << 5) | 0b11101111
        elif coeff == BME280Filter.COEFF16:
            self.config_val &= (~0 << 5) | 0b11110011
        else:
            raise NameError("Invalid filter coefficient")
    
    def sendCommand(self):
        self.bus.write_byte_data(self.sensor_addr, self.config_addr, self.config_val)
        self.bus.write_byte_data(self.sensor_addr, self.ctrl_hum_addr, self.ctrl_hum_val)
        self.bus.write_byte_data(self.sensor_addr, self.ctrl_meas_addr, self.ctrl_meas_val)

    def readout(self):
        mem = {}
        temp_readout = self.bus.read_i2c_block_data(self.sensor_addr, self.temp_addr, self.temp_len)
        dig_regs = self.bus.read_i2c_block_data(self.sensor_addr, self.dig_temp_addr, self.dig_temp_len)

        #unsigned short
        self.dig_T1 = np.uint16((dig_regs[1] << 8) | dig_regs[0])
        #signed short
        self.dig_T2 = np.int16((dig_regs[3] << 8) | dig_regs[2])
        #signed short
        self.dig_T3 = np.int16((dig_regs[5] << 8) | dig_regs[4])

        temp_xlsb = np.uint8(temp_readout[2])
        temp_lsb = np.uint8(temp_readout[1])
        temp_msb = np.uint8(temp_readout[0])

        self.adc_T = np.int32(((temp_msb & 255) << 12) | ((temp_lsb & 255) << 4) | ((temp_xlsb & 0b11110000) >> 4))

        self.dig_T1 = np.int32(self.dig_T1)
        self.dig_T2 = np.int32(self.dig_T2)
        self.dig_T3 = np.int32(self.dig_T3)
        self.adc_T =  np.int32(self.adc_T)

        var1 = np.int32((((self.adc_T>>3) - (self.dig_T1<<1)) * (self.dig_T2)) >> 11)
        var2 = np.int32((((self.adc_T>>4) - self.dig_T1) * ((self.adc_T>>4) - self.dig_T1) >> 12) * (self.dig_T3) >> 14)
        t_fine = np.int32(var1 + var2)
        self.T = np.int32((t_fine * 5 + 128)>>8)
        return self.T

    def simple_readTemp(self):
        self.setFilterCoeff(BME280Filter.COEFFOFF)
        self.setOsrsHum(BME280OS.SKIP)
        self.setOsrsPress(BME280OS.SKIP)
        self.setOsrsTemp(BME280OS.OS1)
        self.setMode(BME280Mode.FORCED)
        self.sendCommand()
        self.readout()
        return self.T

    

sensor = BME280(0x76)
temp = sensor.simple_readTemp()

print("T:" + str(temp))