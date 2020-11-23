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
        self.dig_press_addr = 0x8E
        self.dig_press_len = 18

        self.ctrl_hum_val = 0x00
        self.status_val = 0x00
        self.ctrl_meas_val = 0x00
        self.config_val = 0x00
        self.press_val = 0x00
        self.temp_val = 0x00
        self.hum_val = 0x00

        self.t_fine = 0

        self.adc_T = 0
        self.adc_P = 0
        self.adc_H = 0
        self.T = 0
        self.P = 0
        self.H = 0
    
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

    def readTemp(self):
        temp_readout = self.bus.read_i2c_block_data(self.sensor_addr, self.temp_addr, self.temp_len)
        dig_regs_temp = self.bus.read_i2c_block_data(self.sensor_addr, self.dig_temp_addr, self.dig_temp_len)

        dig_T1 = np.uint16((dig_regs_temp[1] << 8) | dig_regs_temp[0])
        dig_T2 = np.int16((dig_regs_temp[3] << 8) | dig_regs_temp[2])
        dig_T3 = np.int16((dig_regs_temp[5] << 8) | dig_regs_temp[4])

        temp_xlsb = np.uint8(temp_readout[2])
        temp_lsb = np.uint8(temp_readout[1])
        temp_msb = np.uint8(temp_readout[0])

        self.adc_T = np.int32(((temp_msb & 255) << 12) | ((temp_lsb & 255) << 4) | ((temp_xlsb & 0b11110000) >> 4))

        dig_T1 = np.int32(dig_T1)
        dig_T2 = np.int32(dig_T2)
        dig_T3 = np.int32(dig_T3)
        self.adc_T =  np.int32(self.adc_T)

        var1 = np.int32((((self.adc_T>>3) - (dig_T1<<1)) * (dig_T2)) >> 11)
        var2 = np.int32((((self.adc_T>>4) - dig_T1) * ((self.adc_T>>4) - dig_T1) >> 12) * (dig_T3) >> 14)
        self.t_fine = np.int32(var1 + var2)
        self.T = np.int32((self.t_fine * 5 + 128)>>8)

    def readPress(self):
        press_readout = self.bus.read_i2c_block_data(self.sensor_addr, self.press_addr, self.press_len)
        dig_regs_press = self.bus.read_i2c_block_data(self.sensor_addr, self.dig_press_addr, self.dig_press_len)

        dig_P1 = np.uint16((dig_regs_press[1] << 8) | dig_regs_press[0])
        dig_P2 = np.int16((dig_regs_press[3] << 8) | dig_regs_press[2])
        dig_P3 = np.int16((dig_regs_press[5] << 8) | dig_regs_press[4])
        dig_P4 = np.int16((dig_regs_press[7] << 8) | dig_regs_press[6])
        dig_P5 = np.int16((dig_regs_press[9] << 8) | dig_regs_press[8])
        dig_P6 = np.int16((dig_regs_press[11] << 8) | dig_regs_press[10])
        dig_P7 = np.int16((dig_regs_press[13] << 8) | dig_regs_press[12])
        dig_P8 = np.int16((dig_regs_press[15] << 8) | dig_regs_press[14])
        dig_P9 = np.int16((dig_regs_press[17] << 8) | dig_regs_press[16])

        press_xlsb = np.uint8(press_readout[2])
        press_lsb = np.uint8(press_readout[1])
        press_msb = np.uint8(press_readout[0])

        self.adc_P = np.int32(((press_msb & 255) << 12) | ((press_lsb & 255) << 4) | ((press_xlsb & 0b11110000) >> 4))
        var1 = np.int64(self.t_fine) - 128000
        var2 = var1 * var1 * np.int64(dig_P6)
        var2 = var2 + ((var1 * np.int64(dig_P5)) << 17)
        var2 = var2 + (np.int64(dig_P4) << 35)
        var1 = ((var1 * var1 * np.int64(dig_P3)) >> 8) + ((var1 * np.int64(dig_P2)) << 12)
        var1 = ((np.int64(1) << 47) + var1) * (np.int64(dig_P1)) >> 33
        if var1 == 0:
            self.P = 0
            return
        
        self.P = np.int64(1048576 - self.adc_P)
        self.P = (((self.P << 31) - var2) * 3125) / var1
        var1 = np.int64(dig_P9) * (self.P >> 13) * (self.P >> 13) >> 25
        var2 = (np.int64(dig_P8) * self.P) >> 19
        self.P = ((self.P + var1 + var2) >> 8) + (np.int64(dig_P7) << 4)
        self.P = np.uint32(self.P)

    def readHum(self):
        hum_readout = self.bus.read_i2c_block_data(self.sensor_addr, self.hum_addr, self.hum_len)
        dig_regs_hum1 = self.bus.read_i2c_block_data(self.sensor_addr, 0xA1, 1)
        dig_regs_hum2 = self.bus.read_i2c_block_data(self.sensor_addr, 0xE1, 7)

        dig_H1 = np.uint8(dig_regs_hum1[0])
        dig_H2 = np.int16((dig_regs_hum2[1] << 8) | dig_regs_hum2[0])
        dig_H3 = np.uint8(dig_regs_hum2[2])
        dig_H4 = np.int16((dig_regs_hum2[3] << 4) | (dig_regs_hum2[4] & 0b1111))
        dig_H5 = np.int16((dig_regs_hum2[5] << 4) | (dig_regs_hum2[4] >> 4))
        dig_H6 = np.int8(dig_regs_hum2[6])

        hum_lsb = np.uint8(hum_readout[1])
        hum_msb = np.uint8(hum_readout[0])

        self.adc_H = np.int32((hum_msb << 8) | (hum_lsb))
        v_x1_u32r = np.int32(self.t_fine - np.int32(76800))
        v_x1_u32r = (((((self.adc_H << 14) - ((np.int32(dig_H4)) << 20) - ((np.int32(dig_H5)) * v_x1_u32r)) + (np.int32(16384))) >> 15) * (((((((v_x1_u32r * (np.int32(dig_H6))) >> 10) * (((v_x1_u32r * (np.int32(dig_H3))) >> 11) + (np.int32(32768)))) >> 10) + (np.int32(2097152))) * (np.int32(dig_H2)) + 8192) >> 14))
        v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * (np.int32(dig_H1))) >> 4))
        v_x1_u32r = 0 if v_x1_u32r < 0 else v_x1_u32r
        v_x1_u32r = 419430400 if v_x1_u32r > 419430400 else v_x1_u32r
        self.H = np.uint32(v_x1_u32r >> 12)

    def readout(self):
        self.readTemp()
        self.readPress()
        self.readHum()

        return [self.T, self.P, self.H]

    def simple_readTemp(self):
        self.setFilterCoeff(BME280Filter.COEFFOFF)
        self.setOsrsHum(BME280OS.SKIP)
        self.setOsrsPress(BME280OS.OS1)
        self.setOsrsTemp(BME280OS.OS1)
        self.setMode(BME280Mode.FORCED)
        self.sendCommand()
        self.readout()
        return [self.T, self.P, self.H]

    

sensor = BME280(0x76)
data = sensor.simple_readTemp()

print("T:" + str(float(data[0])/100) + " C")
print("P:" + str(float(data[1])/256/100) + " hPa")
print("H:" + str(float(data[2])/1024) + " %RH")