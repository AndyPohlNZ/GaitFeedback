#!/usr/bin/python3

import struct

""" ADXL345 Class
Samples from ADXL345 Accelerometer via I2C

Originally created by Adafruit and released under MIT liscence
Retreived from: https://github.com/adafruit/Adafruit_Python_ADXL345/blob/master/Adafruit_ADXL345/ADXL345.py

Modified by Andy Pohl
            Faculty of Kinesiology - University of Calgary
            Decemeber 2019

"""

# Minimal constants carried over from Arduino library
ADXL345_ADDRESS          = 0x53
ADXL345_REG_DEVID        = 0x00 # Device ID
ADXL345_REG_DATAX0       = 0x32 # X-axis data 0 (6 bytes for X/Y/Z)
ADXL345_REG_POWER_CTL    = 0x2D # Power-saving features control
ADXL345_REG_DATA_FORMAT  = 0x31
ADXL345_REG_BW_RATE      = 0x2C
ADXL345_DATARATE_0_10_HZ = 0x00
ADXL345_DATARATE_0_20_HZ = 0x01
ADXL345_DATARATE_0_39_HZ = 0x02
ADXL345_DATARATE_0_78_HZ = 0x03
ADXL345_DATARATE_1_56_HZ = 0x04
ADXL345_DATARATE_3_13_HZ = 0x05
ADXL345_DATARATE_6_25HZ  = 0x06
ADXL345_DATARATE_12_5_HZ = 0x07
ADXL345_DATARATE_25_HZ   = 0x08
ADXL345_DATARATE_50_HZ   = 0x09
ADXL345_DATARATE_100_HZ  = 0x0A # (default)
ADXL345_DATARATE_200_HZ  = 0x0B
ADXL345_DATARATE_400_HZ  = 0x0C
ADXL345_DATARATE_800_HZ  = 0x0D
ADXL345_DATARATE_1600_HZ = 0x0E
ADXL345_DATARATE_3200_HZ = 0x0F
ADXL345_RANGE_2_G        = 0x00 # +/-  2g (default)
ADXL345_RANGE_4_G        = 0x01 # +/-  4g
ADXL345_RANGE_8_G        = 0x02 # +/-  8g
ADXL345_RANGE_16_G       = 0x03 # +/- 16g

LSB_MULTIPLIER = 0.004
EARTH_GRAVITY = 9.80665

class ADXL345(object):
    """ADXL345 triple-axis accelerometer."""

    def __init__(self, address=ADXL345_ADDRESS, i2c=None, **kwargs):
        """Initialize the ADXL345 accelerometer using its I2C interface.
        """
        # Setup I2C interface for the device.
        if i2c is None:
            import Adafruit_GPIO.I2C as I2C
            i2c = I2C
        self._device = i2c.get_i2c_device(address, **kwargs)
        # Check that the acclerometer is connected, then enable it.
        if self._device.readU8(ADXL345_REG_DEVID) == 0xE5:
            self._device.write8(ADXL345_REG_POWER_CTL, 0x08)
        else:
            raise RuntimeError('Failed to find the expected device ID register value, check your wiring.')

        print("adxl345 connected")


    def set_range(self, value):
        """Set the range of the accelerometer to the provided value.  Range value
        should be one of these constants:
          - ADXL345_RANGE_2_G   = +/-2G
          - ADXL345_RANGE_4_G   = +/-4G
          - ADXL345_RANGE_8_G   = +/-8G
          - ADXL345_RANGE_16_G  = +/-16G
        """
        # Read the data format register to preserve bits.  Update the data
        # rate, make sure that the FULL-RES bit is enabled for range scaling
        format_reg = self._device.readU8(ADXL345_REG_DATA_FORMAT) & ~0x0F
        format_reg |= value
        format_reg |= 0x08  # FULL-RES bit enabled
        # Write the updated format register.
        self._device.write8(ADXL345_REG_DATA_FORMAT, format_reg)
        
        if value == ADXL345_RANGE_2_G:
            self.range = 2
        elif value == ADXL345_RANGE_4_G:
            self.range = 4
        elif value == ADXL345_RANGE_8_G:
            self.range = 8
        elif value == ADXL345_RANGE_16_G:
            self.range = 16

    def get_range(self):
        """Retrieve the current range of the accelerometer.  See set_range for
        the possible range constant values that will be returned.
        """
        return self._device.readU8(ADXL345_REG_DATA_FORMAT) & 0x03

    def set_data_rate(self, rate):
        """Set the data rate of the aceelerometer.  Rate should be one of the
        following constants:
          - ADXL345_DATARATE_0_10_HZ = 0.1 hz
          - ADXL345_DATARATE_0_20_HZ = 0.2 hz
          - ADXL345_DATARATE_0_39_HZ = 0.39 hz
          - ADXL345_DATARATE_0_78_HZ = 0.78 hz
          - ADXL345_DATARATE_1_56_HZ = 1.56 hz
          - ADXL345_DATARATE_3_13_HZ = 3.13 hz
          - ADXL345_DATARATE_6_25HZ  = 6.25 hz
          - ADXL345_DATARATE_12_5_HZ = 12.5 hz
          - ADXL345_DATARATE_25_HZ   = 25 hz
          - ADXL345_DATARATE_50_HZ   = 50 hz
          - ADXL345_DATARATE_100_HZ  = 100 hz
          - ADXL345_DATARATE_200_HZ  = 200 hz
          - ADXL345_DATARATE_400_HZ  = 400 hz
          - ADXL345_DATARATE_800_HZ  = 800 hz
          - ADXL345_DATARATE_1600_HZ = 1600 hz
          - ADXL345_DATARATE_3200_HZ = 3200 hz
        """
        # Note: The LOW_POWER bits are currently ignored,
        # we always keep the device in 'normal' mode
        self._device.write8(ADXL345_REG_BW_RATE, rate & 0x0F)


    def get_data_rate(self):
        """Retrieve the current data rate.  See set_data_rate for the possible
        data rate constant values that will be returned.
        """
        return self._device.readU8(ADXL345_REG_BW_RATE) & 0x0F

    def convertAccnBits(self, x):
        """ Converts recieved signed 16bit from sensor and converts to float acceleration reading
        in m/s/s
        """
        return LSB_MULTIPLIER*x


    def read(self, gforce=True):
        """Read the current value of the accelerometer and return it as a tuple
        of float values in m/s/s or gs depending on status of gforce parameter.
        """
        raw = self._device.readList(ADXL345_REG_DATAX0, 6)
        data = struct.unpack('<hhh', raw)
        x = self.convertAccnBits(data[0])
        y = self.convertAccnBits(data[1])
        z = self.convertAccnBits(data[2])

        if not gforce:
            return x*EARTH_GRAVITY, y*EARTH_GRAVITY, z*EARTH_GRAVITY
        else:
            return x, y, z



if __name__ == "__main__":
    from time import sleep
    from math import sqrt
    # if run directly we'll just create an instance of the class and output 
    # the current readings
    adxl345 = ADXL345()
    sleep(0.01)

    adxl345.set_data_rate(ADXL345_DATARATE_200_HZ)
    sleep(0.01)

    adxl345.set_range(ADXL345_RANGE_16_G)
    sleep(0.01)

    while True:
        ax, ay, az = adxl345.read(gforce=True)

        mag = sqrt(ax**2 + ay**2 + az**2)
        if mag>1.1:
            print( "{},{},{}".format( ax, ay, az ) )


        sleep(.05)