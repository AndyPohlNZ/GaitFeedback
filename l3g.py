#!/usr/bin/python3

import smbus
from math import pi

WHO_AM_I       = 0x0F

CTRL_REG1      = 0x20
CTRL_REG2      = 0x21
CTRL_REG3      = 0x22
CTRL_REG4      = 0x23
CTRL_REG5      = 0x24
REFERENCE      = 0x25
OUT_TEMP       = 0x26
STATUS_REG     = 0x27

OUT_X_L        = 0x28
OUT_X_H        = 0x29
OUT_Y_L        = 0x2A
OUT_Y_H        = 0x2B
OUT_Z_L        = 0x2C
OUT_Z_H        = 0x2D

FIFO_CTRL_REG  = 0x2E
FIFO_SRC_REG   = 0x2F

INT1_CFG       = 0x30
INT1_SRC       = 0x31
INT1_THS_XH    = 0x32
INT1_THS_XL    = 0x33
INT1_THS_YH    = 0x34
INT1_THS_YL    = 0x35
INT1_THS_ZH    = 0x36
INT1_THS_ZL    = 0x37
INT1_DURATION  = 0x38



class L3G :
    drbw = {    (100.0,12.5 ) : (0b00, 0b00),
                (100.0, 25.0) : (0b00, 0b01),
                
                (200.0, 12.5) : (0b01, 0b00),
                (200.0, 25.0) : (0b01, 0b01),
                (200.0, 50.0) : (0b01, 0b10),
                (200.0, 70.0) : (0b01, 0b11),
                
                (400.0,  20.0) : (0b10, 0b00),
                (400.0,  25.0) : (0b10, 0b01),
                (400.0,  50.0) : (0b10, 0b10),
                (400.0, 110.0) : (0b10, 0b11),
                
                (800.0,  30.0) : (0b11, 0b00),
                (800.0,  35.0) : (0b11, 0b01),
                (800.0,  50.0) : (0b11, 0b10),
                (800.0, 110.0) : (0b11, 0b11) }
             
    def __init__( self, address = 0 ) :
        if address == 0 : self.address = 0x68
        else : self.address = 0x69
        bus = smbus.SMBus(1)   # always use bus 1 on Pi 2 and later
        devId = bus.read_byte_data( self.address, WHO_AM_I )
        if devId != 0xd3 :
            raise Exception( 'L3G4200D device not found at address 0x{:02x}'.format( self.address ) )
        print( "l3g4200d: devId = 0x{:02x}".format(devId) )
        bus.close()
        
        self.enableDefault()

    def enableDefault( self ) :
        bus = smbus.SMBus(1)

        # 0x80 = 0b10000000
        # BDU = 1 - enableblock data
        # BLE = 0 - LSB in low address
        # FS = 00 (+/- 250 dps full scale)
        # 0 - unused
        # ST = 00 - self test disabled
        # SIM = 0 - SPI4 wire
        bus.write_byte_data( self.address, CTRL_REG4, 0x80 )
        self.fs = 250

        # 0x6F = 0b01101111
        # DR = 01 (200 Hz ODR)
        # BW = 10 (50 Hz bandwidth)
        # PD = 1 (normal mode)
        # Zen = Yen = Xen = 1 (all axes enabled)
        bus.write_byte_data( self.address, CTRL_REG1, 0x6F )
        self.dataRate = 200.0
        self.bandwidth = 50.0
        
        bus.close()



    def convertGyroBytes( self, x0, x1 ) :
        x = x1 * 256 + x0
        if x > 32767 :
            x -= 65536
        x = self.fs * x / 32768
        return x

    def read( self, radians=True ) :
        bus = smbus.SMBus(1)
        data = bus.read_i2c_block_data( self.address, OUT_X_L | 0x80, 6 )
        bus.close()
        x0 = data[0]
        x1 = data[1]
        y0 = data[2]
        y1 = data[3]
        z0 = data[4]
        z1 = data[5]

        if radians:
            return self.convertGyroBytes( x0, x1 )*(pi/180), self.convertGyroBytes( y0, y1 )*(pi/180), self.convertGyroBytes( z0, z1 )*(pi/180)
        else:                  
            return self.convertGyroBytes( x0, x1 ), self.convertGyroBytes( y0, y1 ), self.convertGyroBytes( z0, z1 )

    
    
    def setFullScale( self, fs ) :
        if fs == 250 : bits = 0b00
        elif fs == 500 : bits = 0b01
        elif fs == 2000 : bits = 0b10
        else : return     # if fs does not have a support value, return having done nothing
        
        bus = smbus.SMBus(1)
        ctrl4 = bus.read_byte_data( self.address, CTRL_REG4 )
        bus.write_byte_data( self.address, CTRL_REG4, (ctrl4 & 0b11001111) | (bits << 4) )
        bus.close()
        
        self.fs = fs
        


    def setDRBW( self, dr, bw ) :
        if (dr,bw) not in L3G.drbw :
            raise Exception( 'Invalid datarate/bandwidth: {}/{}'.format( dr, bw ) )
        drBits, bwBits = L3G.drbw[ (dr, bw) ]
        self.dataRate = dr
        self.bandwidth = bw
        
        bus = smbus.SMBus(1)
        ctrl1 = bus.read_byte_data( self.address, CTRL_REG1 )
        bus.write_byte_data( self.address, CTRL_REG1, (ctrl1 & 0b00001111) | (drBits << 6) | (bwBits << 4) )
        bus.close()
        
        
    def getCtrl( self ) :
        """getCtrl( self ) -
            get the values of all the control registers
        """
        bus = smbus.SMBus(1)
        data = bus.read_i2c_block_data( self.address, CTRL_REG1 | 0x80, 5 )
        bus.close()
        
        return data[0], data[1], data[2], data[3], data[4]
        
        
        


if __name__ == "__main__" :
    import time
    import math
    import sys
    
    l3g = L3G(1)
    time.sleep(0.001)
    print( l3g.read() )
    ctrl1, ctrl2, ctrl3, ctrl4, ctrl5 = l3g.getCtrl()
    print( "control registers: 0b{:08b}, 0b{:08b}, 0b{:08b}, 0b{:08b}".format( ctrl1, ctrl2, ctrl3, ctrl4 ) )
    l3g.setFullScale( 500 )
    ctrl1, ctrl2, ctrl3, ctrl4, ctrl5 = l3g.getCtrl()
    print( "control registers: 0b{:08b}, 0b{:08b}, 0b{:08b}, 0b{:08b}".format( ctrl1, ctrl2, ctrl3, ctrl4 ) )
    time.sleep(0.1)
    print( l3g.read() )
    l3g.setFullScale( 2000 )
    time.sleep(0.001)
    print( l3g.read() )
    l3g.setFullScale( 500 )
    time.sleep(0.001)
    print( l3g.read() )
    print( l3g.read() )
    
    ctrl1, ctrl2, ctrl3, ctrl4, ctrl5 = l3g.getCtrl()
    print( "control registers: 0b{:08b}, 0b{:08b}, 0b{:08b}, 0b{:08b}".format( ctrl1, ctrl2, ctrl3, ctrl4 ) )
    l3g.setDRBW( 100, 12.5 )
    ctrl1, ctrl2, ctrl3, ctrl4, ctrl5 = l3g.getCtrl()
    print( "control registers: 0b{:08b}, 0b{:08b}, 0b{:08b}, 0b{:08b}".format( ctrl1, ctrl2, ctrl3, ctrl4 ) )
    print( "Full scale: ", l3g.fs )
    print( "Data rate: ", l3g.dataRate )
    print( "Bandwidth: ", l3g.bandwidth )
    
    while True :
        xRate, yRate, zRate = l3g.read(radians=True)
        mag = math.sqrt( xRate ** 2 + yRate ** 2 + zRate ** 2 )
        if mag > .5 :
            print( "{},{},{}".format( xRate, yRate, zRate ) )
        time.sleep( 0.01 )
    
    
#    for i in range(10) :
#        xRate, yRate, zRate = l3g.read()
#        print( "{},{},{}".format( xRate, yRate, zRate ) )
#        time.sleep( 0.01 )
    
    
    
    
    