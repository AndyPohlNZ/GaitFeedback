from adxl345 import ADXL345, ADXL345_DATARATE_200_HZ, ADXL345_RANGE_16_G, EARTH_GRAVITY
from l3g import L3G
import numpy as np
import time
from calibrate import loadCalibration, applyCalibration


if __name__ =="__main__":
    accn = ADXL345()
    time.sleep(0.01)


    gyro = L3G(1)
    time.sleep(0.01)

    accn.set_data_rate(ADXL345_DATARATE_200_HZ)
    time.sleep(0.01)

    accn.set_range(ADXL345_RANGE_16_G)
    time.sleep(0.01)

    gyro.setFullScale( 2000 )
    time.sleep(0.01)
    gyro.setDRBW( 200.0, 50.0 )
    time.sleep(0.01)

    calibrate = input("Do you want to use a calibration file? [y/n]")
    if calibrate =='y':
        cal_obj = loadCalibration('calibrationFiles/Calibration_20191128_test.cal')

    while True:
        ax, ay, az = accn.read()
        gx,gy,gz = gyro.read()
        if calibrate =='y':
            ax, ay, az = applyCalibration([ax,ay,az], cal_obj['accn'][0],cal_obj['accn'][1])
            gx, gy, gz = applyCalibration([gx,gy,gz], cal_obj['gyro'][0],cal_obj['gyro'][1])

        print("%.04f,%.04f,%.04f,%.04f,%.04f,%.04f"% (ax,ay,az,gx,gy,gz))
        time.sleep(0.1)
