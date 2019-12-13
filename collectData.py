#!/usr/bin/python3
from calibrate import loadCalibration, applyCalibration
from adxl345 import ADXL345, ADXL345_DATARATE_200_HZ, ADXL345_RANGE_16_G, EARTH_GRAVITY
from l3g import L3G
import time 
import math
from scipy.signal import butter, lfilter, lfilter_zi
import sys

DATADIR = '/home/pi/Desktop/project/dataCollectionFiles'

def butter_lowpass(cutoff, fs, order=2):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a
 

def collectData(accn, gyro, calbriationFile, sessionName):

    dt = 0.005 # 200Hz
    #a, b = computeFilterCoef( 1/dt, 20) # Lowpass filter at 20 Hz
    b, a = butter_lowpass (50, 1/dt, order=2)
    print("Butter Coefficients:")
    print("b = ")
    print(b)
    print("a = ")
    print(a)

    
    t0 = time.time()
    while time.time() -t0 <=3.01:
        print("Begining Recording in %.0f seconds"% (int(3-round(time.time() - t0))))
        time.sleep(1)

    saveFilename = DATADIR + '/Data_' + time.strftime("%Y%m%d") + "_" + sessionName + '.csv'


    # Initiilse filter
    zax = lfilter_zi(b, a)
    zay = lfilter_zi(b, a)
    zaz = lfilter_zi(b, a)

    zgx = lfilter_zi(b, a)
    zgy = lfilter_zi(b, a)
    zgz = lfilter_zi(b, a)    
    print("z = ")
    print(zax)
    print(zay)
    print(zaz)

    # collect data
    with open(saveFilename, 'w+') as saveFile:
        saveFile.write('timestamp,ax,ay,az,gx,gy,gz\n') # write header

        sample = 0
        try:
            while True:
                ax, ay, az = accn.read()
                gx,gy,gz = gyro.read()

                ax, ay, az = applyCalibration([ax,ay,az], cal_obj['accn'][0],cal_obj['accn'][1])
                gx, gy, gz = applyCalibration([gx,gy,gz], cal_obj['gyro'][0],cal_obj['gyro'][1])

                axf, zax = lfilter(b, a, [ax], zi = zax)
                ayf, zay = lfilter(b, a, [ay], zi = zay)
                azf, zaz = lfilter(b, a, [az], zi = zaz)

                gxf, zgx = lfilter(b, a, [gx], zi = zax)
                gyf, zgy = lfilter(b, a, [gy], zi = zay)
                gzf, zgz = lfilter(b, a, [gz], zi = zaz)

                saveFile.write('%0.8f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f\n' % (float(sample * dt), axf, ayf,azf, gxf,gyf,gzf))       
                time.sleep(dt/2 - .002) # adjusted to be approx 200Hz
                sample += 1
                sys.stdout.write("\rData Collected for %0.3f seconds!" % (float(sample*dt)))
                sys.stdout.flush()
        except KeyboardInterrupt:
            pass

    print("Data Colelcted for %.1f seconds!" %(time.time()-t0))
    saveFile.close()



if __name__ == '__main__':

    
    accn = ADXL345()
    time.sleep(0.01)


    gyro = L3G(1)
    time.sleep(0.01)

    accn.set_data_rate(ADXL345_DATARATE_200_HZ)
    time.sleep(0.01)

    accn.set_range(ADXL345_RANGE_16_G)
    time.sleep(0.01)


    cal_obj = loadCalibration("calibrationFiles/Calibration_20191128_test.cal")
   
    collectData(accn, gyro, cal_obj, 'KG5')