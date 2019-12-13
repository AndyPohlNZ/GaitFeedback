#!/usr/bin/python3

from adxl345 import ADXL345, ADXL345_DATARATE_200_HZ, ADXL345_RANGE_16_G, EARTH_GRAVITY
from l3g import L3G
import numpy as np
import time
from math import pi
import pickle

CALIBRATION_DIR = "calibrationFiles"

def calibrateAccn(accn):
    print("Calibrating Accelerometer")
    positions = ['+ve x up', '-ve x up', '+ve y up', '-ve y up', '+ve z up', '-ve z up']

    pos_counter = 0

    w = []
    y =[]
    for pos in positions:
        input("Position IMU %s and press [ENTER]" % pos)
        # take mean of 2 seconds
        nsamp = 0
        axtmp = []
        aytmp = []
        aztmp = []
        while nsamp <600:
            ax, ay, az = accn.read()
            axtmp.append(ax)
            aytmp.append(ay)
            aztmp.append(az)
            time.sleep(0.005)
            nsamp +=1
        ax = np.mean(ax)
        ay = np.mean(ay)
        az = np.mean(az)

        w.append([ax,ay,az,1])
        if '+ve' in pos:
            temp = [0,0,0]
            temp[pos_counter] = -1
            y.append(temp)
        elif '-ve' in pos:
            temp = [0,0,0]
            temp[pos_counter] = 1
            y.append(temp)
            pos_counter +=1
        print(temp)


    w = np.array(w)
    y = np.array(y)
    
    x = np.dot(np.dot(np.linalg.inv(np.dot(np.transpose(w), w)), np.transpose(w)),y)

    bias = x[3,:]
    sensitivity = x[0:3,:]

    print("Bias = ")
    print(bias)
    print("Sensitivity = ")
    print(sensitivity)
    time.sleep(5)

    return bias, sensitivity


def calibrateGyro(gyro):
    print("Calibrating Gyroscope")
    input("Keep Gyro stationary and press [ENTER]")

    # Step 1 compute offset
    offset_data =[]
    t0 = time.time()
    while (time.time()-t0) <3: # collect 3 sec of data
        gx,gy,gz = gyro.read()
        offset_data.append([gx,gy,gz])
        time.sleep(0.05)

    offset_data = np.array(offset_data)
    bias = np.mean(offset_data, axis=0)
    print(bias)

    # Step2 compute sensitivty

    Bg = np.zeros((3,3))
    Bg[0,:] = [bias[0], bias[0], bias[0]]
    Bg[1,:] = [bias[1], bias[1], bias[1]]
    Bg[2,:] = [bias[2], bias[2], bias[2]]

    positions = ['+ve x up', '+ve y up', '+ve z up']

    dt = 0.05 # 200Hz
    pos_counter = 0
    Wg = np.zeros((3,3))
    for pos in positions:
        input("Orientate gyro %s.  Press [ENTER] and complete a full 360deg rotation.  When complete hit [Ctrl-C]" % pos)
        while True:
            try:
                gx, gy, gz = gyro.read()
                Wg[pos_counter,:] += ([gx,gy,gz] - bias[pos_counter])*dt
                sleep(dt)

            except KeyboardInterrupt:
                break

        pos_counter += 1
    print(Wg)

    #k1=np.dot(Wg, 1/(2*pi))
    #k2 = np.dot(Wg, 1/(2*pi))

    k2 = np.diag(np.dot(np.dot(Wg, 1/(2*pi)), np.transpose(np.dot(Wg, 1/(2*pi)))))
    K = np.identity(3) * np.sqrt(k2)
    print("K = ")
    print(K)

    Rg = np.dot(np.dot(np.linalg.inv(K), Wg), 1/(2*pi))
    print("Rg = ")
    print(Rg)

    bias = -1*bias
    sensitivity = np.dot(np.linalg.inv(Rg), np.linalg.inv(K))

    print("Bias = ")
    print(bias)
    print("Sensitivity = ")
    print(sensitivity)
    return bias, sensitivity


def applyCalibration(data, bias, sensitivity):
    cal = np.matmul(sensitivity, data) + bias
    return cal[0], cal[1], cal[2]

def calibratieIMU(accn, gyro):
    accn.set_data_rate(ADXL345_DATARATE_200_HZ)
    sleep(0.01)

    accn.set_range(ADXL345_RANGE_16_G)
    sleep(0.01)

    gyro.setFullScale( 2000 )
    sleep(0.01)
    gyro.setDRBW( 200.0, 50.0 )
    sleep(0.01)

    abias, asens = calibrateAccn(accn)
    gbias, gsens = calibrateGyro(gyro)
    return {"accn":[abias, asens], "gyro":[gbias, gsens]}


def saveCalibration(calObj, sessionName):
    filename = CALIBRATION_DIR + "/Calibration_" + time.strftime("%Y%m%d") + "_" + sessionName +".cal"

    with open(filename, 'wb') as file:
        pickle.dump(calObj, file)
    file.close()

def loadCalibration(filename):
    with open(filename, 'rb') as file:
            cal_obj = pickle.load(file)
    return cal_obj
    


if __name__ == '__main__':
    from time import sleep
    
    accn = ADXL345()
    sleep(0.01)


    gyro = L3G(1)
    sleep(0.01)

    accn.set_data_rate(ADXL345_DATARATE_200_HZ)
    sleep(0.01)

    accn.set_range(ADXL345_RANGE_16_G)
    sleep(0.01)


    cal_obj = calibratieIMU(accn,gyro)

    saveCalibration(cal_obj, 'test')
    

    while True:
        ax, ay, az = accn.read()
        gx,gy,gz = gyro.read()

        ax, ay, az = applyCalibration([ax,ay,az], cal_obj['accn'][0],cal_obj['accn'][1])
        gx, gy, gz = applyCalibration([gx,gy,gz], cal_obj['gyro'][0],cal_obj['gyro'][1])

        print("%.04f,%.04f,%.04f,%.04f,%.04f,%.04f"% (ax,ay,az,gx,gy,gz))
        sleep(0.1)


