from calibrate import loadCalibration, applyCalibration
from adxl345 import ADXL345, ADXL345_DATARATE_200_HZ, ADXL345_RANGE_16_G, EARTH_GRAVITY
from oscFeedbackClient import  OscFeedbackClient
from l3g import L3G
import time 
import math
from scipy.signal import butter, lfilter, lfilter_zi
from scipy import optimize
import sys
import csv
import numpy as np

from explorerhat import motor


def z_norm(x, xbar, sd):
    return (x-xbar)/sd

def butter_lowpass(cutoff, fs, order=2):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a




def stanceTimeVariance(params, *args):
    alpha = params[0]
    beta = 2.5
    tff =  0.1

    data = args[0]
    dt = args[1]
    returnRawStance= args[2]
    debug = args[3]
    # Foot Strike Detection.
    tstance = 0.5
    initialContact = []
    ts = 0

    # TO detection
    tf = 0
    #tff = 0.1
    toeOff = []

    for i, x in enumerate(data):
        res = math.sqrt(x[1]**2 +x[2]**2+x[3]**2)
        if debug: print("Res = %.4f ts = %.4f tf = %.4f" %(res, ts,tf))
        if(res > alpha) and ts > tstance:
            if debug: print('Footstrike!!')
            initialContact.append(i)
            ts = 0
            tf = 0
        elif (res>beta) & (tf>tff) & (tf < 100):
            if debug: print("Toeoff!")
            toeOff.append(i)
            tf = 100 # 
        
        else:
            ts+= dt
            tf +=dt

    # Correct start and end of each vector.
    print("Toe OFF DEBUG")
    print(toeOff)
    print('INITIAL CONTACT debug')
    print(initialContact)

    if len(initialContact)<30:
        stanceVar = 10000000
    else:
        while toeOff[0]<initialContact[0]:
            del(toeOff[0])
        
        while initialContact[-1]>toeOff[-1]:
            del(initialContact[-1])

        
        initialContact = np.array(initialContact)
        toeOff = np.array(toeOff)

        stanceTime = (toeOff - initialContact)*dt
        print("STANCE TIME DEBUG")
        print(stanceTime)
        stanceVar = np.var(stanceTime)
        print("STANCE VAR DEBGU")
        print(stanceVar)

        if stanceVar < 0.0001 or len(stanceTime)<30 or np.mean(stanceTime)<.12: stanceVar = 10000000 # hack to constrain to physiologic values.
    
    if returnRawStance:
        return stanceVar, stanceTime, initialContact
    else:
        return stanceVar

#def computeFSP(gyro, )
def runWarmUp(accn, gyro, calbriationFile):
    #tmpFileLoc = '/home/pi/Desktop/project/dataCollectionFiles/Data_20191119_testing_FFS.csv'
    cal_obj = calbriationFile

    dt = 0.005 # 200Hz
    b, a = butter_lowpass (30, 1/dt, order=2)

    # # Initiilse filter
    zax = lfilter_zi(b, a)
    zay = lfilter_zi(b, a)
    zaz = lfilter_zi(b, a)

    zgx = lfilter_zi(b, a)
    zgy = lfilter_zi(b, a)
    zgz = lfilter_zi(b, a)    

    # Get 2min of data for parameter optimisation
    print("Collecting 2min of data to optimise parameter selection.")

    # with open(tmpFileLoc, 'r') as tmpFile:
    #     reader=csv.reader(tmpFile)
    #     rows=[r for r in reader]
    # tmpFile.close()

    paramOpt =[]
    sample = 0
    while sample*dt <60:
        t0 = time.time()
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

        paramOpt.append([float(sample * dt), axf, ayf, azf, gxf, gyf, gzf])
        time.sleep(dt/5) # adjusted to be approx 200Hz
        sample += 1
        sys.stdout.write("\r Loop Time %.4f - Data Collected for %0.3f seconds!\n" % (time.time() - t0, float(sample*dt)))
        sys.stdout.flush()
    print()


  
    print("Running Optimisation")
    alpha_vals = np.linspace(7,9,20)
    
    stv_res = [0]*len(alpha_vals)
    for i, a in enumerate(alpha_vals):
        print("A = %.2f"%(a))
        stv_res[i] = stanceTimeVariance([a], paramOpt,dt,False, False)
    
    print(stv_res)

    opt_alpha = alpha_vals[stv_res == min(stv_res)]
    opt_beta = 2.5
    tstance = 0.5
    tff = 0.1
    if len(opt_alpha)>1:
        opt_alpha = opt_alpha[0]

    print("Optimal alpha = %.4f"%(opt_alpha))

    stv, stanceTimes, initialContact = stanceTimeVariance([opt_alpha], paramOpt,dt,True, False)
    #print(stanceTimes)

    stance_time_mean = np.mean(stanceTimes)
    fsp_limit = 0.33*stance_time_mean

    fsp_limit_frames = np.floor(fsp_limit/dt)
    print("FSP_limit = %f fsp_limit_frames = %f"% (fsp_limit, fsp_limit_frames))
    # Compute FSP during warmup
    print()
    paramOpt = np.array(paramOpt)
    warmup_fsp = []
    for ic in initialContact:
        temp = paramOpt[ic:int(ic+fsp_limit_frames), 5]
        warmup_fsp.append(max(temp))


    warmup_fsp_mean = np.mean(warmup_fsp)
    warmup_fsp_sd = np.sqrt(np.var(warmup_fsp))

    stride_time = [(t - s)*dt for s, t in zip(initialContact, initialContact[1:])]
    stride_time_mean = np.mean(stride_time)
    stride_time_sd = np.sqrt(np.var(stride_time))
    

    if len(stanceTimes)>len(stride_time):
        stanceTimes = stanceTimes[0:len(stride_time)]

    flight_time = stride_time - stanceTimes
    flight_time_mean = np.mean(flight_time)
    
    print("--------------------- Warmup Concluded ---------------------------------")
    print("Optimal Parameters: alpha = %.4f, beta = %.4f, tstance = %.4f, tff = %.4f" % (opt_alpha, opt_beta, tstance, tff))
    print("Warmup FSP: Mean = %.4f[rads], SD = %.4f [rads]. FSP_limit %.4f [s]" % (warmup_fsp_mean, warmup_fsp_sd, fsp_limit))
    print("Stride time mean = %.4f[s], Stance time mean = %.4f[s] Flight time mean = %.4f[s]"% (stride_time_mean, stance_time_mean, flight_time_mean))
    print("-------------------------------------------------------------------------")
    return (opt_alpha, opt_beta, 0.5, 0.05), (warmup_fsp_mean, warmup_fsp_sd, fsp_limit), (stride_time_mean, stance_time_mean, flight_time_mean)


def activeMode(accn, gyro, calibrationFile, opt_parms, fsp_parms, timing_parms, feedbackType):
    #tmpFileLoc = '/home/pi/Desktop/project/dataCollectionFiles/Data_20191119_testing_FFS.csv'
    dt = 0.005 # 200Hz
    
    cal_obj = calibrationFile

    alpha = opt_parms[0]
    beta = opt_parms[1]
    tstance = opt_parms[2]
    tff = opt_parms[3]

    warmup_fsp_mean = fsp_parms[0]
    warmup_fsp_sd = fsp_parms[1]
    fsp_limit = np.floor(fsp_parms[2]/dt)

    stride_time_mean = timing_parms[0]
    stance_time_mean = timing_parms[1]
    flight_time_mean = timing_parms[2]


    if feedbackType not in ['vibration', 'sound', 'both']:
        raise Exception("Please provide a valid feedback type: vibration/sound/both")

    if feedbackType == 'vibration':
        useMotors = True
    elif feedbackType == 'sound':
        useSound = True
    elif feedbackType =='both':
        useMotors = True
        useSound = True

    b, a = butter_lowpass (50, 1/dt, order=2)

    
    # Initiilse filter
    zax = lfilter_zi(b, a)
    zay = lfilter_zi(b, a)
    zaz = lfilter_zi(b, a)

    zgx = lfilter_zi(b, a)
    zgy = lfilter_zi(b, a)
    zgz = lfilter_zi(b, a) 

    paramOpt =[]
    sample = 0

    foot_state = 'unknown'
    ts=0
    fsp_tmp =[]
    fsp_z = 10000
    fsp = 10000

    # initiate OSC client
    oscClient = OscFeedbackClient()

    missed_footstrikes = 0
    try:
        while True:
            t0 = time.time()
            ax, ay, az = accn.read()
            gx,gy,gz = gyro.read()

            ax, ay, az = applyCalibration([ax,ay,az], cal_obj['accn'][0],cal_obj['accn'][1])
            gx, gy, gz = applyCalibration([gx,gy,gz], cal_obj['gyro'][0],cal_obj['gyro'][1])

            ax, zax = lfilter(b, a, [ax], zi = zax)
            ay, zay = lfilter(b, a, [ay], zi = zay)
            az, zaz = lfilter(b, a, [az], zi = zaz)

            gx, zgx = lfilter(b, a, [gx], zi = zax)
            gy, zgy = lfilter(b, a, [gy], zi = zay)
            gz, zgz = lfilter(b, a, [gz], zi = zaz)

            # get data....
            ares = math.sqrt(ax**2 + ay**2 + az**2)
            # Foot state machine begins

            # STATE 1 FOOT CONTACT FSP DETERMINATION
            if (ares > alpha )& (float(ts) > tstance*.6) & (foot_state != 'Contact'):
                print("Footstrike!!")
                foot_state = 'Contact - FSP zone' # state machine - foot in contact with ground

                if ts>1:
                    missed_footstrikes +=1
                    if missed_footstrikes >5:
                        beta = alpha-0.1
                        missed_footstrikes = 0

                ts = 0
                fsp_tmp = []
                fsp = None

                # stop motors 
                if useMotors:
                    motor.one.speed(0)
                    motor.two.speed(0)
                    startMotor = True

                # stop sound
                if useSound:
                    oscClient.setVolume(0)
                    oscClient.playTone()
                


            if (foot_state == 'Contact - FSP zone') & (len(fsp_tmp)<fsp_limit):
                #print("Foot Contact")
                fsp_tmp.append(gy)

            # STATE 2 FOOT CONTACT NON FSP
            elif (foot_state == 'Contact - FSP zone') & (len(fsp_tmp)>fsp_limit):
                foot_state = 'Contact'

            # STATE 3 EARLY FLIGHT
            if (foot_state =='Contact' or foot_state == 'Contact - FSP zone' )&( float(ts) > tff )&( ares > beta):
                foot_state = 'Flight'
                t_flight = 0
                if fsp is None:
                    fsp = np.max(fsp_tmp) # compute FSP for that step
                    # compute feedback intensity....
                    fsp_z = z_norm(fsp, warmup_fsp_mean, warmup_fsp_sd)
                    print("Toe off!!")
            
            # STATE 4 LATE FLIGHT PROVIDING FEEDBACK
            if (foot_state =='Flight' or foot_state == 'Flight - Feedback') & (ts> stance_time_mean + (flight_time_mean/4)):
                
                foot_state ='Flight - Feedback'
                # starting motor
                if useMotors:
                    if (fsp_z >0) & (startMotor):
                        motorSpeed = fsp_z * (50/3)
                        motorSpeed = max(min(float(motorSpeed), 50), -50)
                        print("Starting motor at speed %.2f"% (motorSpeed))
                        motor.one.speed(motorSpeed)
                        motor.two.speed(motorSpeed)
                        startMotor = False

                # play tones
                if useSound:
                    volume = min(abs(fsp_z)/3,1) # = number between 0 and 1 
                    #print("Playing tone with volumne %.2f"%volume)
                    oscClient.setVolume(volume) 
                    # if (fsp_z<0): # i.e. a good stride
                    #     oscClient.setNotes([60,64,67]) # cmag
                    #     oscClient.playTone()
                    if(fsp_z>0): # poor stride
                        oscClient.setNotes([69,72,76]) # Am chord.
                        oscClient.playTone()


            #time.sleep(dt/2 - .004) # adjusted to be approx 200Hz
            sample += 1
            ts +=dt
            sys.stdout.write("\r Loop Time %.4f Time: %.4f, Ts = %.4f, ares = %.4f, foot_state = %s, fsp_z= %.4f\n"%(time.time() - t0, sample*dt, ts, ares, foot_state, fsp_z))
            sys.stdout.flush()

            #sys.stdout.write("\Active Mode for %0.3f seconds!" % (float(sample*dt)))
            #sys.stdout.flush()
    except KeyboardInterrupt:
        print("Quitting Feedback system")
        oscClient.close()
        motor.one.speed(0)
        motor.two.speed(0)




if __name__ =="__main__":

    # set up accerometer
    accn = ADXL345()
    time.sleep(0.01)

    # set up gyro
    gyro = L3G(1)
    time.sleep(0.01)

    accn.set_data_rate(ADXL345_DATARATE_200_HZ)
    time.sleep(0.01)

    accn.set_range(ADXL345_RANGE_16_G)
    time.sleep(0.01)

    # Load Calibration - set up to that days calibration file.
    cal_obj = loadCalibration("/home/pi/Desktop/project/calibrationFiles/Calibration_20191128_test.cal") 


    print("Have participant start running....")
    for i in range(15):
        sys.stdout.write("\rCollecting data in %.0f seconds!" % (15-i))
        sys.stdout.flush()
        time.sleep(1)

    opt_parms, fsp_parms, timing_parms = runWarmUp(accn, gyro, cal_obj)
       
    
    time.sleep(30)
    activeMode(accn, gyro, cal_obj, opt_parms, fsp_parms, timing_parms, 'both')
