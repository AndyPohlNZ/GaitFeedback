#!/usr/bin/python3

"""
runFeedbackSystem Runs the feedback system in a testing state using pre saved data

Created By: Andrew Pohl
            Faculty of Kinesiology - University of Calgary
            December 2019
"""

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

#from explorerhat import motor


def z_norm(x, xbar, sd):
    """ Compute the Z norm of a variable given mean xbar and standard deviation sd"""
    return (x-xbar)/sd


def stanceTimeVariance(params, *args):
    """ Compute stance time variance for given parameter vector [alpha, beta]
    """
    alpha = params[0]
    beta = params[1]
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
    
    #Debug print statements
    #print("Toe OFF DEBUG")
    #print(toeOff)
    #print('INITIAL CONTACT debug')
    #print(initialContact)

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
        #Debug print statements
        # print("STANCE TIME DEBUG")
        # print(stanceTime)
        stanceVar = np.var(stanceTime)
        #Debug print statements
        # print("STANCE VAR DEBGU")
        # print(stanceVar)

        if stanceVar < 0.0001 or len(stanceTime)<30 or np.mean(stanceTime)<.12: stanceVar = 10000000 # hack to constrain to physiologic values.
    
    if returnRawStance:
        return stanceVar, stanceTime, initialContact
    else:
        return stanceVar


def runWarmUp(tmpFileLoc):
    dt = 0.005 # 200Hz
    
    # Get 2min of data for parameter optimisation
    print("Collecting 2min of data to optimise parameter selection.")
    with open(tmpFileLoc, 'r') as tmpFile:
        reader=csv.reader(tmpFile)
        rows=[r for r in reader]
    tmpFile.close()

    paramOpt =[]
    sample = 0
    while sample*dt < 45: # Note just 45sec for testing
        r = rows[sample+1]
        paramOpt.append([float(sample * dt), float(r[1]), float(r[2]), float(r[3]), float(r[4]), float(r[5]), float(r[6])])
        time.sleep(dt/10000) # adjusted to be approx 200Hz
        sample += 1
        sys.stdout.write("\rData Collected for %0.3f seconds!" % (float(sample*dt)))
        sys.stdout.flush()
    print()

    print("Running Optimisation")

    params = (slice(6, 9, .5), slice(1.8,3,0.2))
    opt_parms = optimize.brute(stanceTimeVariance, params, args = [paramOpt,dt,False, False], finish=None)
    print(opt_parms)

    opt_alpha = opt_parms[0]
    opt_beta = opt_parms[1]

    tstance = 0.5
    tff = 0.05
    print("Optimal alpha = %f"%(opt_alpha))

    stv, stanceTimes, initialContact = stanceTimeVariance([opt_alpha, opt_beta], paramOpt,dt,True, False)

    stance_time_mean = np.mean(stanceTimes)
    fsp_limit = 0.4*stance_time_mean

    fsp_limit_frames = np.floor(fsp_limit/dt)
    print("FSP_limit = %f fsp_limit_frames = %f"% (fsp_limit, fsp_limit_frames))
    # Compute FSP during warmup
    print()
    paramOpt = np.array(paramOpt)
    print(paramOpt)
    warmup_fsp = []
    for ic in initialContact:
        temp = abs(paramOpt[ic:int(ic+fsp_limit_frames), 5])
        print(temp)
        warmup_fsp.append(max(temp)) #CHECK THIS LINE!!!!


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


def activeMode(tmpFileLoc, opt_parms, fsp_parms, timing_parms, feedbackType):
    dt = 0.005 # 200Hz
    
    alpha = opt_parms[0]
    beta = opt_parms[1]
    tstance = opt_parms[2]
    tff = opt_parms[3]

    warmup_fsp_mean = fsp_parms[0]
    warmup_fsp_sd = fsp_parms[1]
    fsp_limit = fsp_parms[2]/dt

    stride_time_mean = timing_parms[0]
    stance_time_mean = timing_parms[1]
    flight_time_mean = timing_parms[2]


    if feedbackType not in ['vibration', 'sound', 'both']:
        raise Exception("Please provide a valid feedback type: vibration/sound/both")

    if feedbackType == 'vibration':
        useMotors = True
        useSound = False
    elif feedbackType == 'sound':
        useSound = True
        useMotors = False
    elif feedbackType =='both':
        useMotors = True
        useSound = True
    
    with open(tmpFileLoc, 'r') as tmpFile:
        reader=csv.reader(tmpFile)
        rows=[r for r in reader]
    tmpFile.close()

    paramOpt =[]
    sample = 0

    foot_state = 'unknown'
    ts=0
    fsp_tmp =[]
    fsp_z = 10000
    fsp = 10000

    # initiate OSC client
    oscClient = OscFeedbackClient()

    try:
        while sample*dt <30*60: # run for 30min
            
            # get data....
            r = rows[sample+1]
            ax = float(r[1])
            ay = float(r[2])
            az = float(r[3]) 
            gx = float(r[4]) 
            gy = float(r[5]) 
            gz = float(r[6])

            ares = math.sqrt(ax**2 + ay**2 + az**2)
            # Foot state machine begins

            # STATE 1 FOOT CONTACT FSP DETERMINATION
            if (ares > alpha )& (float(ts) > tstance) & (foot_state != 'Contact'):
                print("Footstrike!!")
                foot_state = 'Contact - FSP zone' # state machine - foot in contact with ground
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
                fsp_tmp.append(abs(gy))

            # STATE 2 FOOT CONTACT NON FSP
            elif (foot_state == 'Contact - FSP zone') & (len(fsp_tmp)>fsp_limit):
                foot_state = 'Contact'

            # STATE 3 EARLY FLIGHT
            if (foot_state =='Contact' or foot_state == 'Contact - FSP zone' )&( float(ts) > tff )&( ares > beta):
                foot_state = 'Flight'
                t_flight = 0
                if fsp is None:
                    fsp = np.max(fsp_tmp)# compute FSP for that step
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


            time.sleep(dt) # adjusted to be approx 200Hz
            sample += 1
            ts +=dt
            print("Time: %.4f, Ts = %.4f, ares = %.4f, foot_state = %s, fsp_z= %.4f"%(sample*dt, ts, ares, foot_state, fsp_z))

            #sys.stdout.write("\Active Mode for %0.3f seconds!" % (float(sample*dt)))
            #sys.stdout.flush()
    except KeyboardInterrupt:
        print("Quitting Feedback system")
        oscClient.close()
        motor.one.speed(0)
        motor.two.speed(0)




if __name__ =="__main__":
    # Location of tempory data file
    tmpFileLoc = 'testData/TestData.csv'

    opt_parms, fsp_parms, timing_parms = runWarmUp(tmpFileLoc)
    time.sleep(5)
    activeMode(tmpFileLoc, opt_parms, fsp_parms, timing_parms, 'sound')
