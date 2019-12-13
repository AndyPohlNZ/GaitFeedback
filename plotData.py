import matplotlib.pyplot as plt
import numpy as np
import csv

def loadData(filename):
    '''Loads IMU data from CSV'''
    data = []
    with open(filename, 'r') as file:
        csvreader = list(csv.reader(file, delimiter =','))

        line_count = 0
        for row in csvreader:
            if line_count ==0:
                header = row
                line_count +=1
            else:
                float_row = []
                for col in row:
                    float_row.append(float(col))
                data.append(float_row)
                line_count +=1

    file.close()
    data = np.array(data)
    return data, header


def plotIMU(data, time):
    '''Plots IMU for a given time range'''
    timestamp = data[:,0]
    timemin = time[0]
    timemax = time[1]
    lidx = (timestamp>=timemin) & (timestamp<=timemax)
    timestamp = timestamp[lidx]
    ax = data[lidx,1]
    ay = data[lidx,2]
    az = data[lidx,3]
    res = np.sqrt(ax**2 + ay**2 + az**2)
    
    gx = data[lidx,4]
    gy = data[lidx,5]
    gz = data[lidx,6]

    fig = plt.figure()
    plt.subplot(1,1,1)
    plt.plot(timestamp,ax, alpha = 0.4, linewidth = 0.5)
    plt.plot(timestamp,ay, alpha = 0.4, linewidth = 0.5)
    plt.plot(timestamp,az, alpha = 0.4, linewidth = 0.5)
    plt.plot(timestamp, res)
    #plt.legend(['x', 'y', 'z', 'res', 'x', 'y', 'z'])
    plt.xlabel('Time [s]')
    plt.ylabel('Acceleration [g]')
    plt.ylim([-16.5, 16.5])
    
    #plt.subplot(2,1,2)
    plt.plot(timestamp,gx, alpha = 0.8, linewidth = 0.5)
    plt.plot(timestamp,gy, alpha = 0.8, linewidth = 0.5)
    plt.plot(timestamp,gz, alpha = 0.8, linewidth = 0.5)
    plt.legend(['x', 'y', 'z', 'res', 'x', 'y', 'z'])

    #plt.legend(['x', 'y', 'z'])
    #plt.xlabel('Time [s]')
    #plt.ylabel('Gyro [g]')
    #plt.ylim([-6, 6])



    plt.show()

if __name__ == "__main__":
    data, header = loadData('dataCollectionFiles/Data_20191128_KG2.csv')
    plotIMU(data, (15,20))