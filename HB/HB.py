
# GLEAM HB.py 
# Rev A
# 3/13/2022
# 
# Signed: M. Pelissero, S. Shaw
#
#
# A python 3 module for importing and visualizing data obtained through an SSH connection to the system EV Linux system
# 
# Rev Tracking:
# Rev A --- first release. Imports data from prox, IR, Lidar, Lidar scanning, and IMU sensor csv files and visualizes them into graphs/plots. 
#            Saves graphs/plots as images for use in a mission webpage dashboard - S. Shaw, M. Pelissero
#



import pandas as pd
from subprocess import Popen
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
plt.style.use('default')

itCount = 0
mapCount = 0
irCount = 0
lidScanInd = 0
lastScanInd = 0
newScanInd = 0
lastIrInd = 0


        # ------ Initialize Graphs -------------------------------------------------------------------------------------------------------------------------------


p = Popen('python autoSync.py')

while (True):
    try:       

        framSiz = 20
        framNum = 5 - itCount
        
        
        # ------ IMU Visualize -------------------------------------------------------------------------------------------------------------------------------

        imu_headers = ['time', 'xang', 'yang', 'mag']
        imu = pd.read_csv (r'data\imu.csv', names= imu_headers)
        imu_mag = imu['mag']

        mag_x = [0] * len(imu)
        mag_y = [0] * len(imu)
        mag_z = [0] * len(imu)
        for i in range (0,len(imu)):
            imuDatArr = imu_mag[i].split(';')
            mag_x[i] = float(imuDatArr[0])
            mag_y[i] = float(imuDatArr[1])
            mag_z[i] = float(imuDatArr[2])

        frame_mag_x = [0] * framSiz
        frame_mag_y = [0] * framSiz
        frame_mag_z = [0] * framSiz
        try:
            for i in range (0, framSiz):
                frame_mag_x[i] = mag_x[len(mag_x) - framSiz - framNum + i]
                frame_mag_y[i] = mag_y[len(mag_y) - framSiz - framNum + i]
                frame_mag_z[i] = mag_z[len(mag_z) - framSiz - framNum + i]
        except:
            for i in range (0, len(imu)):
                frame_mag_x[i] = mag_x[i]
                frame_mag_y[i] = mag_y[i]
                frame_mag_z[i] = mag_z[i]
        
        fig2, ax2 = plt.subplots()
        ax2.plot(frame_mag_x)
        ax2.plot(frame_mag_y)
        ax2.plot(frame_mag_z)
        ax2.figure.savefig('Graph_Outputs\mag.png')
        plt.close(fig2)


        # ------ Prox Visualize -------------------------------------------------------------------------------------------------------------------------------
        
        prox_headers = ['time', 'prox']
        prox = pd.read_csv ('data\proximity.csv', names= prox_headers)
        frame_prox = [0] * framSiz
        try:
            for i in range (0, framSiz):
                frame_prox[i] = prox['prox'][len(prox) - framSiz - framNum + i]
        except:
            for i in range (0, len(prox)):
                frame_prox[i] = prox['prox'][i]
        fig3, ax3 = plt.subplots()
        ax3.plot(frame_prox)
        ax3.figure.savefig('Graph_Outputs\prox.png')
        plt.close(fig3)

        
        
        # ------ Lidar Scan Visualize -------------------------------------------------------------------------------------------------------------------------
        
        
        lidarscan_headers = ['Distance,Strength,PWM']
        lidscan = pd.read_csv ('data\lidarScan.csv', names= lidarscan_headers)
        indscan=0
        
        j = 0
        #k = 0
        #print(str(lidscan['Distance,Strength,PWM'][k]))
        while(j == 0):
            k = len(lidscan) - i
            if (str(lidscan['Distance,Strength,PWM'][k]) == 'new'):
                j = 1
                lidScanInd = k
            i += 1
        
        newScanInd = k #for checking if there is a new scan
        
        for i in range (lidScanInd + 1,len(lidscan)):
            #print(i)
            lidscanArr = lidscan['Distance,Strength,PWM'][i].split(',')
            if ((int(lidscanArr[1])>200) and (int(lidscanArr[0])<610)):
                indscan+=1
                #print(lidscanArr)
                
        DistanceScan=[0]*indscan
        StrengthScan=[0]*indscan
        PWM=[0]*indscan
        #filtering bad data
        k = 0
        for i in range (lidScanInd + 1, len(lidscan)):
            lidscanArr = lidscan['Distance,Strength,PWM'][i].split(',')
            if ((int(lidscanArr[1])>200) and (int(lidscanArr[0])<610)):
                DistanceScan[k]=int(lidscanArr[0])
                StrengthScan[k]=int(lidscanArr[1])
                PWM[k]=float(lidscanArr[2])
                k += 1
        
        #Calculate x,y
        x=[0]*indscan
        y=[0]*indscan
        for i in range (0,indscan):
            Degree=(PWM[i]-2.0)*(180.0/(11-2))
            #print(Degree)
            Dist=(DistanceScan[i]/100)
            x[i]=Dist*np.cos(np.radians(Degree))
            y[i]=Dist*np.sin(np.radians(Degree))  
        
        
        
        fig4, ax4 = plt.subplots()
        ax4.scatter(x,y)
        #ax4.xlabel('X (m)')
        #ax4.ylabel('Y (m)')
        if (lastScanInd != newScanInd):
            str1 = 'Graph_Outputs\lidarScans\irScan'
            str2 = '.png'
            lidarScanPath = str1 + str(mapCount) + str2
            ax4.figure.savefig(lidarScanPath)
            #ax4.title('Lidar Scan Visualization')
            lastScanInd = newScanInd
        
        plt.close(fig4)
        
        
        # ------ IR Thermal Cam Visualize ------------------------------------------------------------------------------------------------------------------

        ir_headers = ['Time', 'Array']
        ir = pd.read_csv ('data\ir.csv', names= ir_headers)
        
        if (len(ir) != lastIrInd):
            lastIrInd = len(ir)
            ir_1 = ir["Array"][len(ir["Array"]) - 1] # take last reading available
            irDatArr = ir_1.split()

            ir_frame = [0] * 768
            for i in range (0, 768):
                ir_frame[i] = irDatArr[i]
                ir_frame[i] = ir_frame[i].replace("[","")
                ir_frame[i] = ir_frame[i].replace("]","")
                ir_frame[i] = float(ir_frame[i])
                
                frame = ir_frame
            
            """
            mlx_shape = (24,32)
            #plt.ion() # enables interactive plotting
            #figir,axir = plt.subplots(figsize=(12,7))
            therm1 = ax4.matshow(np.zeros(mlx_shape),vmin=0,vmax=60) #start plot with zeros
            cbar = figir.colorbar(therm1) # setup colorbar for temps
            cbar.set_label('Temperature [$^{\circ}$C]',fontsize=14) # colorbar label
            t_array = []
            
            t1 = time.monotonic()
            try:
                data_array = (np.reshape(frame,mlx_shape)) # reshape to 24x32
                therm1.set_data(np.fliplr(data_array)) # flip left to right
                therm1.set_clim(vmin=np.min(data_array),vmax=np.max(data_array)) # set bounds
                #cbar.on_mappable_changed(therm1) # update colorbar range
                plt.pause(0.001) # required
                figir.savefig('Graph_Outputs\ir.png',dpi=300,facecolor='#FCFCFC',bbox_inches='tight') # comment out to speed up
                t_array.append(time.monotonic()-t1)
                #print('Sample Rate: {0:2.1f}fps'.format(len(t_array)/np.sum(t_array)))
            except ValueError:
                continue # if error, just read again
            
            #plt.close('all')
            """
        
     
        # ------ Lidar Visualize -------------------------------------------------------------------------------------------------------------------------------
        
        lidar_headers = ['time', 'Distance/strength']
        liddat= pd.read_csv (r'data\lidar.csv', names= lidar_headers)
        ind=0
                
                
                
        for i in range (0,len(liddat)):
            lidDatArr = liddat['Distance/strength'][i].split(';')
            if ((int(lidDatArr[1])>200) and (int(lidDatArr[0])<610)):
                ind+=1
                
        Distance=[0]*ind
        Strength=[0]*ind
        k = 0
        for i in range (0,len(liddat)):
            lidDatArr = liddat['Distance/strength'][i].split(';')
            if ((int(lidDatArr[1])>200) and (int(lidDatArr[0])<610)):
                Distance[k]=int(lidDatArr[0])
                Strength[k]=int(lidDatArr[1])
                k += 1


        # ------ Plot Everything -------------------------------------------------------------------------------------------------------------------------------

        
        fig7 = plt.figure()
        mngr = plt.get_current_fig_manager()
        mngr.window.geometry("1900x1000+0+0")

        plt.rcParams["figure.autolayout"] = True
        ax1 = plt.subplot2grid((3, 3), (0, 0), colspan=1)
        ax2 = plt.subplot2grid((3, 3), (1, 0), rowspan=1)
        ax3 = plt.subplot2grid((3, 3), (0, 1), rowspan=1)
        ax4 = plt.subplot2grid((3, 3), (1, 1), rowspan=1)
        ax1.set_title("Proximity")
        ax2.set_title("magnetic readings")
        ax3.set_title("last mapping scan")
        ax4.set_title("last thermal scan")
        
        ax1.plot(frame_prox, marker='o', linestyle='')
        ax2.plot(frame_mag_x, 'tab:green', marker='o', linestyle='-')
        ax2.plot(frame_mag_y, 'tab:blue', marker='o', linestyle='-')
        ax2.plot(frame_mag_z, 'tab:red', marker='o', linestyle='-')
        ax3.plot(x, y, 'tab:red', marker='o', linestyle='')
        
        mlx_shape = (24,32)
        #plt.ion() # enables interactive plotting
        #figir,axir = plt.subplots(figsize=(12,7))
        therm1 = ax4.matshow(np.zeros(mlx_shape),vmin=0,vmax=60) #start plot with zeros
        cbar = fig7.colorbar(therm1) # setup colorbar for temps
        cbar.set_label('Temperature [$^{\circ}$C]',fontsize=14) # colorbar label
        t_array = []
        
        t1 = time.monotonic()
        try:
            data_array = (np.reshape(frame,mlx_shape)) # reshape to 24x32
            therm1.set_data(np.fliplr(data_array)) # flip left to right
            therm1.set_clim(vmin=np.min(data_array),vmax=np.max(data_array)) # set bounds
            #cbar.on_mappable_changed(therm1) # update colorbar range
            plt.pause(0.001) # required
            #figir.savefig('Graph_Outputs\ir.png',dpi=300,facecolor='#FCFCFC',bbox_inches='tight') # comment out to speed up
            t_array.append(time.monotonic()-t1)
            #print('Sample Rate: {0:2.1f}fps'.format(len(t_array)/np.sum(t_array)))
        except ValueError:
            continue # if error, just read again
      
        
        #print("hi again")
        itCount += 1
        mapCount += 1
        if (itCount == 5):
            itCount = 0
        #time.sleep(0.7)
        #fig6.set_size_inches(6, 3, forward = True)
        
        #plt.rcParams["figure.figsize"] = (10, 5)

        #print(mngr.window.geometry())
        #manager = plt.get_current_fig_manager()
        #manager.full_screen_toggle()
        
        fig7.savefig('Graph_Outputs\ir.png')
        
        plt.show(block = False)
        plt.pause(0.001)
        plt.close('all')


    except KeyboardInterrupt:
        print("program aborted")
        plt.close('all')
        exit()
    except:
        print("error")
        plt.close('all')
        exit()