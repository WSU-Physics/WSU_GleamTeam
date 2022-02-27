
# GLEAM CompProg.py 
# Rev A
# 2/5/2022
# 
# Signed: M. Pelissero, S. Shaw
#
#
# A python 3 module for main EV computer operations in the GLEAM MNSGC challenge. Gains sensor data and transmits back to HB through the CU
# 
# Rev Tracking:
# Rev A --- first release. Reads 4 sensors (IMU, IR Prox, IR Thermal Cam, and LiDAR) and prints to console user - S. Shaw, M. Pelissero



import serial			#Lidar
import time  			# Lidar, Therm Cam, IMU, Prox
import board			# Therm Cam, IMU, Prox
import busio 			# Therm Cam, Prox
import numpy as np  		#Therm Cam, IMU
import adafruit_mlx90640        #Therm Cam
import adafruit_lsm9ds1		#IMU
import adafruit_vl53l0x		#Prox

from csv import writer


# Initialize IMU -------------------------------------------------------------------------------------------------------------------------------------

DT=0.02         # [s/loop] loop period. 20ms         
AA=0.97         # complementary filter constant
A_GAIN=0.0573    # [deg/LSB]
G_GAIN=0.070     # [deg/s/LSB]
RAD_TO_DEG=57.29578
M_PI=3.14159265358979323846

# Create sensor object, communicating over the board's default I2C bus
i2c = board.I2C()  # uses board.SCL and board.SDA
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)
rate_gyr_y = 0.0   # [deg/s]
rate_gyr_x = 0.0   # [deg/s]
rate_gyr_z = 0.0   # [deg/s]

gyroXangle = 0.0
gyroYangle = 0.0
gyroZangle = 0.0
AccYangle = 0.0
AccXangle = 0.0
CFangleX = 0.0
CFangleY = 0.0


# Fct Def Lidar -------------------------------------------------------------------------------------------------------------------------------------

def getTFminiData():    
        #time.sleep(0.1)
        count = ser.in_waiting
        if count > 8:
            recv = ser.read(9)   
            ser.reset_input_buffer() 
            # type(recv), 'str' in python2(recv[0] = 'Y'), 'bytes' in python3(recv[0] = 89)
            # type(recv[0]), 'str' in python2, 'int' in python3 
            
            if recv[0] == 0x59 and recv[1] == 0x59:     #python3
                distance = recv[2] + recv[3] * 256
                strength = recv[4] + recv[5] * 256
                #print('(', distance, ';', strength, ')')
                # format and store dist and strength pin lidar.csv
                list =["{0};{1}".format(distance, strength)]
                with open('lidar.csv', 'a') as f_object:
                    # Pass this file object to csv.writer()
                    # and get a writer object
                    writer_object = writer(f_object)

                    # Pass the list as an argument into
                    # the writerow()
                    writer_object.writerow(list)

                    #Close the file object
                    f_object.close()
                print(list)
                ser.reset_input_buffer()
                
            if recv[0] == 'Y' and recv[1] == 'Y':     #python2
                lowD = int(recv[2].encode('hex'), 16)      
                highD = int(recv[3].encode('hex'), 16)
                lowS = int(recv[4].encode('hex'), 16)      
                highS = int(recv[5].encode('hex'), 16)
                distance = lowD + highD * 256
                strength = lowS + highS * 256
                #print(distance, strength)
            
            # you can also distinguish python2 and python3: 
            #import sys
            #sys.version[0] == '2'    #True, python2
            #sys.version[0] == '3'    #True, python3

# Initialize Prox -------------------------------------------------------------------------------------------------------------------------------------

# Initialize I2C bus and sensor.			
i2c = busio.I2C(board.SCL, board.SDA)
vl53 = adafruit_vl53l0x.VL53L0X(i2c)


# Initialize Therm Cam -------------------------------------------------------------------------------------------------------------------------------

i2c = busio.I2C(board.SCL, board.SDA, frequency=400000) # setup I2C  
mlx = adafruit_mlx90640.MLX90640(i2c) # begin MLX90640 with I2C comm
mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_2_HZ # set refresh rate
ser = serial.Serial('/dev/serial0',115200,timeout = 1) #end ThermCam

frame = np.zeros((24*32,)) # setup array for storing all 768 temperatures

# -----------------------------------------------------------------------------------------------------------------------------------------------
# ------------------------------------ Main Loop for reading and sending data from sensors ------------------------------------------------------
# -----------------------------------------------------------------------------------------------------------------------------------------------

while True:
# ------ Run Therm Cam -------------------------------------------------------------------------------------------------------------------------------
        try:
            mlx.getFrame(frame) # read MLX temperatures into frame var
        except ValueError:
            continue # if error, just read again   
            print (hello) 
        list =["{0}".format(frame)]
        with open('ir.csv', 'a') as f_object:
                    # Pass this file object to csv.writer()
                    # and get a writer object
                    writer_object = writer(f_object)

                    # Pass the list as an argument into
                    # the writerow()
                    writer_object.writerow(list)

                    #Close the file object
                    f_object.close()
        #print(list)
        print(repr(frame))

# ------ Run IMU -------------------------------------------------------------------------------------------------------------------------------
            # Read acceleration, magnetometer, gyroscope, temperature.
        
        accel_x, accel_y, accel_z = sensor.acceleration
        mag_x, mag_y, mag_z = sensor.magnetic
        gyro_x, gyro_y, gyro_z = sensor.gyro
        temp = sensor.temperature

        mag= mag_x, mag_y, mag_z
       
        #Convert Gyro raw to degrees per second
        rate_gyr_x = gyro_x*G_GAIN
        rate_gyr_y = gyro_y*G_GAIN
        rate_gyr_z = gyro_z*G_GAIN

        #Calculate the angles from the gyro
        gyroXangle= gyroXangle + (rate_gyr_x*DT)
        gyroYangle= gyroYangle + (rate_gyr_y*DT)
        gyroZangle= gyroZangle + (rate_gyr_z*DT)

        #Convert Accelerometer values to degrees
        AccXangle = (np.arctan2(accel_y,accel_z)+M_PI)*RAD_TO_DEG
        AccYangle = (np.arctan2(accel_z,accel_x)+M_PI)*RAD_TO_DEG

        #If IMU is up the correct way, use these lines
        AccXangle= AccXangle-180.0
        if AccYangle > 90:
            AccYangle = AccYangle-270
        else:
            AccYangle = AccYangle + 90

        #Complementary filter used to combine the accelerometer and gyro values.
        CFangleX=AA*(CFangleX+rate_gyr_x*DT) +(1 - AA) * AccXangle
        CFangleY=AA*(CFangleY+rate_gyr_y*DT) +(1 - AA) * AccYangle

        print("X angle, Y angle, Magnetic Field, and Temperature from IMU")
        #print(AccXangle, AccYangle, mag, temp) 
        
        list =["{0:0.3f}".format(AccXangle),
        "{0:0.3f}".format(AccYangle),
        "{0:0.3f};{1:0.3f};{2:0.3f}".format(mag_x, mag_y, mag_z),
        "{0:0.3f}".format(temp)]
        
        print(list)
        #Open our existing CSV file in append mode
        # Create a file object for this file
        with open('imu.csv', 'a') as f_object:
            # Pass this file object to csv.writer()
            # and get a writer object
            writer_object = writer(f_object)

            # Pass the list as an argument into
            # the writerow()
            writer_object.writerow(list)

            #Close the file object
            f_object.close()


    # ------ Run Lidar -------------------------------------------------------------------------------------------------------------------------------
        print("TF Mini Lidar Reading (m)")
        print(getTFminiData())


    # ------ Run Prox -------------------------------------------------------------------------------------------------------------------------------
        print("proximity sensor range")
        print("Range: {0}mm".format(vl53.range))
        
        list =["{0}".format(vl53.range)]
        with open('proximity.csv', 'a') as f_object:
                    # Pass this file object to csv.writer()
                    # and get a writer object
                    writer_object = writer(f_object)

                    # Pass the list as an argument into
                    # the writerow()
                    writer_object.writerow(list)

                    #Close the file object
                    f_object.close()
        #print(list)


    # ------ Delay between readings ---------------------------------------------------------------------------------------------------------------------
        time.sleep(0.1)		# End IMU
