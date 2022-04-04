
# GLEAM CompProg.py 
# Rev D
# 3/22/2022
# 
# Signed: M. Pelissero, S. Shaw, H. Bhushan, M. Nemera
#
#
# A python 3 module for main EV computer operations in the GLEAM MNSGC challenge. Gains sensor data and transmits back to HB through the CU
# 
# Rev Tracking:
# Rev A --- first release. Reads 4 sensors (IMU, IR Prox, IR Thermal Cam, and LiDAR) and prints to console user - S. Shaw, M. Pelissero
# Rev B --- added file saving function for sending data from EV to HB - H. Bhushan, M. Nemera
# Rev C --- added initial static LED and servo assignments - S. Shaw
# Rev D --- added control csv read in and scan function definition/bit-flip operation - S. Shaw, M. Pelissero


import serial			        # Lidar
import time  			        # Lidar, Therm Cam, IMU, Prox
import board			        # Therm Cam, IMU, Prox
import busio 			        # Therm Cam, Prox
import numpy as np  		    # Therm Cam, IMU
import adafruit_mlx90640        # Therm Cam
import adafruit_lsm9ds1		    # IMU
import adafruit_vl53l0x		    # Prox
from gpiozero import LEDBoard   # LED contorl
from signal import pause        # LED control
import RPi.GPIO as GPIO         # Servo Control

import csv                      # for reading csv control file
from csv import writer          # write data to csv file


from datetime import datetime   #import datetime for appending time to csv


itCount = 0

# Initialize Servo Motors and LEDs ---------------------------------------------------------------------------------------------------------------------------

leds = LEDBoard(4, 17, 27, 22, 10, 9) #Hardware definition of GPIO pins to LED control lines
leds.value = (0, 0, 0, 0, 0, 0) #Initialize LED's to on

ledbits = np.zeros((6,)) #define array of LED bits
serv1 = 5.0 #define servo1 starting position
serv2 = 10.0 #define servo2 starting position
serv1Last = 5.0
serv2Last = 10.0
        
servo1PIN = 23 #Hardware def servo 1 PWM line
servo2PIN = 24 #Hardware def servo 2 PWM line
servo3PIN = 25 #Hardware def servo 3 PWM line
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo1PIN, GPIO.OUT)
GPIO.setup(servo2PIN, GPIO.OUT)
GPIO.setup(servo3PIN, GPIO.OUT)

p1 = GPIO.PWM(servo1PIN, 50) # servo PWM GPIO line for PWM with 50Hz
p1.start(0) # Initialization servo 1
p2 = GPIO.PWM(servo2PIN, 50) # servo PWM GPIO line for PWM with 50Hz
p2.start(0) # Initialization servo 2
p3 = GPIO.PWM(servo3PIN, 50) # servo PWM GPIO line for PWM with 50Hz
p3.start(0) # Initialization servo 3


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
    #print(ser.in_waiting)
    count = ser.in_waiting
    if (count == 0 and tryCount < 5):
        p3.ChangeDutyCycle(6.5)
        time.sleep(0.1)
        #print('retrying servo')
        #print(ser.in_waiting)
        count = ser.in_waiting
        tryCount = tryCount + 1
        #print('Try Count: ', tryCount)
    if count > 8:
        recv = ser.read(9)   
        ser.reset_input_buffer() 
        # type(recv), 'str' in python2(recv[0] = 'Y'), 'bytes' in python3(recv[0] = 89)
        # type(recv[0]), 'str' in python2, 'int' in python3 
        
        if recv[0] == 0x59 and recv[1] == 0x59:     #python3
            distance = recv[2] + recv[3] * 256
            strength = recv[4] + recv[5] * 256
            #print('(', distance, ';', strength, ')')
           
            # format and store dist and strength in lidar.csv
            now = datetime.now()
            ct = now.strftime("%H:%M:%S") 
            
            list =[ct, "{0};{1}".format(distance, strength)]
            with open('data/lidar.csv', 'a') as f_object:
                # Pass this file object to csv.writer()
                # and get a writer object
                writer_object = writer(f_object)

                # Pass the list as an argument into
                # the writerow()
                writer_object.writerow(list)

                #Close the file object
                f_object.close()
            print('TF mini reading and strength: ',"{0};{1}".format(distance, strength))
            ser.reset_input_buffer()


# function for lidar scan (mapping)
def scanTFminiData(val):
	#time.sleep(0.1)
    #print(ser.in_waiting)
    count = ser.in_waiting
    tryCount = 0
    if (count == 0 and tryCount < 5):
        p3.ChangeDutyCycle(6.5)
        time.sleep(0.1)
        #print('retrying servo')
        #print(ser.in_waiting)
        count = ser.in_waiting
        tryCount = tryCount + 1
        #print('Try Count: ', tryCount)
    if count > 8:
        recv = ser.read(9)   
        ser.reset_input_buffer() 
            
        if recv[0] == 0x59 and recv[1] == 0x59:     #python3
            distance = recv[2] + recv[3] * 256
            strength = recv[4] + recv[5] * 256
            #print('(', distance, ',', strength, ',', val, ')')
            
            list =["{0},{1},{2}".format(distance, strength, val)]
            with open('data/lidarScan.csv', 'a') as f_object:
                    
                writer_object = writer(f_object)
                writer_object.writerow(list)
                f_object.close()
            print('TF mini Scan Values: ', list)
            ser.reset_input_buffer()  


# Initialize Lidar Scan Parameters-------------------------------------------------------------------------------------------------------------------------------------

val = 2 #servo control value starts at 2 or perpendicular to the front of the EV
ScanLast = 0 #define variable for last scan bit value to compare to new scan bit value
ScanNew = 0 #define variable for new scn bit value that begins new lidar mapping scan when value is changed in control csv file


# Initialize Prox -------------------------------------------------------------------------------------------------------------------------------------
try:
    i2c = busio.I2C(board.SCL, board.SDA) #Initialize I2C bus
    vl53 = adafruit_vl53l0x.VL53L0X(i2c) #Initialize sensor

except:
    print('failed to find Prox sensor')
    #continue

# Initialize Therm Cam -------------------------------------------------------------------------------------------------------------------------------

try:
    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000) # setup I2C  
    mlx = adafruit_mlx90640.MLX90640(i2c) # begin MLX90640 with I2C comm
    mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_2_HZ # set refresh rate
    ser = serial.Serial('/dev/serial0',115200,timeout = 1) #end ThermCam

except:
    print('failed to find thermal cam')
    #continue

frame = np.zeros((24*32,)) # setup array for storing all 768 temperatures


# -----------------------------------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------- Main Loop EV operation loop --------------------------------------------------------------
# -----------------------------------------------------------------------------------------------------------------------------------------------

while True:
    try:

        itCount = itCount + 1

        
    # ------ Run Therm Cam -------------------------------------------------------------------------------------------------------------------------------
        try:
            mlx.getFrame(frame) # read MLX temperatures into frame var
        except ValueError:
            print('IR Cam Error')
            continue # if error, just read again   
            
        now = datetime.now()
        ct = now.strftime("%H:%M:%S")
        
        print('average temp', "{0:0.3f}".format(np.mean(frame)))
        list =[ct, "{0}".format(frame)]
        if (itCount == 5):
            with open('data/ir.csv', 'a') as f_object:
                        # Pass this file object to csv.writer() and get a writer object
                        writer_object = writer(f_object)
                        # Pass the list as an argument into the writerow()
                        writer_object.writerow(list)
                        #Close the file object
                        f_object.close()
           
            itCount = 0
            
            
        #print(repr(frame))

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
        AccXangle= AccXangle-175.0
        if AccYangle > 90:
            AccYangle = AccYangle-261.0
        else:
            AccYangle = AccYangle + 80

        print("X angle, Y angle, and Magnetic Field from IMU")
        
        now = datetime.now()
        ct = now.strftime("%H:%M:%S") 
        
        list =[ct,
        "{0:0.3f}".format(AccXangle),
        "{0:0.3f}".format(AccYangle),
        "{0:0.3f};{1:0.3f};{2:0.3f}".format(mag_x, mag_y, mag_z)
        ]
        
        print("Angle:", "{0:0.3f}".format(AccXangle),"{0:0.3f}".format(AccYangle),'\n',"Magnetic:", "{0:0.3f};{1:0.3f};{2:0.3f}".format(mag_x, mag_y, mag_z))
        # Open our existing CSV file in append mode Create a file object for this file
        with open('data/imu.csv', 'a') as f_object:
            # Pass this file object to csv.writer()
            # and get a writer object
            writer_object = writer(f_object)

            # Pass the list as an argument into
            # the writerow()
            writer_object.writerow(list)

            #Close the file object
            f_object.close()


    # ------ Run Lidar -------------------------------------------------------------------------------------------------------------------------------
        
        try:
            getTFminiData()
        except:
            print('lidar error')
            continue


    # ------ Run Prox -------------------------------------------------------------------------------------------------------------------------------
        
        try:
            print("Proximity Reading: {0}mm".format(vl53.range),'\n')
            
            now = datetime.now()
            ct = now.strftime("%H:%M:%S") 
            
            list =[ct, "{0}".format(vl53.range)]
            with open('data/proximity.csv', 'a') as f_object:
                        # Pass this file object to csv.writer()
                        # and get a writer object
                        writer_object = writer(f_object)

                        # Pass the list as an argument into
                        # the writerow()
                        writer_object.writerow(list)

                        #Close the file object
                        f_object.close()
            #print(list)
        except:
            print('Prox Sensor Failure!!', '\n')
            #continue


    # ------ Read servo, LED control, and lidar scan flip-bit values ----------------------------------------------------------------------------------------------------------
    
    #read control csv
        try:
            #print('servo test')
            controls = open('controls/controls.csv')
            
            csvreader = csv.reader(controls, quotechar='"')
            st = ''
            for row in csvreader:
                st = st + str(row)
            st = st.replace("'","")
            st = st.replace("[","")
            st = st.replace("]","")
            control = st.split(",")
            #print('controls: ', control)
        #define led bits
            ledbits = [0]*6
            for i in range (0, 6):
                ledbits[i] = int(control[i + 2])
        #define servo values
            serv1 = float(control[0])
            serv2 = float(control[1])
            #print('servo 1: ', serv1)
        #define lidar scan mode (scans upon bit-flip)
            ScanNew = int(control[8])
            
        except:
            print('error reading in controls')
            continue


    # ------ LED Control --------------------------------------------------------------------------------------------------------------------------------

        leds.value = ledbits
        

    # ------ Servo Motor Control -----------------------------------------------------------------------------------------------------------------------
        if(serv1 != serv1Last):
            p1.ChangeDutyCycle(serv1)
            time.sleep(0.5)
            p1.ChangeDutyCycle(0)
            serv1Last = serv1
        if(serv2 != serv2Last):
            p2.ChangeDutyCycle(serv2)	
            time.sleep(0.5)
            p2.ChangeDutyCycle(0)
            serv2Last = serv2
        
        
    # ------ Lidar Scanning (mapping) -----------------------------------------------------------------------------------------------------------------------                    
        
        if (ScanLast != ScanNew):
            i = 0
            try:
                list=['new']
                with open('data/lidarScan.csv', 'a') as f_object:
                    
                    writer_object = writer(f_object)
                    writer_object.writerow(list)
                    f_object.close()
                while (i < 3):
                    scanTFminiData(val)
                    val = val + 0.2
                    if (val > 11):
                        val = 2
                        i = i + 1
                    p3.ChangeDutyCycle(val)
                    time.sleep(0.05)
                    p3.ChangeDutyCycle(0)
                
                ScanLast = ScanNew
                p3.ChangeDutyCycle(6.5)
                time.sleep(0.1)
                p3.ChangeDutyCycle(0)
            
            except KeyboardInterrupt:
                continue
                
            except:
                p3.stop()
                print('Scan failed. Probably TF mini error')

    # ------ Delay between readings ---------------------------------------------------------------------------------------------------------------------
        #time.sleep(1)		# End IMU
       
        
    except KeyboardInterrupt:
        p1.stop()
        p2.stop()
        p3.stop()
        print('now exiting normal run-mode')
        exit()
        
    #except:
        #p1.stop()
        #p2.stop()
        #p3.stop()
        #print('code error')
        #exit()
