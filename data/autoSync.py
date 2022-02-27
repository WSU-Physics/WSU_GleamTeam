from subprocess import Popen
import time

while (True):
    p = Popen("lol.bat", cwd=r"C:\Users\bd1912wh\Desktop\data")
    stdout, stderr = p.communicate()
    print ("cool")
    #time.sleep(2)