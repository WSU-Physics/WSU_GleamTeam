from subprocess import Popen
import time

while (True):
    p = Popen("lol.bat")
    stdout, stderr = p.communicate()
    print ("syncing files")
    #time.sleep(2)