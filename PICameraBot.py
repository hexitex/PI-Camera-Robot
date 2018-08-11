
#!/usr/bin/python
#---------------------------------
from __future__ import division
import subprocess

import os
import re
# Import required libraries
import sys
import time
import RPi.GPIO as GPIO
import picamera
import threading
from threading import Thread, current_thread
import queue
from datetime import datetime
import math
from fractions import Fraction
from astral import Astral
import pytz

# Import the PCA9685 module. only needed if servos are in the picture
import Adafruit_PCA9685

startservos=0

try:
# Initialise the PCA9685 using the default address (0x40).
  pwm = Adafruit_PCA9685.PCA9685()

# Alternatively specify a different address and/or bus
#pwm = Adafruit_PCA9685.PCA9685(address=0x41)

# Set frequency to 50hz, good for cheep servos.
  pwm.set_pwm_freq(50)
  startservos=1;
except Exception as e:
  print('Adafruit PCA9685 servo controller not connected '+str(e))

#globals 
exitflag=0
camexec=0
processexec=0
freezeMotion=False
resetPosition=False
resetMidPosition=False
night=0
daylightchange=1
concatFileCounter=1
cambusy=0
cityName='London'
num_words=("zero","one","two","three","four","five","six","seven","eight","nine")

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# these define the pins used for each motor, you can use other combinations 
StepPins1 = (17,18,27,22)
StepPins2 = (23,24,25,4)
StepPins3 = (13,12,6,5)

# Set all pins low and as output, we should do this to turn off any high pins in case of a previous op
for pin in StepPins1:
  GPIO.setup(pin,GPIO.OUT)
  GPIO.output(pin, False)
for pin in StepPins2:
  GPIO.setup(pin,GPIO.OUT)
  GPIO.output(pin, False)
for pin in StepPins3:
  GPIO.setup(pin,GPIO.OUT)
  GPIO.output(pin, False)
GPIO.setup(21,GPIO.OUT) #camera off?
# setup sequences for little stepper motors
# energise two coils for best torque
SeqHighTQ =((0,0,1,1),
           (0,1,1,0),
           (1,1,0,0),
           (1,0,0,1))
# a combination of single and dual coils for smooth movement - micro stepping!
SeqSmooth =((0,0,0,1),
           (0,0,1,1),
           (0,0,1,0),
           (0,1,1,0),
           (0,1,0,0),
           (1,1,0,0),
           (1,0,0,0),
           (1,0,0,1))
# single coils, tick tock - good for time lapse i suppose!
SeqLowTQ=((0,0,0,1),
           (0,0,1,0),
           (0,1,0,0),
           (1,0,0,0))

# default wait time, 12ms provides best torque, min should be no less than 4ms
WaitTimeHT = 12/1000 # high torque
WaitTimeFast =6/1000 # fast but can stick
WaitTimeFastest= 4/1000 # faster with very low torque
WaitTimeSlow=40/1000 # slow with torque due to coil being energised during wait between steps
WaitTimeSlower=80/1000 # as above
WaitTimeSlowerStill=160/1000 # as above
WaitTimeReallySlow=320/1000 # as above
#WaitTimeSlowest=10024/float(1000) # as above
WaitTimeSlowest=5
def getCurrentMTime():
  return int(round(time.time()*1000))
class aThread (Thread):
    def __init__(self, threadID, name, q, lock):
        Thread.__init__(self)
        self.threadID=threadID
        self.name=name
        self.q=q
        self.lock=lock
    
    def run(self):
        time.sleep(1)
        try:
              print("\n Starting "+self.name+ " functions")
              doGenAction(self.name,self.q,self.lock)
              print ("\n Done with "+self.name+ " functions")
              
        except Exception as e:
           print(str(e))
           raise

class bThread (Thread):
    def __init__(self, threadID,name):
        Thread.__init__(self)
        self.threadID=threadID
        
    def run(self):
        global night
        global daylightchange
        global concatFileCounter
        global cambusy
        global resetPosition
        
        while not exitflag:
        
          try:
             #print("\n Starting light check")
             cityname=cityName
             a=Astral()
             city=a['London']
             now=datetime.now(pytz.utc)
             sun=city.sun(date=now,local=True)
             oldnight=night
             if now >=sun['dusk'] or now<=sun['dawn']:
               night=1
             else:
               night=0

             if oldnight!=night:
               daylightchange=1
               print ("\n Done with lightcheck, night is "+str(night))
            # print("\n 1") 
          except Exception as e:
             print(str(e))
             raise
         
          try:
           
            Camera_lock.acquire(0)
          
            if CameraQ.empty() and not cambusy==1 and not resetPosition:
             
              daylightchange=1
              concatFileCounter=concatFileCounter+1
              #shots to take, every n secs, compile into movie every n shots
              CameraQ.put(lambda: takeShot(9000,5,100))
              #reset
              print('added to cam q')
            
            Camera_lock.release()

# we are moving (main body) head back and forth
            Motor2_lock.acquire(0)
            if Motor2Q.empty() and not resetPosition:
              Motor2Q.put(lambda: Step('motor2',StepPins2,45,False,SeqHighTQ,WaitTimeSlowest))
              Motor2Q.put(lambda: Step('motor2',StepPins2,-45,False,SeqHighTQ,WaitTimeSlowest))
            Motor2_lock.release()

# here we are head moving left to right
            Motor3_lock.acquire(0)
            if Motor3Q.empty() and not resetPosition:
              Motor3Q.put(lambda: Step('motor3',StepPins3,-60,False,SeqHighTQ,WaitTimeSlowest))
              Motor3Q.put(lambda: Step('motor3',StepPins3,60,False,SeqHighTQ,WaitTimeSlowest))
            Motor3_lock.release()

            
# scrool along             
            Motor1_lock.acquire(0)
            if Motor1Q.empty() and not resetPosition:
              Motor1Q.put(lambda: Step('motor1',StepPins1,-3200,False,SeqHighTQ,WaitTimeSlowerStill ))
              Motor1Q.put(lambda: Step('motor1',StepPins1,3200,False,SeqHighTQ,WaitTimeSlowerStill ))
              Motor1Q.put(lambda: Step('motor1',StepPins1,-3200,False,SeqHighTQ,WaitTimeSlowerStill ))
              Motor1Q.put(lambda: Step('motor1',StepPins1,3200,False,SeqHighTQ,WaitTimeSlowerStill ))
            Motor1_lock.release()
          except Exception as cm:
            print(str(cm))
            Camera_lock.release()
            Motor1_lock.release()
            raise
          #print('c')
          time.sleep(10)

def doGenAction(name,q,lock):
  global resetPosition #lambda function will use this during resetPositions
  
  while not exitflag:
        try:
          lock.acquire()
          if not q.empty():
            p=q.get()
            print("\nStarting job for "+name)
            p()
            lock.release()  
          else:
            lock.release()
            time.sleep(1)

        except Exception as err:
            print(err)
  print('Exiting!')

# Main stepping function:
def Step(name,stepPins,steps,hold,seq,waitTime):
    "steps the pins that are passed in"

    if waitTime is None:
        waitTime=WaitTimeFastest
        
    if seq is None:
        seq=seqHighTQ
        
# Set for clockwise -1 for anti-clockwise
    # est time
    t=((steps* (len(seq)*waitTime))/60)/60
   # outmsg=
    print ("\nStepping {n} for {s} steps over %1.2f hours".format(n=name,s=steps) %(t))
    stepDir=1

    stepCount = len(seq)
   # print ("step count is "+str(StepCount))
    
    if (steps<0):
        stepDir = -1
    
    stepCounter=0
    subStepCounter=0
    laston=0
    
    while stepCounter!=steps :
      #  print ("in while")
        if resetMidPosition:
          stepCounter=steps
          break
        if not freezeMotion:
                   # print("freezeMotion is "+str(freezeMotion))
                    for pin in range(0, 4):
                        xpin = stepPins[pin]
                        if seq[subStepCounter][pin]!=0:
                          GPIO.output(xpin, True)
                          laston=xpin
                        else:
                          GPIO.output(xpin, False)
                    subStepCounter += stepDir
        
      # If we reach the end of the sequence start again
                    if (subStepCounter>=stepCount):
                       subStepCounter = 0
                       stepCounter += stepDir
                    if (subStepCounter<0):
                        subStepCounter = stepCount+stepDir
                        stepCounter += stepDir
                        time.sleep(0.001) #?
                   # if (WaitTime>2):
                    #    KillPins(StepPins)
                    time.sleep(waitTime)
        
    if hold==True:
        GPIO.output(laston, True)
    else:
        KillPins(stepPins)
        return
        
            
def KillPins(stepPins):
     for pin in range(0, 4):
            xpin = stepPins[pin]
            GPIO.output(xpin, False)
            
def KillServo(channel):
    pwm.set_pwm(channel,0,0)

def waitForSeconds(secs):
    print("\nWaiting for "+str(secs)+" seconds")
    time.sleep(secs)
    
def takeShot(numberOfShots,timeBetweenShots,createMovieEveryNShots):
    global freezeMotion
    global daylightchange
    global concatFileCounter
    global cambusy
    global resetMidPosition
    global resetPosition
    cambusy=1
    #pq.put(subprocess.call("ls",shell=True))
    print ("\nTaking {n} shots over %1.2f hours".format(n=numberOfShots) %((((timeBetweenShots*numberOfShots)/60)/60)) )
    day=time.strftime("%d")
    month=time.strftime("%m")
    year=time.strftime("%Y")
    lconcatFileCounter=concatFileCounter
    fileName="/home/pi/timelapse/concatFile"+str(lconcatFileCounter)+".txt"
    try:
      os.remove(fileName)
    except Exception as fnf:
      print(str(fnf))
    
    concatFile= open(fileName,"w")
    concatFile.write("#Concat File\n")
    processNumberOfShots=0
    batch=0
    shotInBatch=0
    # construct the camrea and make changes as needed for day and night
    camera=picamera.PiCamera()
    picamera.PiCamera.CAPTURE_TIMEOUT=60
    for x in range(0,numberOfShots):
        if daylightchange==1:
          daylightchange=0
          try:
            camera.close()
          except Exception as camerr:
            print(str(camerr))
          if night==1:
            camera = picamera.PiCamera(resolution=(1640, 922))#Fraction(1,6))
            camera.exposure_mode = 'antishake'
            camera.awb_mode = 'tungsten'
        
            time.sleep(3)
         
          else:
            try:
              camera.close()
            except Exception as camerr:
              print(str(camerr))
            camera = picamera.PiCamera(resolution=(1640, 922), framerate=24)#Fraction(1,6))
            camera.iso = 200
            time.sleep(2)
            camera.exposure_mode = 'auto'
            camera.awb_mode = 'auto'
            g = camera.awb_gains
            camera.awb_mode = 'off'
            camera.awb_gains = g
        
        processNumberOfShots+=1
        thistime=getCurrentMTime()
        freezeMotion=True
        try:
          camera.capture('/mnt/monitor/img'+numberToWord(batch)+'{:04d}'.format(shotInBatch)+'.jpg')
          print(x)
        except Exception as ce:
          print(str(ce))
        freezeMotion=False
        adjust=(getCurrentMTime()-thistime)/1000
        #print('\n It took '+str(adjust)+' to take photo')
        shotInBatch+=1
       

        if (createMovieEveryNShots>1):
            if (processNumberOfShots>=createMovieEveryNShots):
                print("trying processing shots")
           
            # one letter away from batch:)
                bitch=batch
                Process_lock.acquire()
                # ffmpeg options -y=overwrite file if there -hide_banner=make less verbose as does -loglevel -panic -i=input file coming inputfiles edning in a number (must start between 1 and 4) -frames:v frames to encode -codec:v=just copy the jpgs into the movie file then the outputfile
                ProcessQ.put(lambda: doSubprocess('ffmpeg -y -hide_banner -loglevel panic -i /mnt/monitor/img'+numberToWord(bitch)+'%04d.jpg -frames:v '+str(createMovieEveryNShots)+' -s 1640x922 -framerate 24 -b:v 15000k /mnt/monitor/movie'+str(bitch)+'.mp4'))
       #         ProcessQ.put(lambda: doSubprocess('ffmpeg -y -hide_banner -loglevel panic -i /home/pi/timelapse/img'+numberToWord(bitch)+'%04d.jpg -frames:v '+str(createMovieEveryNShots)+' -s 1920x1080 -framerate '+str(createMovieEveryNShots)+' -codec:v h264_omx -b:v 8000k /home/pi/timelapse/movie'+str(bitch)+'.mp4'))
       
                ProcessQ.put(lambda: waitForSeconds(10)) # it cleans up to quickly the movie hasn't encoded all the frames so we wait!
                ProcessQ.put(lambda: purgeFiles('/mnt/monitor','img'+numberToWord(bitch)+'.*'))#remove jpg files to save on space
                Process_lock.release()
               # concatString+="/home/pi/timelapse/movie"+str(bitch)+".mp4|"
                concatFile.write("file \'/mnt/monitor/movie"+str(bitch)+".mp4\'\n")
                processNumberOfShots=0
                batch+=1
                shotInBatch=0
        
        adjusted=timeBetweenShots-adjust
       # print ('adjusted to: '+str(adjusted))
    #    print(adjusted)
        if adjusted<0:
            adjusted =0.10
            
        time.sleep(adjusted) #adjust for time to take shot
        
    concatFile.close()
    resetMidPosition=True # stop the motors
    resetPosition=True # stop the queue adding
    resetCameraPosition();
    Process_lock.acquire()
    ProcessQ.put(lambda: doSubprocess('ffmpeg -f concat -safe 0 -i '+fileName+' -codec copy /mnt/monitor/movie'+year+month+day+'_'+str(lconcatFileCounter)+'.mp4'))
    ProcessQ.put(lambda: waitForSeconds(20))
    ProcessQ.put(lambda: os.remove(fileName))
    Process_lock.release()

  #  resetMidPosition=True
  #  resetCameraPosition();
    
    # kill the camera for next op, if required
    camera.close()
    cambusy=0
def resetCameraPosition():
  global resetPosition
  global resetMidPosition
  resetPosition=True;
  
  emptyQueue(Motor1Q)
  emptyQueue(Motor2Q)
  emptyQueue(Motor3Q)
  time.sleep(10) # need this (maybe slower) to clear the queue b4 resetMidPosition is negated
  resetMidPosition=False
  Motor1_lock.acquire(0)
  Motor1Q.put(lambda: Step('motor1',StepPins1,3100,False,SeqLowTQ,WaitTimeFastest))
  Motor1Q.put(lambda: Step('motor1',StepPins1,1,False,SeqLowTQ,WaitTimeFastest)) # dummy move to stop the resetPosition being set false
  Motor1Q.put(lambda: resetPositionFlag(False)) ## slowest movement so should be fine to start things again
  Motor1_lock.release()

  Motor2_lock.acquire(0)
  Motor2Q.put(lambda: Step('motor2',StepPins2,-300,False,SeqLowTQ,WaitTimeFastest))
  Motor2_lock.release()

  Motor3_lock.acquire(0)
  Motor3Q.put(lambda: Step('motor3',StepPins3,-300,False,SeqLowTQ,WaitTimeFastest))
  Motor3Q.put(lambda: Step('motor3',StepPins3,110,False,SeqLowTQ,WaitTimeFastest))
  Motor3_lock.release()

def resetPositionFlag(s):
  global resetPosition
  global resetMidposition
  resetPosition=s
  #resetMidPosition=s
  
def emptyQueue(Q):
  while not Q.empty():
    try:
      Q.get(False)
    except Empty as Exception:
        continue
    #Q.task.done()
      
def doSubprocess(cmd):
  try:
    print(cmd)
    pro=subprocess.Popen('exec '+cmd,  stderr=subprocess.STDOUT, stdout=subprocess.PIPE, shell=True)
    pro.wait()
    pro=None
  except OSError as err:
    print (str(err))

def exit():
  global exitflag
  exitflag=1
  
def MoveServo(channel,frompos,topos,speed, killpower):
    #assume running forwards
    direction=1
    servoCounter=frompos
    if (frompos>topos):
        direction=-1
    while servoCounter!=topos:
        
         if not freezeMotion:
        
            if direction==1:
                servoCounter=format(servoCounter+0.01,'.3f')
            else:
                servoCounter=format(servoCounter-0.01,'.3f')
       
            pwm.set_pwm(channel, 0, calculatePulseWidth(servoCounter,50))
           
            #kill pwm signal as on cheap servos we get a lot of jitter

            #if killpower and speed>2:
                #KillServo(channel)

            time.sleep(speed)
            
    KillServo(channel)
            
     
def calculatePulseWidth(millis, frequency):
    return math.trunc(4096 * millis * frequency/1000)
   
def numberToWord (num):
  naMAP=list(map(int, str(num)))
  retVal=""      
  for x in range (len(naMAP)):
    retVal=retVal+num_words[naMAP[x]]
  return retVal

def test():
  for i in range(0,500):
    a=1
# setup threads and be kind to those that do't need setting up  
Motor1_lock = threading.Lock()
Motor2_lock = threading.Lock()
Motor3_lock = threading.Lock()
Servo1_lock = threading.Lock()
Servo2_lock = threading.Lock()
Servo3_lock = threading.Lock()
Process_lock = threading.Lock()
Camera_lock = threading.Lock()
Motor1Q=queue.Queue(maxsize=0)
Motor2Q=queue.Queue(maxsize=0)
Motor3Q=queue.Queue(maxsize=0)
Servo1Q=queue.Queue(maxsize=0)
Servo2Q=queue.Queue(maxsize=0)
Servo3Q=queue.Queue(maxsize=0)
ProcessQ=queue.Queue(maxsize=0)
CameraQ=queue.Queue(maxsize=0)
threads=[]

Process_lock.acquire()
ProcessQ.put(lambda: test())
Process_lock.release()

resetCameraPosition()

""" be kind
Servo1_lock.acquire()
#MoveServo (channel,start_in_milliseconds,end_in_milliseconds,sleep_time_between_pulses,switch_off_PWM_to_servo)
Servo1Q.put(lambda: MoveServo(0,1.3,1.6,45,True))
Servo1Q.put(lambda: MoveServo(0,1.6,1.3,45,True))
Servo1_lock.release()


Servo2_lock.acquire()
Servo2Q.put(lambda: MoveServo(1,1.0,2.0,45,True))
Servo2Q.put(lambda: MoveServo(1,2.0,1.0,45,True))
Servo2_lock.release()


Servo3_lock.acquire()
Servo3Q.put(lambda: MoveServo(2,1.0,2.0,45,True))
Servo3Q.put(lambda: MoveServo(2,2.0,1.0,45,True))
Servo3_lock.release()

"""

#create the threads
thread9=bThread(9,"lightcheck")
threads.append(thread9)
thread9.start()

thread1 =aThread(1,"motor1",Motor1Q,Motor1_lock)
threads.append(thread1)
thread1.start()
thread2 =aThread(2,"motor2",Motor2Q, Motor2_lock)
threads.append(thread2)
thread2.start()
thread3 =aThread(3,"motor3",Motor3Q, Motor3_lock)
threads.append(thread3)
thread3.start()
thread4 =aThread(4,"camera",CameraQ, Camera_lock)
threads.append(thread4)
thread4.start()
if startservos==1:
  thread5 =aThread(5,"servo1",Servo1Q, Servo1_lock)
  threads.append(thread5)
  thread5.start()
  thread6 =aThread(6,"servo2",Servo2Q, Servo2_lock)
  threads.append(thread6)
  thread6.start()
  thread7 =aThread(7,"servo3",Servo3Q, Servo3_lock)
  threads.append(thread7)
  thread7.start()
thread8 =aThread(8,"process",ProcessQ, Process_lock)
threads.append(thread8)
thread8.start()


def purgeFiles(dir,pattern):
    for f in os.listdir(dir):
        if re.search (pattern,f):
            os.remove(os.path.join(dir,f))


try:
    time.sleep(5)
# camera not doing photos or processes then allow exit
    while not exitflag==1:
      pass
except KeyboardInterrupt:
    print("exit on user demand")
except Exception as err:
    print(str(err))
finally:
    # gpio cleanup    
    GPIO.cleanup()
    #servo cleanup
    try:
      for x in range(0,15):
          pwm.set_pwm(x,0,0)
    except Exception as perr:
      pass
    #cleanup files not needed anymore
    purgeFiles('/home/pi/timelapse','img.*')
    purgeFiles('/home/pi/timelapse','.*ts')
    
    
for t in threads:
    t.join()
