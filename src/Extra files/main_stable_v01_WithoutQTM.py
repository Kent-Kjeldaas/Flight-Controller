'''
###MAIN OFF BOARD###
Initial:
Open GUI <- if time?
User plots route
User start signal -> quad
Quad: Checks inital conditions
Check connection with qualisys
Check if not error on Quad or Qual (requires two-way communication)
else display error msg
 
If READY:
Start logging data
And run loop
 
Loop:
Receieve data
Check mode
Computation
Send mode
'''
 
'''
###MAIN ON BOARD###
Setup:
Wait for signal from OFF BOARD
Checks inital conditions:
   Initialise motor
   Initalise leds
   Initalise IMU?
Check IMU data
Arm motor
Test each of them to see if we get response
Check if not error,
else display error msg
 
If READY:
Loop:
When possible:
Receive data from Qual
 
Each 20ms:
Setpoint for roll, pitch and yaw
Receive data from IMU
PID Compute
Update motor output
'''
 
##import qtmLocal as qtm
import bluetooth
import time
import sys
import math
##from recieveTest import *
 
#SETTINGS:
kp_X = 0.6
kp_Y = 0.6
kp2_X = 0.3
kp2_Y = 0.3
desiredX = 0.0
desiredY = 0.0
desiredZ = 1000
Pitch = 0.0
Roll = 0.0
Throttle = 1400
 
##Temprorarily variables
currentX = 0
currentY = 0
currentZ = 0
 
lastX = 0
lastY = 0
lastZ = 0
 
thrust = 1400
## ------------------
 
 
 
#VARIABLES:
bd_address = "20:15:08:13:78:88"
port = 1
sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((bd_address, port))
info ="init"
 
#Check connection with Qualisys:
'''
with qtm.QTMClient() as qt:
  qt.setup()
  qt.getAttitude()
 
  currentX = qt.getBody(0)['linear_x']
  currentY = qt.getBody(0)['linear_y']
  currentZ = qt.getBody(0)['linear_z']
  angularX = qt.getBody(0)['angular_x']
  angularY = qt.getBody(0)['angular_y']
  angularZ = qt.getBody(0)['angular_z']
 
  lastX = currentX
   lastY = currentY
   lastZ = currentZ
 
   desiredX = currentX
   desiredY = currentY
 
   if(math.isnan(currentX) || math.isnan(currentY) || math.isnan(currentZ) ||):
       print "X-Y-Z CHECK"
       if(math.isnan(angularX) || math.isnan(angularY) || math.isnan(angularZ)):
           print "Phi-Theta-Psi CHECK"
       else:
           print "NO Angles"
   else:
       print ("No Data Received from Qualisys!")
       time.sleep(2)
       sys.exit("No Data Received from Qualisys!")
'''
time.sleep(0.05)
#Send roll and pitch
sock.send(info)  
#receive
data = sock.recv(1024)
time.sleep(1)
if(data == "r"):
    #Receive Data
    while True:
        '''
       ###Balanced and stay within a given 30x30x30cm box###
 
       Quadcopter should be zero in theta and phi angle.
       When drifting outside the box it should slightly adjust its angle
       in order to get back inside of the box.
       Give more thrust if bellow z-axis, and less if above.
       The magnitude of the correction should be decided upon the rapidness of the change.
       Do we care about psi?
       #Define local variables
 
       #Read qual data
     
       qt.getAttitude()
       currentX = qt.getBody(0)['linear_x']
       currentY = qt.getBody(0)['linear_y']
       currentZ = qt.getBody(0)['linear_z']
       angularX = qt.getBody(0)['angular_x']
       angularY = qt.getBody(0)['angular_y']
       angularZ = qt.getBody(0)['angular_z']
 
       #For Z-axis: adjust thrust and pitch/roll angle?
       #Compute
       #Find velocity (DO IT FOR EACH 100ms?) <------- NOT FINISHED
       '''
        Velocity_x = currentX - lastX
        Velocity_y = currentY - lastY
        velocity_z = currentZ - lastZ
 
        #Save new last position
        lastX = currentX
        lastY = currentY
        lastZ = currentZ
 
        #Find distance
        errorX = desiredX - currentX
        errorY = desiredY - currentY
        errorZ = desiredZ - currentZ
 
        #STAY / margin of error
        if(errorX < 30):
            errorX = 1
            #stabilize for roll
            #pay attention to speed
        if(errorY < 30):
            errorY = 2
            #stabilize for pitch
            #pay attention to speed
        if(errorZ < 30):
            errorZ = 2
            #stabilize for yaw
            #pay attention to speed
 
        Speed_X = errorX * kp_X
        Speed_Y = errorY * kp_Y
 
        if(Speed_X >= 0):
          errorSpeed_X = Speed_X - currentX #wrong
       
        else:
          errorSpeed_X = Speed_X + currentX
       
        if(Speed_Y >= 0):
          errorSpeed_Y = Speed_Y - currentY
       
        else:
          errorSpeed_Y = Speed_Y + currentY  
 
        #calculate some force
        dKraft_X = errorSpeed_X * kp2_X
        dKraft_Y = errorSpeed_Y * kp2_Y
        #find a corralation between force and angle
        phi = thrust + dKraft_X
        theta = thrust - dKraft_Y
 
        #Set roll, pitch, throttle to some values
        Roll = 0
        Pitch = 0
        Throttle = 1400
        #Send roll, pitch, throttle
        print"Sending data:"
        commands = [str(Roll), str(Pitch), str(Throttle)]
        commandString = ''.join(commands)
        sock.send(commandString)
 
else:
    print"quadcopter is not ready"
    time.sleep(2)