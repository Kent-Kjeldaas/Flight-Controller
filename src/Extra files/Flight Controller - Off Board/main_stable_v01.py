import qtmLocal as qtm
import bluetooth
import time
import sys
import math
from threading import Thread

#SETTINGS:
kp_X = 0.2 #Limit desired velocity
kp_Y = 0.2 #Limit desired velocity
kp_Z = 0.06 #Limit desired velocity
kp2_X = 0.3 #Tune roll
kp2_Y = 0.3 #Tune pitch
desiredX = 0.0
desiredY = 0.0
desiredZ = 1500
pitch = 0.0
minPitch = -5
maxPitch = 5
roll = 0.0
minRoll = -5
maxRoll = 5
thrust = 1390
maxThrust = 1590
minThrust = 1190
 
#VARIABLES:
lostContact = 0
mass = 1.856
gravity = 9.81
active = False
start = False
star = '*'
bd_address = "20:15:08:13:78:88"
port = 1
sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((bd_address, port))
info ="init"
 
#Check connection with Qualisys:
def Main():
	print "Starting Qual Update Function"

    t1 = Thread(target=UpdateQualInfo)
    t1.daemon = True
    t1.start()

    time.sleep(3)
    print "Starting Loop"

    t2 = Thread(target=Compute)
    t2.daemon = True
    t2.start()

    print "Main Finished"

def UpdateQualInfo():
	with qtm.QTMClient() as qt:
	     qt.setup()
	     time.sleep(3)
	while(True):
		qt.getAttitude()
	    currentX = qt.getBody(0)['linear_x']
	    currentY = qt.getBody(0)['linear_y']
	    currentZ = qt.getBody(0)['linear_z']
	    angularX = qt.getBody(0)['angular_x']
	    angularY = qt.getBody(0)['angular_y']
	    angularZ = qt.getBody(0)['angular_z']
 
def Compute():
    lastX = currentX
    lastY = currentY
    lastZ = currentZ
 
    desiredX = currentX
    desiredY = currentY
   
    if(not math.isnan(currentX) and not math.isnan(currentY) and not math.isnan(currentZ)): # if (-5000 > currentX > 5000) #if (currentX < 5000 and currentX > -5000)
        print "X-Y-Z CHECK"
        active = True
        #ready for flight
        if(not math.isnan(angularX) and not math.isnan(angularY) and not math.isnan(angularZ)):
            print "Phi-Theta-Psi CHECK"
        else:
            print "NO Angles"
    else:
        print "No Data Received from Qualisys!"
        time.sleep(2)
        sys.exit("No Data Received from Qualisys!")
 
	time.sleep(0.05)
	#Send roll and pitch
	sock.send(info)  
	#receive
	print "Waiting for quadcopter signal!"
	data = sock.recv(1024)
	if(data == "r"):
		start = True
    #Receive Data
   	print "Starting given true - true: " + active + " - " + start
    lastTime = time.time()*1000
    while (active and start):
       
        '''
       Balanced and stay within a given 30x30x30cm box###
       Quadcopter should be zero in theta and phi angle.
       When drifting outside the box it should slightly adjust its angle
       in order to get back inside of the box.
       Give more thrust if bellow z-axis, and less if above.
       The magnitude of the correction should be decided upon the rapidness of the change.
       Do we care about psi?
       '''
 
        #Read qual data
        qt.getAttitude()
        currentX = qt.getBody(0)['linear_x']
        currentY = qt.getBody(0)['linear_y']
        currentZ = qt.getBody(0)['linear_z']
        angularX = qt.getBody(0)['angular_x']
        angularY = qt.getBody(0)['angular_y']
        angularZ = qt.getBody(0)['angular_z']
 
        #If we lose qualisys data, then land
        if(math.isnan(currentX) or math.isnan(currentY) or math.isnan(currentZ)):
            print "MISSING INFORMATION!"
            lostContact += 1
            currentX = lastX
            currentY = lastY
            currentZ = lastZ
            if (lostContact >= 5):
                # Landing Function ??
                roll = 0
                pitch = 0
                thrust = 0
                commands = [str(roll), str(pitch), str(thrust), star]
                commandString = ','.join(commands)
                sock.send(commandString)
                print "LOST CONTACT WITH QUALISYS"
                active = False
        else:
        	if(timer > 30):
                # Landing Function ??
                roll = 0
                pitch = 0
                thrust = 0
                commands = [str(roll), str(pitch), str(thrust), star]
                commandString = ','.join(commands)
                sock.send(commandString)
                print "TOO HIGH"
                active = False

            lostContact = 0
 			if (currentZ > 2250 or currentZ < 400):
                # Landing Function ??
                roll = 0
                pitch = 0
                thrust = 0
                commands = [str(roll), str(pitch), str(thrust), star]
                commandString = ','.join(commands)
                sock.send(commandString)
                print "TOO HIGH"
                active = False
 
        #For Z-axis: adjust thrust and pitch/roll angle?
        #Compute
        #Find velocity (DO IT FOR EACH 100ms?) <------- NOT FINISHED
       	
       	currentTime = time.time()*1000
        Velocity_x = (currentX - lastX)/(currentTime - lastTime) #mm/s = 1000
        Velocity_y = (currentY - lastY)/(currentTime - lastTime)
        velocity_z = (currentZ - lastZ)/(currentTime - lastTime)
        lastTime = currentTime 
 
        #Save new last position
        lastX = currentX
        lastY = currentY
        lastZ = currentZ
 
        #Find distance
        errorX = desiredX - currentX #mm = 2000
        errorY = desiredY - currentY
        errorZ = desiredZ - currentZ
 
        #Error distance *Kp gives a desiredSpeed, should be small to start with
        desiredSpeedX = errorX * kp_X #2000*0,2 = 400 mm/s
        desiredSpeedY = errorY * kp_Y
 
 		#Calculate ErrorSpeed
        errorSpeed_X = desiredSpeedX - Velocity_x #400-1000 = -600 mm/s
        errorSpeed_Y = desiredSpeedY - Velocity_y
       
        #Calculate some force
        dRoll_X = errorSpeed_X * kp2_X #-600*0,3 = -180 deg/s
        dPitch_Y = errorSpeed_Y * kp2_Y
 
        #Set roll, pitch, thrust to some values
        roll = dRoll_X
        pitch = dPitch_Y
        #thrustX = thrust/cos(angularX)
        #thrustY = thrust/cos(angularY)
        thrust = 1390 + errorZ*kp_Z # + (thrustX+thrustY)/2

        #LIMIT
        if(roll > maxRoll):
        	roll = maxRoll
        elif(roll < minRoll):
        	roll = minRoll

        if(pitch > maxPitch):
        	pitch = maxPitch
        elif(pitch < minPitch):
        	pitch = minPitch

       	if(thrust > maxThrust):
       		thrust = maxThrust
       	elif(thrust < minThrust):
       		thrust = minThrust 
 
        #Send roll, pitch, throttle
        print "Sending data: " + roll + " " + pitch + " " + thrust 
        commands = [str(roll), str(pitch), str(thrust), star]
        commandString = ','.join(commands)
        sock.send(commandString)
 
else:
    print"quadcopter is not ready"
    time.sleep(2)

if __name__ == 'main':
	Main()