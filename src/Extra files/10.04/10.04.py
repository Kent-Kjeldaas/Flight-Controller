import qtmLocal as qtm
import bluetooth
import time
import sys
import math
import logging
from threading import Thread

# Logging terms specified by Python documentation:
# DEBUG:    Detailed information, typically of interest only when diagnosing problems.
# INFO:     Confirmation that things are working as expected.
# WARNING:  An indication that something unexpected happened, or indicative of some problem in the near future (e.g. ‘disk space low’). The software is still working as expected.
# ERROR:    Due to a more serious problem, the software has not been able to perform some function.
# CRITICAL: A serious error, indicating that the program itself may be unable to continue running.

logging.basicConfig(file='test.log', level=logging.DEBUG, format='%(asctime)s:%(levelname)s:%(message)s')

#SETTINGS:
kp_X = 0.2 #Limit desired velocity
kp_Y = 0.2 #Limit desired velocity
kp_Z = 0.06 #Limit desired velocity
kp2_X = 0.3 #Tune roll
kp2_Y = 0.3 #Tune pitch
desiredX = 0.0
desiredY = 0.0
desiredZ = 750
pitch = 0.0
minPitch = -5
maxPitch = 5
roll = 0.0
minRoll = -5
maxRoll = 5
thrust = 1390
maxThrust = 1590
minThrust = 1190
LimZ_max = 2000 #200cm
LimZ_min = 350 #35cm
LimX_max = 1000
LimX_min = -1000
LimY_max = 1000
LimY_min = -1000

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

#Float position and rotation
currentX = .0
currentY = .0
currentZ = .0
angularX = .0
angularY = .0
angularZ = .0

#Check connection with Qualisys:

def UpdateQualInfo():
    global currentX
    global currentY
    global currentZ
    global angularX
    global angularY
    global angularZ
	with qtm.QTMClient() as qt:
	    qt.setup()
	    time.sleep(3)
	    debug.info("Updating Info")
            while(True):
                qt.getAttitude()
                currentX = qt.getBody(0)['linear_x']
                currentY = qt.getBody(0)['linear_y']
                currentZ = qt.getBody(0)['linear_z']
                angularX = qt.getBody(0)['angular_x']
                angularY = qt.getBody(0)['angular_y']
                angularZ = qt.getBody(0)['angular_z']
                logging.debug("x, y, z, phi, theta, psi:" + linear_x + " " + linear_y + " " + linear_z + " " + angular_x + " " + angular_y + " " + angular_z)

def Compute():
    global currentX
    global currentY
    global currentZ
    global angularX
    global angularY
    global angularZ
    global lostContact
    global LimZ_max #might not need to be global?
    global LimZ_min
    global LimX_max
    global LimX_min
    global LimY_max
    global LimY_min
    lastX = currentX
    lastY = currentY
    lastZ = currentZ

    #sufficent for now, need to change later..
    desiredX = currentX
    desiredY = currentY

    if(not math.isnan(currentX) and not math.isnan(currentY) and not math.isnan(currentZ)): # if value is in scope (might need to drop isnan because of unknown bug)
        logging.debug("X-Y-Z CHECK")
        active = True
        #ready for flight
        if(not math.isnan(angularX) and not math.isnan(angularY) and not math.isnan(angularZ)):
            logging.debug("Phi-Theta-Psi CHECK")
        else:
            logging.warning("NO Angles")
    else:
        active = False
        logging.critical("No Data Received from Qualisys!")
        time.sleep(2)
        sys.exit("No Data Received from Qualisys!")
 
    time.sleep(0.05)
    #Send roll and pitch
    sock.send(info)
    #receive
    logging.debug("Waiting for quadcopter signal!")
    data = sock.recv(1024)
    if(data == "r"):
        start = True
    #Receive Data
    logging.debug("Starting given true - true")
    lastTime = time.time()*1000
    timer = time.time()
    try:
        while (active and start)
            '''
            #Read qual data
            qt.getAttitude()
            currentX = qt.getBody(0)['linear_x']
            currentY = qt.getBody(0)['linear_y']
            currentZ = qt.getBody(0)['linear_z']
            angularX = qt.getBody(0)['angular_x']
            angularY = qt.getBody(0)['angular_y']
            angularZ = qt.getBody(0)['angular_z']

            '''
            #If we lose qualisys data, then land
            if(math.isnan(currentX) or math.isnan(currentY) or math.isnan(currentZ)):
                logging.debug("MISSING INFORMATION!")
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
                    logging.warning("LOST CONTACT WITH QUALISYS")
                    active = False
                    break
            else:
                lostContact = 0
                # Stop if time is greater than thirty seconds (Not really needed? Should be done on Quad?)
                if (timer > time.time()+30):
                    # Landing Function ??
                    roll = 0
                    pitch = 0
                    thrust = 0
                    commands = [str(roll), str(pitch), str(thrust), star]
                    commandString = ','.join(commands)
                    sock.send(commandString)
                    logging.warning("Time Out")
                    active = False
                    break
                # check if out of scope
                if (currentZ > LimZ_max or currentZ < -LimZ_min or currentX > LimX_max or currentX < LimX_min or currentY > LimY_max or currentY < LimY_min): 
                    # Landing Function ??
                    roll = 0
                    pitch = 0
                    thrust = 0
                    commands = [str(roll), str(pitch), str(thrust), star]
                    commandString = ','.join(commands)
                    sock.send(commandString)
                    logging.warning("Out Of Scope")
                    active = False
                    break
     
            #For Z-axis: adjust thrust and pitch/roll angle?
            #Compute
            #Find velocity (DO IT FOR EACH 100ms?) <------- NOT FINISHED
            
            currentTime = time.time()*1000
            logging.debug('Time current: ' + currentTime) #debug
            logging.debug('Time last: ' + lastTime) #debug
            if(currentTime - lastTime != 0):
                    Velocity_x = (currentX - lastX)/(currentTime - lastTime) #mm/s = 1000
                    Velocity_y = (currentY - lastY)/(currentTime - lastTime)
                    velocity_z = (currentZ - lastZ)/(currentTime - lastTime)
            else: # Might need to just not update velocity
                    Velocity_x = 0
                    Velocity_y = 0
                    Velocity_z = 0
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
            roll = 0
            pitch = 0
            if(thrust > 0 and thrust < 2000):
               thrust = int(thrust)
               logging.debug("Sending data: " + roll + " " + pitch + " " + thrust
               commands = [str(roll), str(pitch), str(thrust), star]
               commandString = ','.join(commands)
               sock.send(commandString)
               time.sleep(0.3)

        else:
            logging.debug("quadcopter is not ready")
            time.sleep(2)
                
    except (IndexError) as e:
        roll = 0
        pitch = 0
        thrust = int(thrust)
        logging.warning("Sending data: " + roll + " " + pitch + " " + thrust
        commands = [str(roll), str(pitch), str(thrust), star]
        commandString = ','.join(commands)
        sock.send(commandString)
        logging.warning("EXCEPTION: TERMINATE")
        time.sleep(0.3)
    

logging.debug("Starting Qual Update Function")

t1 = Thread(target=UpdateQualInfo)
t1.start()

time.sleep(3)

logging.debug("Starting Loop")
t2 = Thread(target=Compute)
t2.start()