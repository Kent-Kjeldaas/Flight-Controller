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
Recieve data
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
Recieve data from Qual

Each 20ms: 
Setpoint for roll, pitch and yaw
Recieve data from IMU
PID Compute
Update motor output
'''

import qtmLocal as qtm
import Trajectory_Plannning
from recieve import *

bool Stay = False
bool Take_Off = False
bool Land = False

def Plot_Route():
	Stay = True

#Check connection with Qualisys:
with qtm.QTMClient() as qt:
    qt.setup()
    qt.getAttitude()

    currentX = qt.getBody(0)['linear_x']
    currentY = qt.getBody(0)['linear_y']
    currentZ = qt.getBody(0)['linear_z']
    angularX = qt.getBody(0)['angular_x']
    angularY = qt.getBody(0)['angular_y']
    angularZ = qt.getBody(0)['angular_z']

#Check if error
	#display if nan
#Log_data at some time
	#save to file?
#Recieve Data
    qt.getAttitude()

    currentX = qt.getBody(0)['linear_x']
    currentY = qt.getBody(0)['linear_y']
    currentZ = qt.getBody(0)['linear_z']
    angularX = qt.getBody(0)['angular_x']
    angularY = qt.getBody(0)['angular_y']
    angularZ = qt.getBody(0)['angular_z']

#Check Mode & Compute
if (Stay){
	Trajectory_Plannning.Stay()
}
else if(Take_Off) {
	Trajectory_Plannning.Take_Off()
}
else if(Land) {
	Trajectory_Plannning.Land()
}

#Send data
recieve.sendBD("info")
recieve.recieveBD()