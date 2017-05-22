#Recieve
import qtmLocal as qtm
import bluetooth
import time
import sys

#####################################
############# BLUETOOTH #############
#####################################

bd_address = "20:15:08:13:78:88"
port = 1
sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((bd_address, port))
   
def sendBD(var_X, var_Y, var_Z):
  sock.send(var_X, var_Y, var_Z)
  sock.send(var_Y)
  sock.send(var_Z)

def recieveBD():
  data = sock.recv(1024)

def endBlueTooth(*args):
sock.close()


#####################################
############# QUALISYS! #############
#####################################

def initQual(*args):
  with qtm.QTMClient() as qt:
      qt.setup()
      qt.getAttitude()

      currentX = qt.getBody(0)['linear_x']
      currentY = qt.getBody(0)['linear_y']
      currentZ = qt.getBody(0)['linear_z']
      angularX = qt.getBody(0)['angular_x']
      angularY = qt.getBody(0)['angular_y']
      angularZ = qt.getBody(0)['angular_z']
