import bluetooth
import time
import sys
 
bd_address = "20:15:08:13:78:88"
port = 1
sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((bd_address, port))
 
#def initBlueTooth(*args):
 
def sendBD(var):
  #SEND:
  sock.send(var)  
 
def recieveBD(*args):
  #RECIEVE:
    data = sock.recv(1024)
 
def endBlueTooth(*args):
    sock.close()
 
print "Connected"
while True:
    var = raw_input("Skriv microseconds: ")
    sendBD(var)
    #print "Sent"
    recieveBD()
    #print "Recieved data from arduino"
 
 
 
    #Super simple data:
    Roll = int(round((errorY * pRoll) + (velY * dRoll)))
    Pitch = int(round((errorX * pPitch) + (velX * dPitch)))
    Throttle = int(round(errorZ * pThrottle))
    Heading = int(round(errorHeading * pHeading))
 
    ## Her er hvordan vi KAN sende alt som en string:
    commands = [chr(sigRoll), chr(sigPitch), chr(sigThrottle), chr(sigHeading)]
    commandString = ''.join(commands)
    sock.send(commandString)
 
endBlueTooth()