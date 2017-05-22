#Trajectory Planning

def Take_Off( parameters ):
	pass

def Land( parameters ):
	pass

def Stay( parameters ):
  '''
   ###Balanced and stay within a given 30x30x30cm box###
   
   Quadcopter should be zero in theta and phi angle.
   When drifting outside the box it should slightly adjust its angle
   in order to get back inside of the box. 
   Give more thrust if bellow z-axis, and less if above. 
   The magnitude of the correction should be decided upon the rapidness of the change.
   Do we care about psi? 
  '''
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
	    Velocity_x = currentX - lastX
	    Velocity_y = currentY - lastY
	    velocity_z = currentZ - lastZ

	    #Save position
	    lastX = currentX
	    lastY = currentY
	    lastZ = currentZ

	    #Find distance
	    errorX = desiredX - currentX
	    errorY = desiredY - currentY
	    errorZ = desiredZ - currentZ

	    #STAY / margin of error
	    if(errorX < 30){
	    	#stabilize for roll
	    	#pay attention to speed
	    }
	    if(errorY < 30){
	    	#stabilize for pitch
	    	#pay attention to speed
	    }
	    if(errorZ < 30){
	    	#stabilize for yaw
	    	#pay attention to speed
	    }

	    Speed_X = errorX * kp_X
	    Speed_Y = errorY * kp_Y

	    if(Speed_X >= 0){
	      errorSpeed_X = Speed_X - currentX #wrong
	    }
	    else{
	      errorSpeed_X = Speed_X + currentX
	    }
	    if(Speed_Y >= 0){
	      errorSpeed_Y = Speed_Y - currentY
	    }
	    else{
	      errorSpeed_Y = Speed_Y + currentY   
	    }

	    #calculate some force
	    dKraft_X = errorSpeed_X * kp2_X
	    dKraft_Y = errorSpeed_Y * kp2_Y
	    #find a corralation between force and angle
	    phi = thrust + dKraft_X
	    theta = thrust - dKraft_Y
	 
	    #Send roll and pitch
	    motor1.writeMicroseconds(m1)
	    motor2.writeMicroseconds(m2)


def Ascend( parameters ):
	pass

def Descend( parameters ): 
	pass

def Travel_To_Point(x_cord, y_cord, z_cord):
	pass

def Travel(vector):
	pass

def Pitch( parameters ):
	pass

def Roll( parameters ):
	pass

def Yaw( parameters ):
	pass

def Plot_Route( parameters ):
	pass