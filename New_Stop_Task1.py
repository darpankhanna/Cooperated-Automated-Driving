from Quanser.q_essential import LIDAR
from Quanser.product_QCar import QCar
from Quanser.q_ui import gamepadViaTarget
from Quanser.q_misc import Calculus
from Quanser.q_interpretation import basic_speed_estimation
import os
import time
import struct
import numpy as np 
import matplotlib.pyplot as plt
import math
plt.pause(15)
#############################################
myCar = QCar()
LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])
mtr_cmd = np.array([0,0])
current, batteryVoltage, encoderCounts = myCar.read_write_std(mtr_cmd, LEDs)
current_velocity = 0
#############################################

#############################################

min_distance_between_object_and_car = 0.07 # to be modified as per needs
distance_between_lidar_and_front_of_car = 0.02 # to be modified as per needs
constant_vehicle_velocity = 0.04 # Can be modified as per needs

#############################################

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
## Timing Parameters and methods 
startTime = time.time()
def elapsed_time():
	return time.time() - startTime

sampleRate = 10
sampleTime = 1/sampleRate
simulationTime = 80.0
print('Sample Time: ', sampleTime)

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
## Additional parameters and buffers
counter = 0


#################################################
# Car Acceleration/Deceleration function Starts #
#################################################

def accelerate(current_velocity, final_velocity, LEDs,lidar_object_distance = 0.05): # 0.05 is default acceleration Distance in which car would attain final velocity
	print(current_velocity, "#",final_velocity, "#",LEDs,"#",lidar_object_distance)
	mtr_cmd = np.array([current_velocity,0])
	current, batteryVoltage, encoderCounts = myCar.read_write_std(mtr_cmd, LEDs)
	while current_velocity != final_velocity:
		acceleration = (((final_velocity**2) - (current_velocity**2)) / (2 * ( lidar_object_distance - min_distance_between_object_and_car - distance_between_lidar_and_front_of_car )))
		current_velocity = current_velocity + acceleration
		vdiff = final_velocity-current_velocity
		if vdiff < 0: vdiff*=-1
		if final_velocity == 0 and (current_velocity < 0 or  vdiff < 0.04): 
			current_velocity = 0
		elif final_velocity != 0 and (current_velocity > final_velocity or vdiff < 0.04):
			current_velocity = final_velocity
		mtr_cmd = np.array([current_velocity,0])
		print(">> ",current_velocity, "#",final_velocity)
		current, batteryVoltage, encoderCounts = myCar.read_write_std(mtr_cmd, LEDs)
		time.sleep(1)
	return {"current":current, "batteryVoltage": batteryVoltage, "encoderCounts": encoderCounts}
#################################################
#  Car Acceleration/Deceleration function Stop  #
#################################################




# LIDAR initialization and measurement buffers
myLidar = LIDAR(num_measurements=720)
file1 = open("Lidar_Angle.txt", "w") 
# polar plot object for displaying LIDAR data later on
ax = plt.subplot(111, projection='polar')
plt.show(block=False)

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
## Main Loop
try:
	while elapsed_time() < simulationTime:
		# Clear existing polar plot figure
		plt.cla()

		# Start timing this iteration
		start = time.time()

		# Capture LIDAR data
		myLidar.read()

		# print(myLidar.angles)
		# Plot the LIDAR data on the plot every 1 second
		if counter % 10 == 1:
			ax.scatter(myLidar.angles, myLidar.distances, marker='.')
			ax.set_theta_zero_location("W")
			# The set_theta_direction(-1) is set just for the plotting
			ax.set_theta_direction(-1)
			#file1.write(str(myLidar.angles) + "\n" + str(myLidar.distances) + "\n\n")
			plt.pause(0.00001)
			mtr_cmd = np.array([constant_vehicle_velocity,0])
			current_velocity = constant_vehicle_velocity
			current, batteryVoltage, encoderCounts = myCar.read_write_std(mtr_cmd, LEDs)
			flag = 0

		# Main loop for break system  
		for i in myLidar.angles:
			# it checks for the angle between 70 to 110 degree (the angle is converted into pi format)
			if i>=1.22 and i<=1.92:
				ai = np.where(myLidar.angles == i)
				lidar_object_distance = myLidar.distances[ai[0][0]]
				#after getting the distance of the specific angle check if the angle is less than 1m
				if lidar_object_distance < 1 and lidar_object_distance > 0.05:
					print("\n\nApply Break     ", i, lidar_object_distance)
					LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
					#mtr_cmd sets the velocity and the steering angle so 0,0 specifies it to stay still
					stopping_final_velocity = 0
					res = accelerate(current_velocity, stopping_final_velocity, LEDs, lidar_object_distance)
					current_velocity = stopping_final_velocity
					current = res["current"]
					batteryVoltage = res["batteryVoltage"]
					encoderCounts = res["encoderCounts"]
					flag = 1
					break
				else:
					flag = 0
			else:
				flag = 0

		

		# End timing this iteration
		end = time.time()

		# Calculate the computation time, and the time that the thread should pause/sleep for
		computationTime = end - start
		sleepTime = sampleTime - ( computationTime % sampleTime )
		# Pause/sleep and print out the current timestamp
		time.sleep(sleepTime)
		counter += 1
		
		print('Simulation Timestamp :', elapsed_time(), ' s.')
	
	LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
	mtr_cmd = np.array([0,0])
	current, batteryVoltage, encoderCounts = myCar.read_write_std(mtr_cmd, LEDs)
	print("User interrupted!")

except KeyboardInterrupt:
	LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
	mtr_cmd = np.array([0,0])
	current, batteryVoltage, encoderCounts = myCar.read_write_std(mtr_cmd, LEDs)
	print("User interrupted!")

finally:

	# Close the plot figure
	plt.close()    
	file1.close()
	# Terminate the LIDAR object
	myLidar.terminate()
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
