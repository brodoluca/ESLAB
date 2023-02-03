#NB_MainScenarioRover.py

# Copyright 2020 Jaime Burbano - jaime.burbanovillavicencio@fh-dortmund.de
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.



#Last Update: 21-10-2020

#Description: Executes the system to get the imiage from the camera, detect the borders
#rectify the ROI (area within the borders), read the scenario image and overlay the 
#obstacles and targets defined there in the real scenario (as a graph).



#Note: This script requires as input the image that describes the scenario. 
#You have to design an image describing the 
#position of the obstacles and the targets, which will be mapped to the image gotten from the camera. 
#The image must be created based on the provided template.
#In the image scenario, rectangles will be interpreted as obstacles and circles as targets and triangles as starting point. 

#ARGUMENTS:
#
# -i / --image  --> the scenario image to upload
# -id           --> the id of the marker on your crawler
# -c / --aws    --> if the application connects or not to aws [True/False] default:False
# -m / --mode   --> the game mode ["bottom"/"top"/"mixed"] default:bottom

#Note: you can exit the program pressing "q" when a window is active.

import numpy as np
import cv2
import time 
import argparse
import transform as tr
import get_xaxis_image_points_opt as marker
import os
import drawScenario
import shapedetector
import configureSystem
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
import json
import  AWSIoTPythonSDK.MQTTLib as p

import PathFinder

DRAW_OBS = True

FIELD_METER = 1.30

ROOM_OVER_OBSTACLE = 30

HEIGHT_OBSTACLE = 90.0
WIDTH_OBSTACLE = 42
DEBUG = True
OPTIMIZE = False
#*** SYSTEM CONFIGURATION
#-----------------------------------------------------------------
myScenarioConfig=configureSystem.configureScenario() #use this object to apply the configurations 
													 #of the system in the configureSystem.py 
											 
myAWSConfig=configureSystem.configureAWS()													 
host = myAWSConfig.get_aws_host()
rootCAPath = myAWSConfig.get_root_file()
certificatePath = myAWSConfig.get_cert_file()
privateKeyPath = myAWSConfig.get_priv_file()
clientId = myAWSConfig.get_thing_name()
topic_rover = myAWSConfig.get_topic("rover")
topic_targer = myAWSConfig.get_topic("target")
useWebsocket=myAWSConfig.useWebsocket
def __get_ordered_list(points, x, y, reverse = False):
        points.sort(key = lambda p: (p[1] - x)**2 + (p[0] - y)**2, reverse = reverse)
        return points										 

def connect_aws():

	if useWebsocket and certificatePath and privateKeyPath:
		print("X.509 cert authentication and WebSocket are mutual exclusive. Please pick one.")
		exit(2)

	if not useWebsocket and (not certificatePath or not privateKeyPath):
		print("Missing credentials for authentication.")
		exit(2)

# Port defaults
	if useWebsocket:  # When no port override for WebSocket, default to 443
		port = 443
	if not useWebsocket:  # When no port override for non-WebSocket, default to 8883
		port = 8883	
	
	
	myAWSIoTMQTTClient = None
	if useWebsocket:
		myAWSIoTMQTTClient = AWSIoTMQTTClient(clientId, useWebsocket=True)
		myAWSIoTMQTTClient.configureEndpoint(host, port)
		myAWSIoTMQTTClient.configureCredentials(rootCAPath)
	else:
		myAWSIoTMQTTClient = AWSIoTMQTTClient(clientId)
		myAWSIoTMQTTClient.configureEndpoint(host, port)
		myAWSIoTMQTTClient.configureCredentials(rootCAPath, privateKeyPath, certificatePath)

	# AWSIoTMQTTClient connection configuration
	#myAWSIoTMQTTClient.configureOfflinePublishQueueing(20, p.DROP_OLDEST)
	myAWSIoTMQTTClient.configureAutoReconnectBackoffTime(1, 32, 20)
	myAWSIoTMQTTClient.configureOfflinePublishQueueing(-1)  # Infinite offline Publish queueing
	myAWSIoTMQTTClient.configureDrainingFrequency(2)  # Draining: 2 Hz
	myAWSIoTMQTTClient.configureConnectDisconnectTimeout(10)  # 10 sec
	myAWSIoTMQTTClient.configureMQTTOperationTimeout(5)  # 5 sec

# Connect and subscribe to AWS IoT
	myAWSIoTMQTTClient.connect()
	return myAWSIoTMQTTClient
					 
													 
def capture_image():
	ret, img = cap.read() #get the frame from the camera
	resize_ratio=1
	#if the received image from the camera is bigger, then resize it
	if max(img.shape) > IMAGE_MAX_WH:
		resize_ratio = float(IMAGE_MAX_WH) / max(img.shape[0], img.shape[1])
		img = cv2.resize(img, (0, 0),fx=resize_ratio,fy=resize_ratio,interpolation=cv2.INTER_AREA)
	return img, resize_ratio

	
if __name__ == "__main__":
	#Requires the image of the scenario as input
	ap = argparse.ArgumentParser()
	ap.add_argument("-i", "--image", required=True,
		help="path to the input image")
	ap.add_argument("-c", "--aws", default=False,
		help="connect to aws?")
	ap.add_argument("-id", required=True,
		help="rover id")		
	ap.add_argument("-m", "--mode", default="bottom",
		help="game mode: bottom, top, mixed")		
		
		
	args = vars(ap.parse_args())
	image_scenario = cv2.imread(args["image"])
	connect_to_aws = args["aws"]
	rover_id = args["id"]
	game_mode = args["mode"]	

	
	sd = shapedetector.ShapeDetector() #creates an object to detect the forms existent in the image scenario	
			   
	#Init variables:

	orientations,image_scenario, obstacles_scenario, targets_scenario, start_point=sd.get_shapes_coordinates(image_scenario)

	########################### Creates a blank maze from the templates
	dict_maze = {}
     #let's create a blank maze
	end_points = []
	maze = np.zeros((image_scenario.shape[0],image_scenario.shape[1]))
	for start in start_point:
        #inverted because numpy
		start_point_no_list = start[::-1]
    #print(o)
	for (orientation,obstacle) in zip(orientations,obstacles_scenario):
        #we assign 1 because that's our symbol for obstacle
		maze[obstacle[::-1]] = 1
        
		if(orientation==1): # if horizontal
			for i in range(int(obstacle[1] -8 - WIDTH_OBSTACLE/2 - ROOM_OVER_OBSTACLE),int(obstacle[1] -8 + WIDTH_OBSTACLE/2 + ROOM_OVER_OBSTACLE)):
				for j in range(int(obstacle[0]+(8)-HEIGHT_OBSTACLE/2 - ROOM_OVER_OBSTACLE),int(obstacle[0]+8+HEIGHT_OBSTACLE/2 + ROOM_OVER_OBSTACLE)):
					maze[i][j]=1 
		else:
			for i in range(int(obstacle[1] -8 - HEIGHT_OBSTACLE/2 - ROOM_OVER_OBSTACLE),int(obstacle[1] -8 + HEIGHT_OBSTACLE/2 + ROOM_OVER_OBSTACLE)):
				for j in range(int(obstacle[0]+(8)-WIDTH_OBSTACLE/2 - ROOM_OVER_OBSTACLE),int(obstacle[0]+8+WIDTH_OBSTACLE/2 + ROOM_OVER_OBSTACLE)):
					maze[i][j]=1  
    
	for target in targets_scenario:
		end_points.append(target[::-1])
	dict_maze[1] = maze
	dict_maze[2] = start_point_no_list
	dict_maze[3] = end_points
	dict_maze[4] = obstacles_scenario
	dict_maze[5] = orientations

	solver = PathFinder.PathFinder(dict_maze[1].tolist(),dict_maze[2],dict_maze[3],dict_maze[4])
	outcomes, db_points = solver.determine_rover_commands()
	
	#Do we want to optimaze the path?
	#if(OPTIMIZE):
#		p=solver.optimize_path(dict_maze, db_points.copy(), solver.start,solver.end_points.copy())
	#	new_outcomes = solver.optimize_commands(p)

	if(DEBUG):
		image2 = image_scenario.copy()
		#for outcome in outcomes:
		#	print(outcome)
		start = solver.start
		for path in db_points:
			for point in db_points[path]:
				image2 = cv2.line(image2, start[::-1],point[::-1], (0, 0, 255), 4)
				start = point
		#if(OPTIMIZE):
		#	image3 = image_scenario.copy()
		#	#for outcome in new_outcomes:
		#	print(new_outcomes)
		#	start = solver.start
		#	for path in p:
		#		for point in p[path]:
		#			image3 = cv2.line(image3, start[::-1],point[::-1], (0, 0, 255), 4)
		#			start = point
			
		cv2.imshow("Test",image2)
		#cv2.imshow("Optimized path",image3)
		cv2.waitKey(0)
		cv2.destroyAllWindows()

	cv2.imshow("Image_scenario", image_scenario)
	size_image_virtual_scenario=tuple(image_scenario.shape[1::-1])
	print("image_virtual_scenario_size: ",size_image_virtual_scenario)	
	print ("detected obstacles in scenario: ", obstacles_scenario)
	print ("detected targets in scenario: ", targets_scenario)
	print ("rover starts in: ", start_point)	

	#::::::::::::::::::::::check aws connection::::::::::::::::
	if connect_to_aws:
		myAWSIoTMQTTClient=connect_aws()
		mes_rover = json.dumps({'directions': '{}'.format(outcomes)})		
		myAWSIoTMQTTClient.publish(topic_rover, mes_rover, 1)	
		
	myCameraConfig=configureSystem.configureCamera() #Change to "1" to get the USB camera in configure.System.py

	cap = cv2.VideoCapture(myCameraConfig.get_resource()) #select resource 1 (webcam) --> 0 integrated cam in laptop
	cap.set(cv2.CAP_PROP_FRAME_WIDTH, myCameraConfig.get_camera_width()) #this value could change if using another webcam
	cap.set(cv2.CAP_PROP_FRAME_HEIGHT, myCameraConfig.get_camera_height()) #this value could change if using another webcam
	time.sleep(1) #wait until webcam is ready

	IMAGE_MAX_WH=myScenarioConfig. get_scenario_image_size()#declare the maximum pixels admited (<=1080)
	resize_ratio=1	#initializes the variable

	myBorders=drawScenario.Borders() #creates an object of the class Borders
	myObstacles=drawScenario.Obstacles() #creates an object of the class Obstacles
	myTargets= drawScenario.Targets() #creates an object of the class Targets
	myStart= drawScenario.StartPoint() #creates an object of the class StartPoint	
	border_list_default=[(20,10),(120,10),(120,120),(20,120)]	#default value for the border coordinates (just in case)
	current_target_coordinate=None #Init the target coordinate
	#print(targets_scenario)
	#targets_scenario = __get_ordered_list(targets_scenario,start_point[0][1],start_point[0][0],True)
	myTargets.set_target_list(targets_scenario,game_mode) #formats the target list to determine which one will be shown
	#print(targets_scenario)
	border_list=[] #init the variable where we will put the points of the border markers
	# iterate until we find all the border markers
	while len(border_list)!= len(myScenarioConfig.get_border_ids()):
		border_list.clear() #initializes the variable where we will put the points of the border markers
		img, resize_ratio=capture_image() #gets a frame from the camera
		#size_image_real_scenario=tuple(img.shape[1::-1]) #comment
		#print("image_real_scenario_size: ",size_image_real_scenario) #comment
		my_markers=(marker.get_markers_info(img)) #get markers info (coordinates and angles)			
		#cv2.imshow("Original", img) #shows the original image with the detected markers as a reference
		if my_markers:

			myBorders.draw_borders (img, my_markers, False) #draws the borders 
														    #-->False means we dont use real time border recognition
			borders_coordinates=myBorders.get_borders() #returns the coordinates of the border markers
			#print (borders_coordinates)
			for coordinate_id in borders_coordinates:
				border_list.append(borders_coordinates[coordinate_id]) #appends only the points into a list to be sent to the 
			
			pts = np.array(border_list_default, dtype = "float32") #just in case
			print ("not all borders detected:",my_markers)
			
		time.sleep(0.2)
	start_time=time.time()
#once we detected the working area, display the scenario and start game	
	continue_game=True
	while(True):
		start=time.time()
		#rectification method
		pts = np.array(border_list, dtype = "float32") #creates an array with the border points gotten before

		img, _=capture_image() #gets a clean capture of the scene (without border markers recognition)
		warped = tr.four_point_transform(img, pts) #cuts the ROI (region of interest) delimited by the
													   #four markers and rectifies it.
		
		size_image_real_scenario=tuple(warped.shape[1::-1]) #gets the size in pixels of the rectified ROI
			
		#Resizes the rectified ROI to form a square equal to the scenario image
		resize_ratio_x=size_image_virtual_scenario[0]/size_image_real_scenario[0] 
		resize_ratio_y=size_image_virtual_scenario[1]/size_image_real_scenario[1] 
		warped = cv2.resize(warped, (0, 0),fx=resize_ratio_x,fy=resize_ratio_y,interpolation=cv2.INTER_AREA)
				
		my_rover_coordinates=(marker.get_markers_info(warped)) #detects the markers in the rectified ROI image 
		myStart.draw_start(warped,start_point[0])	#draw the starting point for the rover	
		#iterates through the coordinates of the recognized obstacles in the image scenario
		if(DRAW_OBS):
			for obstacle_coord in obstacles_scenario:
				warped=myObstacles.draw_obstacles(warped, obstacle_coord) #draws the obstacle from the image scenario
																			#in its equivalent position in the rectified ROI 																
		if targets_scenario:
			warped,target_list, current_target_coordinate =myTargets.draw_current_target(warped) #draws the obstacle from the image scenario
			targets_scenario = __get_ordered_list(targets_scenario,start_point[0][1],start_point[0][0],True)	
			if my_rover_coordinates:
				try:
					continue_game=myTargets.check_caught(target_list, my_rover_coordinates[int(rover_id)][0]) 
				except:
					print ("no rover_marker in image")														
						
				#print("Target:", current_target_coordinate," my rover: ", my_rover_coordinates) #prints {"markerID":[(x coord,y coord), angle]} of the rover in the rectified ROI image		
		#start = solver.start
		#for path in db_points:q
		#	for point in db_points[path]:
		#		image2 = cv2.line(warped, start[::-1],point[::-1], (0, 0, 255), 4)
		#		start = point
		new_t = []
		for target in target_list:
			if(target[1]):
				new_t.append(target[0][::-1])
		print( my_rover_coordinates)
		
		final_path = []
		if(True):
			try:	
				dict_maze[1] = maze
				dict_maze[2] = my_rover_coordinates[int(rover_id)][0][::-1]
				dict_maze[3] = new_t
				dict_maze[4] = obstacles_scenario
				dict_maze[5] = orientations
				solver = PathFinder.PathFinder(maze.tolist(),dict_maze[2],dict_maze[3].copy(),dict_maze[4])
				
				outcomes, db_points = solver.determine_rover_commands()
				print("uff")
			
				if(OPTIMIZE and (my_rover_coordinates[int(rover_id)][0][0] > 30 or my_rover_coordinates[int(rover_id)][0][0] < 400)) and ((my_rover_coordinates[int(rover_id)][0][1] > 30 or my_rover_coordinates[int(rover_id)][0][1] < 400)):
					db_points=solver.optimize_path(dict_maze, db_points.copy(), dict_maze[2],dict_maze[3])
					outcomes = solver.optimize_commands(db_points)
				start = my_rover_coordinates[int(rover_id)][0][::-1]
				for path in db_points:
					for point in db_points[path]:
						image2 = cv2.line(warped, start[::-1],point[::-1], (0, 0, 0), 4)
						start = point
				angle = 270-my_rover_coordinates[int(rover_id)][1]
				if(angle <0):
					angle = 360 + angle
				final_path = [(0,angle)]
				final_path += outcomes
				#print(p)
				print(final_path)
			except Exception as e:
				print(e)
				#mes_rover = json.dumps({'outcome': '{}'.format([(0,0),(0,0)])})
				#myAWSIoTMQTTClient.publish(topic_rover, mes_rover, 1)
				pass

		cv2.imshow("Rectified", warped) #uncomment
		end=time.time()
		if(end-start_time >0.4):
			if connect_to_aws:
				start_time=end;
				#myAWSIoTMQTTClient.publish(topic_rover, "test", 1)
				mes_rover = json.dumps({'outcome': '{}'.format(final_path)})		
				myAWSIoTMQTTClient.publish(topic_rover, mes_rover, 1)

				
			#mes_target = json.dumps({'target': '{}'.format(current_target_coordinate)})		
			#myAWSIoTMQTTClient.publish(topic_targer, mes_target, 1)	
			#print("publishing")
		
		#end=time.time()
		#print (end-start )
		#time.sleep(0.1)
		


		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
		if not continue_game:
			print ("All targets caught...")
			mes_rover = json.dumps({'outcome': '{}'.format([(0,0),(0,0)])})
			myAWSIoTMQTTClient.publish(topic_rover, mes_rover, 1)
			time.sleep (2)
			break
	cap.release()
	cv2.destroyAllWindows()
	
	
