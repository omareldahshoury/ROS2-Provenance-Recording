#!/usr/bin/python3

import os
import pandas as pd
from datetime import datetime
# import cv2 #cv2 is only used for waitKey
# We use the time module to keep track of the execution time using this module
# timeit would be better, so it is better to change this in the future
from time import time


# We also initialize a counter to keep track of iterations
counter = 0

# We create a folder where we ouput all the files
base_directory = '..//Extracted_Info//'
if not os.path.isdir(base_directory):
	os.mkdir(base_directory)
elif os.listdir(base_directory):
	# We warn if directory is not empty
	print("Warning!!! Directory ('Extracted_Info') is not empty. It is a good practice to clear the directory before starting a recording session, else you may end up with extra files and wrong information")

# Infinite loop exits when the flag is set to zero
while counter < 1:
	
	# We append the counter which displays the iteration number
	counter = counter + 1
	
	# Reset the data variable
	data = ""

	# We store the time to generate time stamps
	now = datetime.now()
	current_time = now.strftime('%H:%M:%S.%f %a-%d-%b-%Y')
	print(current_time)
	start_time = time()


	# Reading the Topics
	# First we store the topic list
	with open(base_directory + 'topics.txt', 'w') as fp:
		os.system("ros2 topic list >> " + base_directory + "topics.txt")

	# Then we read the list of topics
	with open(base_directory + 'topics.txt', 'r') as file:
		topic_list = file.read()
	print(topic_list)

	# Then we cycle through the list of topics and store the in depth info about the topics
	for topic in topic_list.rstrip('\n ').split('\n'):
	#     with open('topic'+'_'.join(str(topic).split('/'))+'.txt', 'w') as tfp:
		os.system("ros2 topic info "+topic+" >> "+ base_directory + 'topic'+'_'.join(str(topic).split('/'))+'.txt')


	# Reading the Nodes
	# First we store the node list
	with open(base_directory + 'nodes.txt', 'w') as fp:
		os.system("ros2 node list >> " + base_directory + "nodes.txt")

	# Then we read the list of nodes
	with open(base_directory + 'nodes.txt', 'r') as file:
		node_list = file.read()

	# Then we cycle through the list of nodes and store the in depth info about the nodes
	for node in node_list.rstrip('\n ').split('\n'):
	#     with open('node'+'_'.join(str(node).split('/'))+'.txt', 'w') as nfp:
		os.system("ros2 node info "+node+" >> "+ base_directory +'node'+'_'.join(str(node).split('/'))+'.txt')
	

	# We finally create a common file which has all the agents and entities along with time stamps
	# Reading data from nodes.txt
	with open(base_directory + 'nodes.txt') as nfp:
	    data_node = nfp.read()
	  
	# Reading data from topics.txt
	with open(base_directory + 'topics.txt') as tfp:
	    data_topic = tfp.read()

	data += "\n"  
	data += current_time
	data += "\n"
	data += data_node
	data += "\n"
	data += data_topic
	data += "\n"
  
	with open (base_directory + 'tracker.txt', 'a') as fp:
	    fp.write(data)

	# key = cv2.waitKey(1)
	# # Here we wait for an exit key which in our case is "Ctrl+S"
	# if (key == (ord('s')&0xFF)):
	# 	break
	# else: 
	# 	# Here we print the time elapsed for the last iteration
	# 	print("Iteration:", counter, "total Time elapsed:", time()-start_time, 'seconds')
