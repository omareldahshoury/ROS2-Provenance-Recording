import os
import pandas as pd
from datetime import datetime
import cv2 #cv2 is only used for waitKey
# We use the time module to keep track of the execution time using this module
# timeit would be better, so it is better to change this in the future
from time import time


# We initialize empty varaible to store the data
data = ""
# We also initialize some other initial parameters
key = 0
flag = 0
counter = 0

# Infinite loop exits when the flag is set to zero
while True:
	
	# We append the counter which displays the iteration number
	counter = counter + 1
	data = data_node = data_topic = ""
	iterate_nodes = 1
	iterate_topics = 1

	# We store the time to generate time stamps
	now = datetime.now()
	current_time = now.strftime('%H:%M:%S.%f %a-%d-%b-%Y')
	print(current_time)
	start_time = time()


	# Reading the Topics
	# First we store the topic list
	with open('topics.txt', 'w') as fp:
		os.system("ros2 topic list >> topics.txt")

	# Then we read the list of topics
	with open('topics.txt', 'r') as file:
		topic_list = file.read()
	print(topic_list)

	# Then we cycle through the list of topics and store the in depth info about the topics
	for topic in topic_list.rstrip('\n ').split('\n'):
	#     with open('topic'+'_'.join(str(topic).split('/'))+'.txt', 'w') as tfp:
		os.system("ros2 topic info "+topic+" >> "+'topic'+'_'.join(str(topic).split('/'))+'.txt')


	# Reading the Nodes
	# First we store the node list
	with open('nodes.txt', 'w') as fp:
		os.system("ros2 node list >> nodes.txt")

	# Then we read the list of nodes
	with open('nodes.txt', 'r') as file:
		node_list = file.read()

	# Then we cycle through the list of nodes and store the in depth info about the nodes
	for node in node_list.rstrip('\n ').split('\n'):
	#     with open('node'+'_'.join(str(node).split('/'))+'.txt', 'w') as nfp:
		os.system("ros2 node info "+node+" >> "+'node'+'_'.join(str(node).split('/'))+'.txt')
	

	# We finally create a common file which has all the agents and entities along with time stamps
	# Reading data from nodes.txt
	with open('nodes.txt') as nfp:
	    data_node = nfp.read()
	  
	# Reading data from topics.txt
	with open('topics.txt') as tfp:
	    data_topic = tfp.read()

	data += "\n"  
	data += current_time
	data += "\n"
	data += data_node
	data += "\n"
	data += data_topic
	data += "\n"
  
	with open ('tracker.txt', 'a') as fp:
	    fp.write(data)

	key = cv2.waitKey(1)
	# Here we wait for an exit key which in our case is "Ctrl+S"
	if (key == (ord('s')&0xFF)):
		break
	else: 
		# Here we print the time elapsed for the last iteration
		print("Iteration:", counter, "total Time elapsed:", time()-start_time, 'seconds')
