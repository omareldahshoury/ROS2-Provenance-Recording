import os
import pandas as pd
from datetime import datetime
import cv2

data = ""
key = 0
flag = 0
counter = 0

while flag==0:
	
	counter = counter + 1
	iterate_nodes = 1
	iterate_topics = 1

	now = datetime.now()

	current_time = now.strftime('%H:%M:%S.%f %a-%d-%b-%Y')


	with open('1.txt', 'w') as fp:
		os.system("ros2 topic list >> 1.txt")


	with open('2.txt', 'w') as fp:
		os.system("ros2 node list >> 2.txt")

	  
	data1 = data2 = ""
	  
	# Reading data from file1
	with open('1.txt') as fp:
	    data1 = fp.read()
	  
	# Reading data from file2
	with open('2.txt') as fp:
	    data2 = fp.read()

	data += "\n"  
	data += current_time
	data += "\n"
	data += data1
	data += "\n"
	data += data2
	data += "\n"
  
	with open ('3.txt', 'w') as fp:
	    fp.write(data)

	key = cv2.waitKey(1)
	if key == 113 or key == 87 or key == ord('s')&0xFF or key == 27&0xFF:
		flag = 1
	else: 
		flag = 0
		print(counter, "iterations")
