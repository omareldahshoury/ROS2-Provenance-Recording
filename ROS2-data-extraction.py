import os
import pandas as pd
from datetime import datetime

data = ""



now = datetime.now()

current_time = now.strftime("%H:%M:%S")

os.system("ros2 topic list >> 1.txt")
os.system("ros2 node list >> 2.txt")
  
data1 = data2 = ""
  
# Reading data from file1
with open('1.txt') as fp:
    data1 = fp.read()
  
# Reading data from file2
with open('2.txt') as fp:
    data2 = fp.read()

data += "\n"  
data = current_time
data += "\n"
data += data1
data += "\n"
data += data2
data += "\n"
  
with open ('3.txt', 'w') as fp:
    fp.write(data)
