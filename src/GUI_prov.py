from kivymd.app import MDApp
from kivymd.uix.label import MDLabel
from kivy.lang import Builder
from kivy.uix.screenmanager import Screen
from kivymd.uix.button import MDRectangleFlatButton
import os
import pandas as pd
from datetime import datetime
from time import time
from single_instance_prov_generator import *
from ROS2_data_extraction_updated import *

class GUIApp(MDApp):
	def __init__(self, **kwargs):
		super().__init__(**kwargs)
		self.screen = Builder.load_file('gui_config.py')
		self.flag = False

	def build(self):
		return self.screen

	
	def dummyprov_run(self,*args):
		#Defining the functionality of the Dummy Prov Model button		
		curr_directory = os.getcwd()
		print("Generating Prov model...")
		file_name = 'dummy_prov_model.py'
		exec(open(os.path.join(curr_directory,file_name)).read())
		#print("Done. Model is saved in the Extracted_Info directory.")
		print("\n Opening Prov model")
		from PIL import Image
		img = Image.open('ros-prov.png')
		img.show()
		print("Done.")		
		
		
	def ROS_Snapshot(self,*args):
		#Defining the functionality of the Ros Snapshot button		
		curr_directory = os.getcwd()
		#print("Runing ROS2 Snapshot...")
		file_name = 'single_instance_prov_generator.py'
		exec(open(os.path.join(curr_directory,file_name)).read())
		

	def record(self,*args):

		#Defining the functionality of the Record button
		curr_directory = os.getcwd()
		print("\n\nRecording is in progress...")
		file_name = 'ROS2_data_extraction_updated.py'
		exec(open(os.path.join(curr_directory,file_name)).read())
		#exec(compile(open(os.path.join(curr_directory,file_name),"rb").read(),file_name,'exec'), globals(), locals())
		print("Recording Done.")
		#exec(compile(open(filename, "rb").read(), filename, 'exec'), globals(), locals())


#if __name__ == "__main__":
	#a = Process(target=de)
    	#b = Process(target=modbusStart)
    	#a.start()
    	#b.start()

GUIApp().run()
