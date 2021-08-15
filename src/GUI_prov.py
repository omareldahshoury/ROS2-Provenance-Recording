from kivymd.app import MDApp
from kivymd.uix.label import MDLabel
from kivy.lang import Builder
from kivy.uix.screenmanager import Screen
from kivymd.uix.button import MDRectangleFlatButton
import os
import pandas as pd
from datetime import datetime
from time import time


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
		exec(open(os.path.join(curr_directory,'dummy_prov_model.py')).read())
		#print("Done. Model is saved in the Extracted_Info directory.")
		print("\n Opening Prov model")
		from PIL import Image
		img = Image.open('ros-prov.png')
		img.show()
		print("Done.")		
		
		
	def ROS_Snapshot(self,*args):
		#Defining the functionality of the Ros to Prov Parser button		
		curr_directory = os.getcwd()
		print("Runing ROS2 Snapshot...")
		print("Current Directory: ", curr_directory)
		exec(open(os.path.join(curr_directory,'single_instance_prov_generator.py')).read())
		

	def record(self,*args):
		#Defining the functionality of the Record button

		curr_directory = os.getcwd()
		print("\n\nRecording is in progress...")
		exec(open(os.path.join(curr_directory,'ROS2-data-extraction-updated.py')).read())
		#print("Recording Done.")

	
		

GUIApp().run()
