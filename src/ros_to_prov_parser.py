#!/usr/bin/python3

""" This code converts the data extracted from a ROS system to prov readable format
    The output format will be in PROV-N Notation
"""

# We first import the relevant libararies
import os
import datetime
import prov
from prov.model import ProvDocument
# For visualizations
from prov.dot import prov_to_dot
from IPython.display import Image


# We create a class for generating entitity objects
class Entity:
    # Here we intialize the class
    def __init__(self, prov_doc, UID, no_of_pubs, no_of_subs):
        self.UID = UID
        self.no_of_pubs = no_of_pubs
        self.no_of_subs = no_of_subs
        self.message = None
        self.init_time = datetime.datetime.now()
        self.generate_entity(prov_doc)
    
    # We collect the data and generate the entity in prov understandable terms
    def generate_entity(self, prov_doc):
        self.elem = prov_doc.entity('topic:{}'.format(self.UID),\
            other_attributes = {'prov:label': self.UID,\
            'prov:type': self.message,\
            'prov:Number_of_Publishers': self.no_of_pubs,\
            'prov:Number_of_Subscribers': self.no_of_subs,\
            'prov:time_initialized':self.init_time})

    def topic_info_parser(data):
        data_li = data.rstrip("\n ").split("\n")
        UID = data_li[0].split(" ")[-1].lstrip("/")
        no_of_pubs = int(data_li[1].split(" ")[-1])
        no_of_subs = int(data_li[2].split(" ")[-1])

        return UID, no_of_pubs, no_of_subs



if __name__ == "__main__":
    
    # We create an object of the prov class where we input all the data in PROV-N format
    prov_doc = ProvDocument()

    # Defining namespaces
    prov_doc.set_default_namespace('https://docs.ros.org/en/dashing/Installation.html')
    prov_doc.add_namespace('node', 'https://docs.ros.org/en/dashing/Tutorials/Understanding-ROS2-Nodes.html') # represents ros nodes
    prov_doc.add_namespace('topic', 'https://docs.ros.org/en/dashing/Tutorials/Topics/Understanding-ROS2-Topics.html') # represents ros topics
    prov_doc.add_namespace('activity', 'undefined') # represents the processes performed

    
    # First we access the folder where all the files are stored
    base_directory = '..//Extracted_Info//'
    # We access the list of the files stored in the directory
    all_files = os.listdir(base_directory)

    # This is not actually required in our case as we have appended the topic files
    # with the suffix topic which allows us to access them directly
    with open(base_directory + 'topics.txt', 'r') as tfp:
        print("The various topics beind monitored are:\n" + tfp.read())

    print(all_files)
    # Extracting relevant topic data to create entities
    for file_name in all_files:
        # We check for valid files only
        if file_name.startswith("topic_"):
            with open(base_directory + file_name, 'r') as tfp:
                UID, no_of_pubs, no_of_subs = Entity.topic_info_parser(tfp.read())
                # e = Entity(prov_doc, UID, no_of_pubs, no_of_subs)
                exec("entity_{} = Entity(prov_doc, UID, no_of_pubs, no_of_subs)".format((file_name.lstrip("topic_").rstrip('.txt'))))
    # print(Entity.message = "stlsfjdoiwjkef") int = 10

    print(prov_doc.get_provn())

    # Saving the File and Visualizing the graph
    dot = prov_to_dot(prov_doc)
    dot.write_png(base_directory + 'ros-prov.png')
    Image(base_directory + 'ros-prov.png')