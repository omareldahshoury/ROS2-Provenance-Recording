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


# We create a class for generating entitity objects
class Agent:
    # Here we intialize the class
    def __init__(self, prov_doc, UID):
        self.UID = UID
        self.init_time = datetime.datetime.now()
        self.generate_agent(prov_doc)
    
    # We collect the data and generate the agent in prov understandable terms
    def generate_agent(self, prov_doc):
        print("My name is:", self.UID)
        self.elem = prov_doc.agent('node:{}'.format(self.UID),\
            other_attributes = {'prov:label': self.UID,\
            'prov:time_initialized':self.init_time})

    # This function parses data from the node_ files and extracts relevant data
    def agent_info_parser(self, prov_doc, data):
        # print('Hi, I have reached this function')
        # print(data)
        # First we extract the name of the node
        UID = data.pop(0).strip(" /")
        print(UID)
        
        # We create a reference dictionary which stores the order of keys to be read
        # If we have to extract additional info, it can be added here
        extract_dict = {'Subscribers': None, 'Publishers': None, 'Services': None}
        current_key, li = "Nothing", []
        # We cycle through the data
        for _ in range(len(data)):
            # For the data, we check if it is one of the keys
            info = data[0].strip(" :")
            if info in extract_dict.keys():
                # We store all the data which is related to the key
                extract_dict[current_key] = li
                # We then change the key to the next input
                current_key = data.pop(0).strip(" :")
                # Finally, we reset the key
                li = []
            else:
                # We keep appending all entries to a list, these will be parsed later
                li.append('_'.join(data.pop(0).strip(" /").split('/')))
        # We store the data from the last key as well
        extract_dict[current_key] = li
        # print(extract_dict, "\n\n")


        # Now we build the relations between the systems
        # First we create subscribers to get YouTube Famous
        generate_subs(self.UID, prov_doc, extract_dict['Subscribers']) 
        # print("========\n", extract_dict['Subscribers'], "\n========")

        return


# This function relates an agent to a set of topics as subscriber
# Note that we didn't create a class for activities themselves as these will be called
# by just using their names/labels
def generate_subs(agent, prov_doc, subs): #, subs
    print(subs)
    # If it is empty, then we return
    if subs == ['']:
        print("No subscriptions")
        return
    else:
        for elem in subs:
            topic, msg_format = elem.split(":")
            exec("entity_{}.message = msg_format".format(topic))
            prov_doc.activity('activity:Subscribe_to_{}'.format(topic), datetime.datetime.now())

    # prov_doc.activity('activity:Subscribe_to_chatter', datetime.datetime.now())

# Global declarations
# List of Objects for the purpose of passing in between functions
list_of_objects = []

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
                print(UID)
                # e = Entity(prov_doc, UID, no_of_pubs, no_of_subs)
                # exec("global entity_{}".format(UID))
                exec("entity_{} = Entity(prov_doc, UID, no_of_pubs, no_of_subs)".format(UID))
                # We append the new object to the global List of Objects
                exec("list_of_objects.append(entity_{})".format(UID))
    
    # Printing list of active nodes
    with open(base_directory + 'nodes.txt', 'r') as nfp:
        print("The various nodes beind monitored are:\n" + nfp.read())

    # Extracting relevant node data to create agents and draw relations with other elements
    for file_name in all_files:
        # We check for valid files only
        if file_name.startswith("node_"):
            print("File:", file_name)
            with open(base_directory + file_name, 'r') as nfp:
                # data = nfp.read().split('\n')
                UID = file_name.lstrip("node_").rstrip('.txt') 
                exec("agent_{} = Agent(prov_doc, UID)".format(UID))
                exec("agent_{}.agent_info_parser(prov_doc, nfp.read().split('\\n'))".format(UID))
                exec("list_of_objects.append(agent_{})".format(UID))
    # print(Entity.message = "stlsfjdoiwjkef") int = 10
    

    # Previewing the prov doc in Prov N notation
    # print(prov_doc.get_provn())

    # Saving the File and Visualizing the graph
    dot = prov_to_dot(prov_doc)
    dot.write_png(base_directory + 'ros-prov.png')
    Image(base_directory + 'ros-prov.png')