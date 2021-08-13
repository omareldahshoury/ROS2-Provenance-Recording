""" [ROS][Prov Utilities]
    Header file which contains Relevant Classes and Methods to store the ROS 2 Prov Data such as
    Topics, Nodes etc 
"""

# Importing the Relevant libraries
import datetime
import prov


# We create a class for generating topic objects
class Topic:
    """ Class which generates ROS Topics using prov entities from the prov class
        This also has specific methods used to update ROS Topics. 
    """
    # Here we intialize the class
    def __init__(self, prov_doc, UID, init_time):
        self.UID = UID
        self.init_time = init_time
        self.last_active = init_time
        self.generate_topic(prov_doc)
    
    # We collect the data and generate the entity in prov understandable terms
    # This function should only be called the first time i.e. during initialization
    def generate_topic(self, prov_doc):
        self.elem = prov_doc.entity('topic:{}'.format(self.UID),\
            other_attributes = {'topic:label': self.UID,\
                                'topic:time_initialized':self.init_time,\
                                'topic:last_active':self.last_active})

    # This function is used to update the existing topics if there are any changes
    def update_topic(self, prov_doc):
        # The code for editing the attributes will come here
        pass


# We create a class for generating message objects
class Message:
    """ Class which generates ROS Message Storage Formats using prov entities from the 
        prov class.
    """
    # Here we intialize the class
    def __init__(self, prov_doc, msg_type):
        self.msg_type = msg_type
        self.generate_msg_type(prov_doc)
    
    # We collect the data and generate the message entity in prov understandable terms
    # This function should only be called the first time i.e. during initialization or
    # when a new message type has been used
    def generate_msg_type(self, prov_doc):
        self.elem = prov_doc.entity('msg:{}'.format(self.msg_type))
    

# We create a class for generating entitity objects
class Agent:
    """ Class used to generate and store information related to prov Agents. Examples of these
        are ROS Nodes. This class also has other methods and functions used to update Agent definitions
        and relations
    """
    # Here we intialize the class
    def __init__(self, prov_doc, UID, init_time):
        self.UID = UID
        self.init_time = init_time
        self.generate_agent(prov_doc)
    
    # We collect the data and generate the agent in prov understandable terms
    def generate_agent(self, prov_doc):
        self.elem = prov_doc.agent('node:{}'.format(self.UID),\
            other_attributes = {'node:label': self.UID,\
                'node:time_initialized':self.init_time,\
                'node:last_active':self.init_time})

    def update_agent(self, prov_doc):
        # The code for editing agent attributes comes here
        pass