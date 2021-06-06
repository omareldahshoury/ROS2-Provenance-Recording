#!/usr/bin/python3

""" This code converts the data extracted from a ROS system to prov readable format
    The output format will be in PROV-N Notation
"""

# We first import the relevant libararies
import os
import datetime
import prov
from prov.model import ProvDocument


# We create a class for generating entitity objects
class Entitiy:
    # Here we intialize the class
    def __init__(self, prov_doc, UID, no_of_pubs, no_of_subs):
        _UID = UID
        _no_of_pubs = no_of_pubs
        _no_of_subs = no_of_subs
        _init_time = datetime.datetime.now()
        self.generate_entity(prov_doc)
    
    # We collect the data and generate the entity in prov understandable terms
    def generate_entity(self, prov_doc):
        



if __name__ == "__main__":
    
    # We create an object of the prov class where we input all the data in PROV-N format
    prov_doc = ProvDocument()

    
    # First we access the folder where all the files are stored
    base_directory = '..//Extracted_Info//'
    # We access the list of the files stored in the directory
    all_files = os.listdir(base_directory)

    # This is not actually required in our case as we have appended the topic files
    # with the suffix topic which allows us to access them directly
    with open(base_directory + 'topics.txt', 'r') as tfp:
        print("The various topics beind monitored are:\n" + tfp.read())

    # Extracting relevant topic data to create entities
    for file_name in all_files:
        # We check for valid files only
        if file_name.startswith("topic_"):
            with open(base_directory + file_name, 'r') as tfp:
                data = tfp.read()

    print("Testing 1, 2, 3, ...")
