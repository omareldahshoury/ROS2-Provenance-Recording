""" This Program takes a snapshot of a single instance of a running ROS System and generates
    an equivalent Prov Model as per the definitions defined in Prov_Model.md
"""

# Used to extract ROS system info
import rclpy
# Used to add timestamp information to arrange the datapoints sequentially
from datetime import datetime
# To introduce small delays
import time
# Primarily used to correctly source paths
import os
# Libraries used to create the Prov Doc 
import prov
from prov.model import ProvDocument
# Custom Library to hold the Prov Definitions
import prov_utilities
# To keep the order of input sequential wrt time
from collections import OrderedDict
# For visualizations
from prov.dot import prov_to_dot
from IPython.display import Image


def extract_ros_info(recording_node):
    """ Function to extract the information from the ROS system
        Note that the passed node and it's relations won't be recorded
    
        Input:      Object of Class rclpy.node.Node()

        Output:     Dictionary with extracted system relevant information
    """
    
    # First we note the time and store it in the dictionary, this will be our key for the snapshot taken
    # at this instance
    current_time = datetime.now().strftime('%H:%M:%S.%f %a-%d-%b-%Y')
    
    # We initialize an empty dictionary, this will store all the relevant information
    ros_info = OrderedDict()
    ros_info = {current_time: {'nodes': {}, 'topics':{}}}
    
    # Exlcusions
    # We exclude certain nodes from our database which we don't want to record
    excluded_nodes = [recording_node.get_name()]
    # We could do the same with topics
    excluded_topics = []

    # We then retrieve the list of all the nodes in the system along with their namespaces
    nodes_n_namespaces = recording_node.get_node_names_and_namespaces()
    # We then update this info in the dictionary
    for node, namespace in nodes_n_namespaces:
        # We don't want to save the recorder node as it isn't part of the system
        if node not in excluded_nodes:
            # We get the Publishing relations for each node
            pubs = recording_node.get_publisher_names_and_types_by_node(node, namespace)
            list_of_pubs = [elem for elem,_ in pubs]
            # We then get the Subscriptions for each node
            subs = recording_node.get_subscriber_names_and_types_by_node(node, namespace)
            list_of_subs = [elem for elem,_ in subs]
            # Then we finally store them
            ros_info[current_time]['nodes'][node] = {'name': node,
                                                    'namespace': namespace,
                                                    'publishes_to': list_of_pubs,
                                                    'subscribes_to': list_of_subs
                                                    }

    # We then get the list of all the topics
    topic_list = recording_node.get_topic_names_and_types()
    for topic, type in topic_list:
        if topic not in excluded_topics:
            ros_info[current_time]['topics'][topic] = { 'name': topic,
                                                        'type': type[0]
                                                        }

    # Finally, we return the data structure
    return ros_info


# Code from https://stackoverflow.com/questions/3229419/how-to-pretty-print-nested-dictionaries
def display_ros_info(ros_info, indent=0):
    """ Code to print the dictionary in a semi-readable format

        Input:      Dictionary of extracted info

        Output:     None
    """
    for key, value in ros_info.items():
        print('\t' * indent + str(key))
        if isinstance(value, dict):
            display_ros_info(value, indent+1)
        else:
            print('\t' * (indent+1) + str(value))


def ros2prov(ros_info):
    """ This function takes the extracted information of the ROS system as input
        and then uses it to create a Prov Model.
        This function repurposes the code used in `ros_to_prov_parser.py` with
        appropriate edits to suit the current version

        Input:      Dictionary of extracted info

        Output:     Saves a pdf of the visual representation of the Prov Model
                    Saves the Prov Model in Prov-N format
    """
    # We create an object of the prov class where we input all the data in PROV-N format
    prov_doc = ProvDocument()

    # Defining namespaces
    prov_doc.set_default_namespace('https://docs.ros.org/en/dashing/Installation.html')
    prov_doc.add_namespace('node', 'https://docs.ros.org/en/dashing/Tutorials/Understanding-ROS2-Nodes.html') # represents ros nodes
    prov_doc.add_namespace('topic', 'https://docs.ros.org/en/dashing/Tutorials/Topics/Understanding-ROS2-Topics.html') # represents ros topics
    prov_doc.add_namespace('activity', 'undefined') # represents the processes performed
    prov_doc.add_namespace('msg', 'http://wiki.ros.org/msg') # represents ros messages
    prov_doc.add_namespace('data_format', 'The format in which the data is stored/passed') # represents the processes performed

    # We begin with creating the topics (entities)
    for topic_info in ros_info[list(ros_info.keys())[-1]]['topics']:
        exec("entity_{} = prov_utilities.Topic(prov_doc,\
                                 ros_info[list(ros_info.keys())[-1]]['topics'][topic_info]['name'],\
                                 ros_info[list(ros_info.keys())[-1]]['topics'][topic_info]['type'],\
                                 list(ros_info.keys())[-1])"\
                                .format(ros_info[list(ros_info.keys())[-1]]['topics'][topic_info]['name'].lstrip('/')))
        exec("list_of_objects.append(entity_{})".\
            format(ros_info[list(ros_info.keys())[-1]]['topics'][topic_info]['name'].lstrip('/')))

    return prov_doc


# Global Object list, because I am lazy to pass em through functions each and everytime
list_of_objects = []

if __name__ == "__main__":
    
    # We initialize rclpy
    rclpy.init()
    
    # Here we create a node which is used to record the ROS System
    r2p = rclpy.create_node('ros2prov_recorder')

    # 100 millisec time to let the system setup
    time.sleep(0.1)

    # Next we extract the info
    ros_info = extract_ros_info(r2p)

    # Displaying the extracted Info
    display_ros_info(ros_info)

    # Now we proceed to create a model using the collected info
    prov_doc = ros2prov(ros_info)

    # Saving the File and Visualizing the graph
    print('Saving Files... to', os.curdir)
    
    dot = prov_to_dot(prov_doc)
    dot.write_png('ros-prov-revised.png')
    Image('ros-prov-revised.png')
    
    # Saving the File in JSON Format
    prov_doc.serialize('ros2prov.json')