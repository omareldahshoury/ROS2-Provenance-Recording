""" This Program takes a snapshot of a single instance of a running ROS System and generates
    an equivalent Prov Model as per the definitions defined in Prov_Model.md
"""

# Used to extract ROS system info
import rclpy
# Used to add timestamp information to arrange the datapoints sequentially
from datetime import datetime
# To introduce small delays
import time


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