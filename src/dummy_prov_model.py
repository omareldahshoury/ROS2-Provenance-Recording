# Importing the required libraries
from prov.model import ProvDocument
import prov
import datetime

# For visualizations
from prov.dot import prov_to_dot
from IPython.display import Image

# First we create a Provenance data file
ros = ProvDocument()

# We can write name spaces here
ros.set_default_namespace('https://docs.ros.org/en/dashing/Installation.html')
ros.add_namespace('node', 'https://docs.ros.org/en/dashing/Tutorials/Understanding-ROS2-Nodes.html') # represents ros nodes
ros.add_namespace('topic', 'https://docs.ros.org/en/dashing/Tutorials/Topics/Understanding-ROS2-Topics.html') # represents ros topics
ros.add_namespace('msg', 'http://wiki.ros.org/msg') # represents ros messages
ros.add_namespace('activity', 'undefined') # represents the processes performed
ros.add_namespace('msg_format', 'The format in which the data is stored/passed') # represents the processes performed

# Some more for future use
ros.add_namespace('module', 'python programs or groups') # represents collection of ros enities, may be in the form of a program
ros.add_namespace('user', 'agent editing/contributing to the program') # represents code contributors
ros.add_namespace('enduser', 'end user who interacts with the program') # this represents the end user and his interaction with the code


### Creating the elements of Prov (Entities, Agents and Activities/Processes)
# First we generate the nodes as agents
ros.agent('node:talker',\
         {'node:time_initialized':datetime.datetime.now(),\
          'node:param_use_sim_time':'False'})
ros.agent('node:listener',\
         {'node:time_initialized':datetime.datetime.now(),\
          'node:param_use_sim_time':'False'})

# We then create the processes/activities related to each node and topic
# For instance, activity:node_Publish_to_topic or activity:node_Subscribe_to_topic

# Actvities related to node:talker
ros.activity('activity:talker_Publish_to_chatter', datetime.datetime.now())
ros.activity('activity:talker_Publish_to_parameter_events', datetime.datetime.now())
ros.activity('activity:talker_Publish_to_rosout', datetime.datetime.now())

# Actvities related to node:listener
ros.activity('activity:listener_Subscribe_to_chatter', datetime.datetime.now())
ros.activity('activity:listener_Publish_to_parameter_events', datetime.datetime.now())
ros.activity('activity:listener_Publish_to_rosout', datetime.datetime.now())

# We also have to define the set ROS param activity for the nodes
ros.activity('activity:talker_set_param')
ros.activity('activity:listener_set_param')

# We then create the entities that are being used

# We begin with describing the topics being used
# The essential parameters which we'd like to define are: name, type/msg_format and time of initialization
ros.entity('topic:chatter',\
           other_attributes={'prov:label':'chatter',\
            'prov:time_initialized':datetime.datetime.now()})
ros.entity('topic:parameter_events',\
           {'prov:label':'parameter_events',\
            'prov:time_initialized':datetime.datetime.now()})
ros.entity('topic:rosout',\
           {'prov:label':'rosout',\
            'prov:time_initialized':datetime.datetime.now()})

# As the topics use a pre-defined data type, we could also make these data structures entities
ros.entity('msg_format:std_msgs/msg/String')
ros.entity('msg_format:rcl_interfaces/msg/ParameterEvent')
ros.entity('msg_format:rcl_interfaces/msg/Log')

# Then we describe the message being exchanged, we should note that the graphical view can become a bit clunky
# but we have to model messages as entities to ensure that we store the information, as for the point regarding
# readability, we can improve this later by query languages such as cypher which allow us to narrow down our
# vision to a specific portion of the visualized model by specifiying some constraints.

ros.entity('msg:message_1',\
           {'prov:topic':'/chatter',\
            'prov:type':'std_msgs/msg/String',\
            'prov:time_generated':datetime.datetime.now()})

# Finally we draw the relations between the nodes, topics, messages and activities

# Next we define the relations for each topic/msg

# We begin with the topic Chatter
# Node "Talker" publishes to topic "Chatter" through the activity "talker_Publish_to_chatter"
ros.used('node:talker','activity:talker_Publish_to_chatter')
ros.wasInfluencedBy('topic:chatter', 'activity:talker_Publish_to_chatter')
# Node "Listener" listens to the chatter topic through the activity "listener_Subscribe_to_chatter"
ros.used('node:listener', 'activity:listener_Subscribe_to_chatter')
ros.wasInformedBy('activity:listener_Subscribe_to_chatter', 'topic:chatter')

# Topics borrow from pre-defined message types
ros.wasDerivedFrom('topic:chatter', 'msg_format:std_msgs/msg/String')
ros.wasDerivedFrom('topic:parameter_events', 'msg_format:rcl_interfaces/msg/ParameterEvent')
ros.wasDerivedFrom('topic:rosout', 'msg_format:rcl_interfaces/msg/Log')

# We then give the same relations for the messages
# Node "Talker" publishes the message "message_1"
ros.wasGeneratedBy('msg:message_1', 'activity:talker_Publish_to_chatter')
# Node "Listener" listens to the message "message_1"
ros.used('activity:listener_Subscribe_to_chatter', 'msg:message_1')

# Defining the hidden relations

# Publishing to parameter_events
# talker
ros.used('node:talker','activity:talker_Publish_to_parameter_events')
ros.wasInfluencedBy('topic:parameter_events', 'activity:talker_Publish_to_parameter_events')
# listener
ros.used('node:listener','activity:listener_Publish_to_parameter_events')
ros.wasInfluencedBy('topic:parameter_events', 'activity:listener_Publish_to_parameter_events')

# Publishing to rosout
# talker
ros.used('node:talker','activity:talker_Publish_to_rosout')
ros.wasInfluencedBy('topic:rosout', 'activity:talker_Publish_to_rosout')
# listener
ros.used('node:listener','activity:listener_Publish_to_rosout')
ros.wasInfluencedBy('topic:rosout', 'activity:listener_Publish_to_rosout')

# We also have to define the set relations, which change the ROS param of the nodes
# Talker
ros.wasInfluencedBy('node:talker', 'activity:talker_set_param')
ros.used('node:talker','activity:talker_set_param')
# Listener
ros.wasInfluencedBy('node:listener', 'activity:listener_set_param')
ros.used('node:listener','activity:listener_set_param')


# Saving the File and Visualizing the graph
dot = prov_to_dot(ros)
dot.write_png('ros-prov.png')
Image('ros-prov.png')

# Printing the prov doc so far
# print(ros.get_provn())