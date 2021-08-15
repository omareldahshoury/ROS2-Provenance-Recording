# Simple ROS 2 Prov Model

Here we detail the specifications for a simple ROS 2 Prov Models. We use the example of two communicating ROS 2 nodes to create the Prov Model.

**Setup:**
- We don’t need to explicitly initialize a ROS Master in ROS 2
- We start a talker node <pre>ros2 run demo_nodes_py talker</pre>
- We start a listener node <pre>ros2 run demo_nodes_py listener</pre>

We define namespaces for each component in the ROS Prov Model (E.g. Node, Topic, Message, Topic, Activity etc)

### Modelling the Components
**Entities:**
 - **Messages:**
   * _Definition_: messages passed between _nodes_ through _topics_
   * _Namespace_: A namespace which has a distinct name (E.g. _msg_)
   * We model messages as separate entities from Topics, messages emanate from the publishing activity and run parallel to the topic and join the receiving node subscription activity. This helps us to trace the original publisher of the message
   * _Essential Attributes_:
     - The message being sent
     - Time stamp – when the message was created (as well as sent and received)
     - Generating node (not just the activity) prov:wasGeneratedBy (here we mean the sending node - it is like Qualified Expression). Currently, this can be automatically inferred from by the generating activity which is connected to a single node & topic
     - Format of the message (string, integer, custom formats etc. This may simply be expressed as a relational Qualifier to the message format)
 - **Topics:**
   * _Definition_: where/through which the nodes publish their messages or subscribe to a message
   * _Namespace_: _topic_
   * _Essential Attributes_:
     - Name/label of the topic
     - Initialization time stamp
     - Supported Format of message (This may simply be expressed as a relational Qualifier to the message format)
 - **Message Format**
   * _Definition_: This specifies the data structure/format of a message
   * _Namespace_: _msg_format_
   * _Essential Attributes_:
     - Name of the message format
     - May have a self contained description of the message

Entities are defined by the data flow model. The connections between entities is done using `prov:wasDerivedFrom`.

**Message Passing:**
It is better to use a Qualified Expression to show the entire path, this may be better at avoiding Anti-patterns.
 - Entity (message) is related to the Activity (Publisher) by `prov:wasGeneratedBy`
 - Activity (subscriber) is related to the Entity (message/topic) by `prov:used`
 - We do not require the concept of a separate sent msg and derived msg as both these activities do not alter the message (this is separate from the fact that the origin and end node will be traceable, this merely states that the message passing through the topic doesn’t change the message itself, it merely forwards it). In case such relations are required, we may relate them using prov:specializationOf.

**Expressing Revisions in Prov Model:**
Currently, we only consider revisions of entities
 - Iterations are related to each other using `prov:wasRevisionOf`
 - Each iteration is related to the original using `prov:specializationOf`

**Expressing obsolete entities:**
Consider an ‘updated’ entity which was generated from ‘original’ entity
 - Updated `prov:wasDerivedFrom` original
 - Original `prov:wasInvalidatedBy` activity
 - Activity `prov:wasGeneratedBy` original

**Agents:**<br>
We introduce the Agents to the picture after we have decided the data process view, agents assist us to refine our prov model. We see agents from the view of responsibility

 - **Nodes:**
   * _Definition_: ROS 2 Nodes qualify as nodes within our model
   * _Namespace_: We simply use _node_ as the namespace
   * _Essential Attributes_:
     - Label/Name of the Node
     - Initialization Time
     - Last Active Time
     - Life Cycle of the Node
   * _Node Parameters_ (ROS2 Params):
     - We model ROS2 Params as attributes of the nodes as these are not separate entities, rather these can be understood as variable descriptions of nodes
     - In order to keep track of changes to this params, we can introduce the concept of activities (get and set methods) which modify these params specific to the node.

**Activities:**
 - Definition: ROS communication concepts form the basis of activities. Actions from Agents on entities are modelled through specific activities
 - Namespace: _activity_
 - Essential Attributes:
   * Name/label specifying the agent and the entity
   * Initialization time
This is the final piece of the puzzle which helps us describe the prov model

**Adding relations:**
 - An activity between a publishing Node and a Topic to which the message is published has 2 relational qualifiers
   * We use `prov.used` for the relation starting from the Node to the Publishing Activity to denote that the Node/Agent used the Activity
   * We use `prov.wasInfluencedBy` for the relation between the Topic and Publishing Activity. Here it means that the Topic was influenced by the Activity
 - An activity between a Subscribing Node and a Topic from which the message has been received has 2 relational qualifiers
   * We use `prov.used` for the relation starting from the Node to the Subscription Activity to denote that the Node/Agent used the Activity
   * We use `prov.wasInformedBy` for the relation between the Subscribing Activity and the topic. This means that the Activity received information from the Topic.
 - ROS Parameters also require 2 relational Qualifiers between the same Node and the Activity associated with the particular parameters. Separete activities can be used for getter and setter methods
   * We use `prov.used` for the relations starting from the Node to the Getter/Setter Activity
   * We use `prov.wasInfluencedBy` for the relation between the Activity and the Node. This is used for information retrieval or for chaning a ROS Parameter. The directionality of the relation changes based on whether it is a Getter Method or a Setter Method.

This file is used as reference file when we are trying to build a model of our system. There are still many aspects of a ROS system which have to be modeled and defined such as Services or Actions and their relations which the other components of the system.
