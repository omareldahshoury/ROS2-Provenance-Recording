### Simple ROS 2 Prov Model

Here we detail the specifications for a simple ROS 2 Prov Models. We use the example of two communicating ROS 2 nodes to create the Prov Model.

**Setup:**
- We don’t need to explicitly initialize a ROS Master in ROS 2
- We start a talker node
- We start a listener node

We define namespaces for each component in the ROS Prov Model

**Entities:**
 - **Messages:**
   * Definition: messages passed between nodes through topics
   * Namespace: A namespace which uses a distinct topic name (E.g. prov:message:topic_name)
   * We model messages as separate entities from Topics, messages emanate from the publishing activity and run parallel to the topic and join the receiving node. This helps us to trace the original publisher of the message
   * Essential Attributes:
     - Format of the message (string, integer, custom formats etc)
     - The message itself
     - Time stamp – when the message was created (as well as sent and received)
     - Generating node (not just the activity) prov:wasGeneratedBy (here we mean the sending node – we could create a custom prov definition for this)
 - **Topics:**
   * Definition: where the messages are published and subscribed
   * Namespace: prov:topic
   * Essential Attributes:
     - Supported Format of message
     - Initialization time stamp

Entities are defined by the data flow model. The connections between entities is done using prov:wasDerivedFrom

**Message Passing:**
It is better to use a Qualified Expression to show the entire path, this may be better at avoiding Anti-patterns.
 - Entity (message/topic) is related to the Activity (Publisher) by prov:wasGeneratedBy
 - Activity (subscriber) is related to the Entity (message/topic) by prov:used
 - We do not require the concept of a separate sent msg and derived msg as both these activities do not alter the message (this is separate from the fact that the origin and end node will be traceable, this merely states that the message passing through the topic doesn’t change the message itself, it merely forwards it). In case such relations are required, we may relate them using prov:specializationOf.

**Expressing Revisions in Prov Model:**
Currently, we only consider revisions of entities
 - Iterations are related to each other using prov:wasRevisionOf
 - Each iteration is related to the original using prov:specializationOf

**Expressing obsolete entities:**
Consider an ‘updated’ entity which was generated from ‘original’ entity
 - Updated prov:wasDerivedFrom original
 - Original prov:wasInvalidatedBy activity
 - Activity prov:wasGeneratedBy original

**Agents:**
We introduce the Agents to the picture after we have decided the data process view, agents assist us to refine our prov model. We see agents from the view of responsibility

**Nodes:**
 - Definition: ROS 2 Nodes qualify as nodes within our model
 - Namespace: We simply use prov:node as the namespace
 - Essential Attributes:
   * Initialization Time
   * Duration since initialization
   * How was it generated prov:wasGeneratedBy
   * Life Cycle of the Node
 - Node Parameters (ROS2 Params):
   * We model ROS2 Params as attributes of the nodes as these are not separate entities, rather these can be understood as variable descriptions of nodes
   * In order to keep track of changes to this params, we can introduce the concept of activities (get and set methods) which modify these params specific to the node.

**Activities:**
 - Definition: ROS communication concepts form the basis of activities (can be expanded later)
 - Namespace: prov:activity
 - Essential Attributes:
   * Initialization time
   * Duration since initialization
This is the final piece of the puzzle which helps us describe the prov model

Some open questions and/or problems to be solved:
 - Should publishers/subscribers (i.e. activities) be unique to nodes, topics or type of message being relayed? (The Answer is: we should have a custom publisher for a specific node publishing to a specific topic, for instance, 	prov:activity:Publish_to_topic_from_node)
 - In case where there is a single common activity for multiple entities, how to model the input/output relations without losing the relations (should more relations be added with the generating node and the subscribing node?)
 - What should be the relations between messages and topics in entities? 
   * Should the messages be stored sequentially in the entity topic? (For the purposes of this projecdt, just being able to track the count of the messages seems to be sufficient)
   * How should the two be visualized (ideally as messages passing through topics), how is this to be done?
