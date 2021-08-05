import datetime
import prov

# We create a class for generating topic objects
class Topics:
    # Here we intialize the class
    def __init__(self, prov_doc, UID, message_type, init_time):
        self.UID = UID
        self.message_type = None
        self.init_time = init_time
        self.last_active = init_time
        self.generate_entity(prov_doc)
    
    # We collect the data and generate the entity in prov understandable terms
    # This function should only be called the first time i.e. during initialization
    def generate_entity(self, prov_doc):
        self.elem = prov_doc.entity('topic:{}'.format(self.UID),\
            other_attributes = {'prov:label': self.UID,\
                                'prov:type': self.message,\
                                'prov:time_initialized':self.init_time,\
                                'prov:last_active':self.last_active})

    def topic_info_parser(data):
        data_li = data.rstrip("\n ").split("\n")
        UID = data_li[0].split(" ")[-1].lstrip("/")
        no_of_pubs = int(data_li[1].split(" ")[-1])
        no_of_subs = int(data_li[2].split(" ")[-1])

        return UID, no_of_pubs, no_of_subs

# We create a class for generating entitity objects
class Entity:
    # Here we intialize the class
    def __init__(self, prov_doc, UID, no_of_pubs, no_of_subs):
        self.UID = UID
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
        # print("My name is:", self.UID)
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

        # Now we create publishers
        generate_pubs(self.UID, prov_doc, extract_dict['Publishers']) 


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
            
            # We first update the entity object definitions and then the prov definitions for the same
            exec("entity_{}.message = msg_format".format(topic))
            exec("entity_{}.generate_entity(prov_doc)".format(topic))

            # Now we generate the Subscribe to activity Activity
            prov_doc.activity('activity:Subscribe_to_{}'.format(topic), datetime.datetime.now())
            # Now we build the relations in between the entity, agent and activity
            exec("prov_doc.used('activity:Subscribe_to_{}','node:{}')".format(topic, agent))
            exec("prov_doc.used('topic:{0}','activity:Subscribe_to_{0}')".format(topic))
            exec("prov_doc.wasInformedBy('node:{0}','topic:{1}')".format(agent, topic))

# This function relates an agent to a set of topics as a publisher
def generate_pubs(agent, prov_doc, pubs):
    print(pubs)
    # If it is empty, then we return
    if pubs == ['']:
        print("No subscriptions")
        return
    else:
        for elem in pubs:
            topic, msg_format = elem.split(":")
            
            # We first update the entity object definitions and then the prov definitions for the same
            exec("entity_{}.message = msg_format".format(topic))
            exec("entity_{}.generate_entity(prov_doc)".format(topic))

            # Now we generate the Publish to activity Activity
            prov_doc.activity('activity:Publish_to_{}'.format(topic), datetime.datetime.now())
            # Now we build the relations in between the entity, agent and activity
            exec("prov_doc.used('node:{}','activity:Publish_to_{}')".format(agent, topic))
            exec("prov_doc.used('topic:{0}','activity:Publish_to_{0}')".format(topic))
            exec("prov_doc.wasInformedBy('topic:{}','node:{}')".format(topic, agent))