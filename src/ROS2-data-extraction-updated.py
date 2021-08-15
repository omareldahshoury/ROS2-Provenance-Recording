from collections import namedtuple
import rclpy
from rclpy.node import Node
import time
from datetime import datetime
import pandas as pd

class MyNode(Node):
    def __init__(self):
        
        # defining a dataframe to store all the data
        df2 = pd.DataFrame()

        # to store the current timestamp
        now = datetime.now()
        current_time = now.strftime('%H:%M:%S.%f %a-%d-%b-%Y')
        print("The time is : ", current_time)
        
        super().__init__("my_node")

        # to get the list of nodes
        nodes = self.get_node_names()
        nodes = [n for n in nodes if not n.startswith('_') and n != "my_node"]

        def topic_or_service_is_hidden(name):
            """
            Return True if a given topic or service name is hidden, otherwise False.
            """
            return any(token for token in name.split('/') if token.startswith('_'))
 
        time.sleep(1)

        # looping over all the currently active nodes
        for names in nodes:

            # defining variables
            topic_list = []
            publisher_list = []
            subscriber_list = []
            service_server_list = []
            service_client_list = []
            

            # to get the topics
            topics = self.get_topic_names_and_types(names)
            active_topics = [(n, t) for (n, t) in topics if n.startswith('rt')]
            service_requests_topics = [(n, t) for (n, t) in topics if n.startswith('rq')]
            service_response_topics = [(n, t) for (n, t) in topics if n.startswith('rr')]


            # to get the list of publishers
            publishers = self.get_publisher_names_and_types_by_node(names, "/")
            for n1 in publishers:
                publisher_list.append(n1)


            # to get the list of subscribers
            subscribers = self.get_subscriber_names_and_types_by_node(names, "/")
            for n2 in subscribers:
                subscriber_list.append(n2)

            # to get the list of service server
            services = self.get_service_names_and_types_by_node(names, "/")
            for n3 in services:
                service_server_list.append(n3)

            # to get the list of service clients
            clients = self.get_client_names_and_types_by_node(names, "/")
            for n4 in clients:
                service_client_list.append(n4)  


            # to store all the data in a dataframe

            '''
            The generated csv file has the following format:
            col1 = timestamp,
            col2 = nodes
            col3 = active topics
            col4 = service request topics
            col5 = service response topics
            col6 = publishers
            col7 = number of publishers
            col8 = subscribers
            col9 = number of subscribers
            col10 = service servers
            col11 = service client
            '''
            temp_dict = {'Timestamps': current_time,
                            'Nodes': names,
                            'Active Topics': list(active_topics),
                            'Service Request Topics': list(service_requests_topics),
                            'Service Response Topics': list(service_response_topics),
                            'Publisher': list(publisher_list),
                            'No of Publishers': len(publisher_list),
                            'Subscribers': list(subscriber_list),
                            'No of Subscribers': len(subscriber_list),
                            'Service Server List': list(service_server_list),
                            'Service Client List': list(service_client_list),
                            }
            df1 = pd.DataFrame.from_dict(temp_dict, orient='index')
            df2=df1.transpose()
            df2.transpose()
            df2.to_csv('Data_storage.csv', mode='a', header=False)
            

def main(args=None):
    rclpy.init(args=args)
    while True:
        try:
            node = MyNode()
            time.sleep(1)
        except KeyboardInterrupt:
            break

if __name__ == "__main__":
    main()



