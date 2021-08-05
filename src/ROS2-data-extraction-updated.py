# from _typeshed import HasFileno
from collections import namedtuple

from rclpy.action.graph import get_action_client_names_and_types_by_node
from rclpy.action.graph import get_action_server_names_and_types_by_node
from rclpy.node import HIDDEN_NODE_PREFIX
from ros2cli.node.strategy import NodeStrategy
import rclpy
from rclpy.node import Node
import time
from datetime import datetime
import pandas as pd

class MyNode(Node):
    def __init__(self):

        df2 = pd.DataFrame()

        now = datetime.now()
        current_time = now.strftime('%H:%M:%S.%f %a-%d-%b-%Y')
        print("The time is : ", current_time)
        
        super().__init__("my_node")
        print("list of nodes")
        nodes = self.get_node_names()
        nodes = [n for n in nodes if not n.startswith('_') and n != "my_node"]
        for i in nodes:
            print(i)

        def topic_or_service_is_hidden(name):
            """
            Return True if a given topic or service name is hidden, otherwise False.
            """
            return any(token for token in name.split('/') if token.startswith('_'))
 

        print("----------------------------------------------")
        time.sleep(1)
        for names in nodes:

            topic_list = []
            publisher_list = []
            subscriber_list = []
            service_server_list = []
            service_client_list = []
            

            # print("these are j ",j)
            topics = self.get_topic_names_and_types(i)
            active_topics = [(n, t) for (n, t) in topics if n.startswith('rt')]
            service_requests_topics = [(n, t) for (n, t) in topics if n.startswith('rq')]
            service_response_topics = [(n, t) for (n, t) in topics if n.startswith('rr')]

            print("currently active topics for ", names)
            for i,t in active_topics:
                print(i)

            print("----------------------------------------------")
            print("service requests topics for ", names)
            for i,t in service_requests_topics:
                print(i)

            print("----------------------------------------------")
            print("service response topics for ", names)
            for i,t in service_response_topics:
                print(i)
            
            print("----------------------------------------------")
            print()


            print("list of publishers for ", names)
            publishers = self.get_publisher_names_and_types_by_node(names, "/")
            for n1 in publishers:
                publisher_list.append(n1)
            for i, j in publisher_list:
                print(i,":",j)
            print("number of publishers : ", len(publisher_list))       
            print("----------------------------------------------")
            print()


            print("list of subscribers for ", names)
            subscribers = self.get_subscriber_names_and_types_by_node(names, "/")
            for n2 in subscribers:
                subscriber_list.append(n2)
            for i, j in subscriber_list:
                print(i,":",j)
            print("number of subscribers : ", len(subscriber_list))   
            print("----------------------------------------------")
            print()

            print("list of service server for ", names)
            services = self.get_service_names_and_types_by_node(names, "/")
            for n3 in services:
                service_server_list.append(n3)
            for i, j in service_server_list:
                print(i,":",j)
            print("----------------------------------------------")
            print()

            print("list of service clients for ", names)
            clients = self.get_client_names_and_types_by_node(names, "/")
            for n4 in clients:
                service_client_list.append(n4)  
            for i, j in service_client_list:
                print(i,":",j)         
            print("----------------------------------------------")
            print()


            print("--------------------ROS2 DATA FRAME STORAGE-------------------------------")
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



