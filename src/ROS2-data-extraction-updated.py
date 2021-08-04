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

        topic_list = []
        publisher_list = []
        subscriber_list = []
        service_server_list = []
        service_client_list = []

        now = datetime.now()
        current_time = now.strftime('%H:%M:%S.%f %a-%d-%b-%Y')
        print("The time is : ", current_time)

        super().__init__("my_node")
        print("list of nodes")
        nodes = self.get_node_names()
        nodes = [n for n in nodes if not n.startswith('_') and n != "my_node"]
        for i in nodes:
            print(i)

        # print("the state is ", self.get_current_state("node_name").id())

        def topic_or_service_is_hidden(name):
            """
            Return True if a given topic or service name is hidden, otherwise False.
            """
            return any(token for token in name.split('/') if token.startswith('_'))
 

        print("----------------------------------------------")
        time.sleep(1)
        for j in nodes:
            # print("these are j ",j)
            topics = self.get_topic_names_and_types(i)
            active_topics = [(n, t) for (n, t) in topics if n.startswith('rt')]
            service_requests_topics = [(n, t) for (n, t) in topics if n.startswith('rq')]
            service_response_topics = [(n, t) for (n, t) in topics if n.startswith('rr')]

            print("currently active topics for ", j)
            for i,t in active_topics:
                print(i)

            print("----------------------------------------------")
            print("service requests topics for ", j)
            for i,t in service_requests_topics:
                print(i)

            print("----------------------------------------------")
            print("service response topics for ", j)
            for i,t in service_response_topics:
                print(i)
            
            print("----------------------------------------------")
            print()


        for names in nodes:
            print("list of publishers for ", names)
            publishers = self.get_publisher_names_and_types_by_node(names, "/")
            for n1 in publishers:
                publisher_list.append(n1)
            for i, j in publisher_list:
                print(i,":",j)
            print("number of publishers : ", len(publisher_list))       
            print("----------------------------------------------")
            print()


        for names in nodes:
            print("list of subscribers for ", names)
            subscribers = self.get_subscriber_names_and_types_by_node(names, "/")
            for n2 in subscribers:
                subscriber_list.append(n2)
            for i, j in subscriber_list:
                print(i,":",j)
            print("number of subscribers : ", len(subscriber_list))   
            print("----------------------------------------------")
            print()

        for names in nodes:
            print("list of service server for ", names)
            services = self.get_service_names_and_types_by_node(names, "/")
            for n3 in services:
                service_server_list.append(n3)
            for i, j in service_server_list:
                print(i,":",j)
            print("----------------------------------------------")
            print()

        for names in nodes:
            print("list of service clients for ", names)
            clients = self.get_client_names_and_types_by_node(names, "/")
            for n4 in clients:
                service_client_list.append(n4)  
            for i, j in service_client_list:
                print(i,":",j)         
            print("----------------------------------------------")
            print()

        # print("list of action server")
        # time.sleep(2)
        # services = self.get_action_server_names_and_types_by_node(nodes)
        # for (n5,t5) in services:
        #     print(n5, ":", t5)

        # print("----------------------------------------------")
        # print("list of action client")
        # time.sleep(2)
        # services = self.get_action_client_names_and_types_by_node(node_name, node_namespace)
        # for (n6,t6) in services:
        #     print(n6, ":", t6)


        # print("--------------------------DATA---FRAME-------------------------------")
        # data={"col1": topic_list, "col2": subscriber_list}
        # df = pd.DataFrame.from_dict(data, orient='index')
        # print(df)
        # df.to_csv('to_csv_file.csv', index=False,header=True, encoding='utf-8')

        # data = {'Nodes': node_name, 'topics': node_namespace}  
        # df = pd.DataFrame(data)
        # df['topics'] = df['topics'].astype('object')
        # df.loc[0, 'topics'] = topic_list
        # print(df)

def main(args=None):
    rclpy.init(args=args)
    for loop in range(0,5):
        node = MyNode()
        time.sleep(1)

if __name__ == "__main__":
    main()


