from collections import namedtuple

from rclpy.action.graph import get_action_client_names_and_types_by_node
from rclpy.action.graph import get_action_server_names_and_types_by_node
from rclpy.node import HIDDEN_NODE_PREFIX
from ros2cli.node.strategy import NodeStrategy
import rclpy
from rclpy.node import Node
import time
import pandas as pd

class MyNode(Node):
    def __init__(self):

        topic_list = []
        publisher_list = []
        subscriber_list = []
        service_server_list = []
        service_client_list = []

        super().__init__("my_node")
        print("list of nodes")
        nodes = self.get_node_names()
        print(nodes)
        node_name = nodes[1]
        node_namespace = nodes[0]

        HIDDEN_TOPIC_PREFIX = '_'

        def topic_or_service_is_hidden(name):
            """
            Return True if a given topic or service name is hidden, otherwise False.
            """
            return any(token for token in name.split('/') if token.startswith(HIDDEN_TOPIC_PREFIX))
 

        print("----------------------------------------------")
        print("list of topics")
        time.sleep(1)
        topics = self.get_topic_names_and_types(nodes)
        topics = [(n, t) for (n, t) in topics if not topic_or_service_is_hidden(n)]
        for i,t in topics:
            print(i)

        print("----------------------------------------------")
        print("list of publishers")
        time.sleep(1)
        publishers = self.get_publisher_names_and_types_by_node("node_name", "/")
        for n1 in publishers:
            publisher_list.append(n1)
        print(publisher_list)

        print("----------------------------------------------")
        print("list of subscribers")
        time.sleep(1)
        subscribers = self.get_subscriber_names_and_types_by_node(node_name, "/")
        for n2 in subscribers:
            subscriber_list.append(n2)
        print(subscriber_list)


        print("----------------------------------------------")
        print("list of service server")
        time.sleep(1)
        services = self.get_service_names_and_types_by_node(node_name, "/")
        for n3 in services:
            service_server_list.append(n3)
        print(service_server_list)


        print("----------------------------------------------")
        print("list of service client")
        time.sleep(1)
        clients = self.get_client_names_and_types_by_node(node_name, "/")
        for n4 in clients:
            service_client_list.append(n4)  
        print(service_client_list)          

        # print("----------------------------------------------")
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


        print("--------------------------DATA---FRAME-------------------------------")
        data={"col1": topic_list, "col2": subscriber_list}
        df = pd.DataFrame.from_dict(data, orient='index')
        print(df)
        df.to_csv('to_csv_file.csv', index=False,header=True, encoding='utf-8')

        # data = {'Nodes': node_name, 'topics': node_namespace}  
        # df = pd.DataFrame(data)
        # df['topics'] = df['topics'].astype('object')
        # df.loc[0, 'topics'] = topic_list
        # print(df)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
if __name__ == "__main__":
    main()


