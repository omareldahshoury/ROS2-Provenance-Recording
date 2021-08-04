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

        super().__init__("my_node")
        print("list of nodes")
        nodes = self.get_node_names()
        print(nodes)
        node_name = nodes[1]
        node_namespace = nodes[0]
 

        print("----------------------------------------------")
        print("list of topics")
        topics = self.get_topic_names_and_types(nodes)
        for n in topics:
            topic_list.append(n)
        print(topic_list)

        print("----------------------------------------------")
        print("list of publishers")
        time.sleep(2)
        publishers = self.get_publisher_names_and_types_by_node(node_name, "/")
        for n1 in publishers:
            publisher_list.append(n1)
        print(publisher_list)

        print("----------------------------------------------")
        print("list of subscribers")
        time.sleep(2)
        subscribers = self.get_subscriber_names_and_types_by_node(node_name, "/")
        for (n2,t2) in subscribers:
            print(n2, ":", t2)

        print("----------------------------------------------")
        print("list of service server")
        time.sleep(2)
        services = self.get_service_names_and_types_by_node(node_name, "/")
        for (n3,t3) in services:
            print(n3, ":", t3)

        print("----------------------------------------------")
        print("list of service client")
        time.sleep(2)
        clients = self.get_client_names_and_types_by_node(node_name, "/")
        for (n4,t4) in clients:
            print(n4, ":", t4)

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


        data = {'Nodes': node_name, 'topics': node_namespace}  
        df = pd.DataFrame(data)
        df['topics'] = df['topics'].astype('object')
        df.loc[0, 'topics'] = topic_list
        print(df)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
if __name__ == "__main__":
    main()


