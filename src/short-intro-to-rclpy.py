# rclpy is the main library we use to extract info from the ROS system.
# This just provides a simple list of commands which can be used to extract
# the relevant info.

import rclpy
# For more details about the rclpy library
# https://docs.ros2.org/latest/api/rclpy/api/node.html

if __name__ == "__main__":
    
    # We initialize rclpy using the init function
    rclpy.init()
    
    # Here we create a simple node
    r2p = rclpy.create_node('ros2prov_recorder')

    
    # Short delay introduced to ensure that the nodes are properly initialized before moving forward
    for i in range(10000000):
        pass
    # This can be avoided using call back functions

    # Outputting the number of Publishers associated with a topic (we pass topic name as an argument)
    print("Number of Publishers are:", r2p.count_publishers('/parameter_events'))
    # Outputting the number of Subscribers associated with a topic (we pass topic name as an argument)
    print("Number of Publishers are:", r2p.count_subscribers('/parameter_events'))

    # Getting the name of a node (Note, we use the node object to retrieve its name)
    print("The name of the node is", r2p.get_name())
    # Getting the namespace of a node (Note, we use the node object to retrieve its namespace)
    print("The namespace of the node is", r2p.get_namespace())

    # Printing all the node names
    # This command gets the name of all the ROS Nodes in the ROS system
    print('\n--------------\nRetrieving and printing all the Node Names')
    nodes = r2p.get_node_names()
    print(*nodes, sep=', ')

    # Printing all the node names along with their respective namespaces
    print('\n--------------\nRetrieving and printing all the Node Names along with Name spaces')
    nodes_n_namespaces = r2p.get_node_names_and_namespaces()
    print(*nodes_n_namespaces, sep=', ')

    # Printing the topics to which a node publishes
    print('\n--------------\nRetrieving and printing all the topics to which a node publishes')
    for n, ns in nodes_n_namespaces:
        pub_topics = r2p.get_publisher_names_and_types_by_node(n, ns)
        print('\n' + ns + n + ':')
        for elem, type in pub_topics:
            print(elem, ':', type)

    # Printing the topics to which a node subscribes
    print('\n--------------\nRetrieving and printing all the topics to which a node subscribes')
    for n, ns in nodes_n_namespaces:
        sub_topics = r2p.get_subscriber_names_and_types_by_node(n, ns)
        print('\n' + ns + n + ':')
        for elem, type in sub_topics:
            print(elem, ':', type)

    # Printing the list of server service topics for a given node
    print('\n--------------\nRetrieving and Printing the list of server service topics for a given node')
    for n, ns in nodes_n_namespaces:
        services = r2p.get_service_names_and_types_by_node(n, ns)
        print('\n' + ns + n + ':')
        for elem, type in services:
            print(elem, ':', type)
            
    # Printing all the topics
    print('\n--------------\nRetrieving and Printing the list of all discovered topics')
    topics = r2p.get_topic_names_and_types()
    for topic, type in topics:
            print(topic, ':', type)

    # The following commands aren't working due to improper usage, have to figure out what is the problem
    # Printing the all client names
    # print('\n--------------\nRetrieving and Printing the list of all discovered clients')
    # for n, ns in nodes_n_namespaces:
    #     clients = r2p.get_client_names_and_types_by_node(n, ns)
    #     print('\n' + ns + n + ':')
    #     for elem, type in clients:
    #         print(elem, ':', type)

    # Topic Commands
    # Publishers from the topic name
    # print('\n--------------\nRetrieving and Printing the list of all Publishers from topic name')
    # for topic, type in topics:
    #     pubs = rclpy.get_publishers_info_by_topic(topic, no_mangle=False)
    #     print(topic, ':')
    #     print(*pubs, sep=', ')

    # Subscribers from the topic name
    # print('\n--------------\nRetrieving and Printing the list of all Subscribers from topic name')
    # for topic, type in topics:
    #     pubs = r2p.get_publishers_info_by_topic(topic, no_mangle=False)
    #     print(topic, ':')
    #     print(*pubs, sep=', ')


    # The spin is initiated to prevent the program from terminating early. Ideally, we'd like this
    # to keep running as long as the ROS system is running in the background for a specified period of time
    # Call backs can be executed during this period, so having callbacks for node initializations, shutdown,
    # forming of new relations etc can be beneficial from the POV of saving time
    rclpy.spin(r2p)