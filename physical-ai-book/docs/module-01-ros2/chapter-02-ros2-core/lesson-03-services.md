--- 
id: lesson-03-services
title: Lesson 3 - ROS 2 Services
sidebar_position: 3
description: Learn how to use ROS 2 services for synchronous, one-to-one communication between nodes.
---

# Lesson 3: ROS 2 Services

## Lesson Objective

By the end of this lesson, you will be able to write a ROS 2 service client and server in Python using `rclpy` and use them to implement a request-response communication pattern.

## Prerequisites

- Completion of Lesson 2: ROS 2 Topics

## Concept Explanation

ROS 2 services are a communication mechanism for synchronous, one-to-one communication between nodes. A node that provides a service is called a "service server," and a node that uses a service is called a "service client." When a client calls a service, it sends a request message to the server and waits for a response. This is a great way to implement a remote procedure call (RPC) pattern in a ROS 2 system.

## Step-by-Step Technical Breakdown

1.  **Define a service type**: We will use a standard ROS 2 service type to define the structure of our request and response messages.
2.  **Write a service server**: We will write a Python node that provides a service.
3.  **Write a service client**: We will write another Python node that calls the service.
4.  **Run the nodes**: We will run both nodes and see the client call the service and receive a response from the server.

## Real-World Analogy

Think of a ROS 2 service as a restaurant. The service server is the chef, who can prepare a variety of dishes. The service client is a customer, who can order a dish from the menu. When the customer orders a dish, they send a request to the chef and wait for the dish to be prepared and served.

## Hands-On Task

Follow the step-by-step technical breakdown to create a service client and server and use them to implement a request-response communication pattern.

## Python + ROS2 Code Example

**Service Server:**
```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MyServiceServer(Node):
    def __init__(self):
        super().__init__('my_service_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}')
        self.get_logger().info(f'Sending back response: {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MyServiceServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Service Client:**
```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MyServiceClient(Node):
    def __init__(self):
        super().__init__('my_service_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self):
        self.req.a = 41
        self.req.b = 1
        self.future = self.client.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    node = MyServiceClient()
    node.send_request()

    while rclpy.ok():
        rclpy.spin_once(node)
        if node.future.done():
            try:
                response = node.future.result()
            except Exception as e:
                node.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                node.get_logger().info(
                    'Result of add_two_ints: for %d + %d = %d' %
                    (node.req.a, node.req.b, response.sum))
            break

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Common Mistakes & Debugging Tips

- **Forgetting to wait for the service**: A service client must wait for the service to be available before calling it. If you don't, you will get an error.
- **Mismatched service types**: The client and server must use the exact same service type. You can use `ros2 service type <service_name>` to see the type of a service.

## Mini Assessment

1.  What are ROS 2 services used for?
2.  What is the difference between a service client and a service server?
3.  What is a service type?
4.  How can you see a list of all the active services on a ROS 2 system?
