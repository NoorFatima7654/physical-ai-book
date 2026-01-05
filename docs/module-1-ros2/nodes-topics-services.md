---
sidebar_position: 3
title: "ROS 2 Nodes, Topics, Services (simple explanation + examples)"
---

# ROS 2 Nodes, Topics, Services (simple explanation + examples)

## Overview

ROS 2 uses a distributed computing model where different processes (nodes) communicate with each other through standardized interfaces. Understanding these communication patterns is essential for building robot applications.

## Nodes

Nodes are individual processes that perform specific functions. Each node typically handles one aspect of robot behavior:

### Humanoid Robot Examples:
- A **head camera node** captures visual data for the humanoid's "vision" (`/head_camera/image_raw`)
- A **joint controller node** manages the positions of the humanoid's arm joints (`/arm_controller/commands`)
- A **balance control node** processes IMU data to maintain the humanoid's stability (`/balance_controller/imu_data`)
- A **path planning node** calculates walking routes for the humanoid (`/path_planner/goal`)
- A **speech synthesis node** converts text to speech for the humanoid's "voice" (`/speech_synthesizer/speak`)

In humanoid robotics, nodes often represent different body parts or cognitive functions, making the robot's behavior modular and manageable.

### Creating a Simple Node

```
# Basic structure of a ROS 2 node
import rclpy
from rclpy.node import Node

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('my_robot_node')
        # Node initialization code here

def main(args=None):
    rclpy.init(args=args)
    node = MyRobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics

Topics are channels for continuous data streaming between nodes. Data flows from publishers to subscribers:

### Humanoid Robot Examples:
- A **head camera node** publishes visual data to `/head_camera/image_raw` which can be subscribed to by object detection nodes
- A **joint state publisher** broadcasts the current positions of all humanoid joints to `/joint_states`
- A **laser scanner node** publishes distance measurements to `/scan` for obstacle detection
- A **robot state publisher** broadcasts the full robot state to `/robot_state` for visualization in RViz
- A **microphone node** publishes audio data to `/audio` for speech recognition

Multiple nodes can subscribe to the same topic, enabling the same data to be processed by multiple systems simultaneously. For example, a humanoid's camera data might be used simultaneously by perception, navigation, and manipulation systems.

### Key Characteristics:
- Publishers send data to a topic (e.g., a camera node publishing images to `/head_camera/image_raw`)
- Subscribers receive data from a topic (e.g., an object detection node subscribing to `/head_camera/image_raw`)
- Multiple nodes can subscribe to the same topic
- Multiple nodes can publish to the same topic (though this is less common)

### Example: Publisher Node

```
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'robot_status', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello Robot {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1
```

### Example: Subscriber Node

```
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'robot_status',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
```

## Services

Services provide request-response communication for one-time interactions:

### Humanoid Robot Examples:
- A **navigation service** that accepts a destination and returns whether the humanoid can navigate there (`/navigate_to_pose`)
- A **gripper control service** that accepts a command and returns the result of opening/closing the humanoid's hand (`/gripper_controller/gripper_cmd`)
- A **speech service** that accepts text and returns when the humanoid has finished speaking (`/tts_service/speak`)
- A **joint calibration service** that resets the humanoid's joint positions to known values (`/calibrate_joints`)
- A **robot reset service** that returns the humanoid to a safe home position (`/reset_robot`)

Services are synchronous and blocking, making them ideal for operations that must complete before the requesting node can continue.

### Key Characteristics:
- A node sends a request and waits for a response
- Examples: saving a map, changing robot state, requesting specific calculations
- Services are synchronous and blocking

### Example: Service Server

```
from add_two_ints_srv.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {response.sum}')
        return response
```

### Example: Service Client

```
import rclpy
from rclpy.node import Node
from add_two_ints_srv.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

    def send_request(self, a, b):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

## Actions

Actions are used for long-running tasks that provide feedback and can be canceled:

### Humanoid Robot Examples:
- **Walking to a location**: The humanoid receives a destination and provides continuous feedback on progress while walking (`/move_base_flex`)
- **Arm manipulation**: The humanoid moves its arm to grasp an object, providing feedback on joint positions and gripper status (`/arm_controller/follow_joint_trajectory`)
- **Speech synthesis**: The humanoid speaks a phrase, providing feedback on progress and allowing interruption (`/speak_action`)
- **Object recognition**: The humanoid analyzes its environment, providing continuous feedback on detected objects (`/object_detection_action`)
- **Balancing**: The humanoid maintains balance while performing tasks, providing feedback on stability metrics (`/balance_controller/balance_action`)

Actions are ideal for humanoid robotics because they allow for long-running behaviors with real-time feedback and the ability to cancel operations if needed.

### Key Characteristics:
- Navigation to a goal
- Manipulation tasks
- Long computations

## Best Practices

- Keep nodes focused on single responsibilities
- Use descriptive names for topics and services
- Handle errors gracefully
- Use appropriate quality of service settings
- Consider timing and synchronization in your system design

## Summary

Understanding nodes, topics, and services forms the foundation of ROS 2 programming. These concepts allow you to build modular, maintainable robot applications where different components can communicate effectively.

In humanoid robotics, these concepts are particularly important because a humanoid robot is essentially a complex system of interconnected parts that must work together seamlessly. Each body part (arms, legs, head) can be controlled by separate nodes, while sensory information (vision, touch, balance) flows through topics to perception systems. Services and actions enable coordinated behaviors like walking, grasping, and speaking.

This modular approach allows humanoid robot developers to work on individual components without affecting the entire system, making development and debugging much more manageable.