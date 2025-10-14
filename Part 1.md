
First Half - Creating your own publisher & subscriber

---
### Prepared By:    Ishaan Kathiriya                                  
#### [Linkedin](https://www.linkedin.com/in/ishaan-kathiriya/)
---

Before we start writing any code, we need to do a bit of organization.
Node will exists within packages and all your packages will exists within a ROS2 'workspace'

ROS2 **workspace**:  A workspace is nothing more than a folder organization in which you
will create and build your packages. Your entire ROS 2 application will live within this workspace.


## Creating a workspace

To create a workspace, you simply need to create a new directory inside your home directory.

For the workspace's name, let's just keep it as `ros2_ws` for now.

In your terminal:
```bash
mkdir ros2_ws
```

This is pretty much what a workspace is! You just create a new directory inside which you'll create all your application

Before we proceed, you need to create a sub-directory `src`  inside the ros2 workspace you created. This is where we will be creating all our packages

```bash
cd ros2_ws/
mkdir src
```


## Building your workspace

```bash
cd ros2_ws
colcon build
```


Remember to build from the root of your workspace only!

-> After building, you'll see 3 new directories appear - build, install, log


## Sourcing your workspace

Every time you build your workspace, you have to source it so that the environment (the session you are in) knows about the new changes in the workspace.

To source the workspace, source the setup.bash script:
```bash
source ~/ros2_ws/install/setup.bash

----- OR ----

cd ros2_ws
source install/setup.bash

```



---

## Creating a Package


Any node you create will exist within a package. Hence, to create a node, you first have to create a package (inside your workspace)

- Packages can be of two types - Python packages or C++ packages. 

For the sake of convenience, we'll be making Python packages for now.




### Creating a Python Package 

Make sure you're in the src directory


```bash
cd ros2_ws/src
```



Command to create a package:

Template:
```bash
ros2 pkg create <pkg_name> --build-type <build_type> --dependencies <list_of_dependencies_separated_with_spaces>
```



Now, let's create our first python package:
```bash
ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy
```


With this command, we say that we want to create a package named my_py_pkg, with the ament_python build type, and we specify one dependency: rclpy—this is the Python library for ROS 2 that you will use in every Python node


You'll now see a lot of files that are created in your src directory. Not all of them are important for now.


A quick overview of the most important files/directories:

**my_py_pkg** : As you can see, inside the package, there is another directory with the same name. 
This directory already contains an__init__.py file. This is where we will create our Python nodes.

**package.xml**:  Every ROS 2 package (Python or C++) must contain this file. We will use it to provide more information about the package as well as dependencies.

**setup.py**: This is where you will write the instructions to build and install your Python nodes.




## Building a Package:

Now that you’ve created one or more packages, you can build them, even if you don’t have any nodes in the packages yet.

Go back to the root of your workspace! and run colcon build

```bash
cd ros2_ws
colcon build
source install/setup.bash
```



---


## Creating a Python Node


You will organize your nodes inside packages. For one package (sub-part of your application), you can have several nodes (functionalities). To fully understand how to organize packages and nodes, you need practice and experience.


```bash
cd ~/ros2_ws/src/my_py_pkg/my_py_pkg/

touch my_first_node.py   # (OR you can simply create a file using VS Code)

chmod +x my_first_node.py
```





## Writing A minimal ROS2 python node


```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyCustomNode(Node):
	def __init__(self):
		super().__init__('my_node_name')
		
def main(args=None):
	rclpy.init(args=args)
	node = MyCustomNode()
	rclpy.spin(node)
	rclpy.shutdown()
	

if __name__ == '__main__':
	main()
```



NOTE: The node is not doing anything right now. We'll add functionalities soon, but the above template for a ROS2 python node remains constant.


```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyCustomNode(Node):
	def __init__(self):
		super().__init__('my_node_name')
		self.get_logger().info("Hello World")
		
def main(args=None):
	rclpy.init(args=args)
	node = MyCustomNode()
	rclpy.spin(node)
	rclpy.shutdown()
	

if __name__ == '__main__':
main()
```



## Building the node


You are now going to build the node so that you can run it.

You might think: why do we need to build a Python node? Python is an interpreted language; couldn’t we just run the file itself?

Yes, this is true: you could test the code just by running it in the terminal ($ python3
my_first_node.py). 

However, what we want to do is actually install the file in our workspace, so we
can start the node with ros2 run, and later on, from a launch file.



To 'build' we need to do one more thing in the package.

Open the setup.py from the my_py_pkg package. Locate entry_points and 'console_scripts' at the end of the file. For each node we want to build, we have to add one line inside the 'console_scripts' array:

syntax:
```python
entry_points={
'console_scripts': [
	"test_node = my_py_pkg.my_first_node:main"
],
},
```


Template:
```
<executable_name> = <package_name>.<file_name>:<function_name>
```


There are a few important things to correctly write this line:

- First, choose an executable name. This will be the name you use with `ros2 run <pkg_name> <executable_name>`

- For the filename, skip the .py extension.

- The function name is main, as we have created a main() function in the code.

- If you want to add another executable for another node, don’t forget to add a comma between each executable and place one executable per line.


**Node name**: defined inside the code, in the constructor. This is what you’ll see with the ros2 node list, or in rqt_graph.

**Filename**: the file where you write the code.

**Executable name:** defined in setup.py and used with ros2 run.


Now, let's build  & source our node:

```bash
cd ros2_ws
colcon build
source install/setup.bash
```



After this, let's run our simple logger node:

```bash
ros2 run my_py_pkg test_node
```






## Improving the node - Timer & Callback


Our node is printing one piece of text when it’s started. We now want to make the node print a string every second, as long as it’s alive.


This behavior of “doing X action every Y seconds” is very common in robotics. For example, you
could have a node that “reads a temperature every 2 seconds”, or that “gives a new motor command every 0.1 seconds”.


To achieve the above goal, we will: add a **timer** to our node. A timer will trigger a **callback** function at a specified rate.



We just need to modify the `MyCustomNode` class, everything else stays the same

```python
class MyCustomNode(Node):
	def __init__(self):
		super().__init__('my_node_name')
		self.counter = 0
		self.timer = self.create_timer(1.0, self.print_hello)
		
	def print_hello(self):
		self.get_logger().info("Helloo" + str(self.counter))
		self.counter +=1
	
		
```


The important line is the `create_timer()`  method.

We need to give two arguments: 
- The rate at which we want to call the function (float number), 
- And the callback function. Note that the callback function should be specified without any parenthesis.



This instruction means that we want to call the print_hello method every 1.0 second.



-> Now, build & source the node and try to run it.


---

## Writing a Topic Publisher


The code for this application will be super minimalistic so that you get an idea on how to create custom publisher and subscriber.


**What we want to do for now is publish a number on a topic. This topic is new and we will *create* it.**

 **You don’t really create a topic - you create a publisher or a subscriber to that topic. This will automatically create the topic name, which will be registered**



To write a publisher, we need a node.



--> Inside the same package, where you created your first node, create another node named like: `number_publisher`

eg:
```bash
cd ros2_ws/src/my_py_pkg/my_py_pkg/
touch number_publisher.py
chmod +x number_publisher.py
```


Our node template:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class NumberPublisherNode(Node):
	def __init__(self):
		super().__init__('number_publisher')
		
def main(args=None):
	rclpy.init(args=args)
	node = NumberPublisherNode()
	rclpy.spin(node)
	rclpy.shutdown()
	

if __name__ == '__main__':
	main()
```




#### Adding a publisher to the node

Where can we create a publisher in this node? We will do that in the constructor.

Before we proceed, we need to decide what the **name** and the **interface** of the topic will be:

**Case 1**: You’re publishing to a topic that already exists (other publishers or subscribers on that topic), and then you use the same name and interface

**Case 2**: You create a publisher for a new topic (what we are doing now), and then you have to choose a name and interface



Let's name our topic as **number**

For the interface, you have two choices: use an existing interface or create a custom one. To get
started, we will use an existing interface

For now, I have provided the exact interface we will be using, later you can explore other existing interfaces ROS2 provides

**Interface** :     `example_interfaces/msg/Int64`


check the interface
```bash
ros2 interface show example_interfaces/msg/Int64
```





Now that we have this information, let’s create the publisher. First, import the interface, and then create the publisher in the constructor:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class NumberPublisherNode(Node):
	def __init__(self):
		super().__init__('number_publisher')
		self.number_publisher = self.create_publisher(Int64, "number", 10)
```



`create_publisher()`  :

- **Topic interface**: We’ll use **Int64** from the example_interfaces package.

- **Topic name**: As defined previously, this is **number**

- **Queue size**: If the messages are published too fast and subscribers can’t keep up, messages will be buffered (up to 10 here) so that they’re not lost. This can be important if you send large messages (images) at a high frequency, on a lossy network. As we get started, there’s no need to worry about this; I recommend that you just set the queue size to 10 every time.




-> Now, a publisher won't publish automatically on a topic, we need to write the code for it




### Publishing with a Timer


A common behavior in robotics is to do X action every Y seconds. For example, publish an image
from a camera every 0.5 seconds, or in this case, publish a number on a topic every 1.0 second

```python
def __init__(self):
	super().__init__('number_publisher')
	self.number = 2
	self.number_publisher = self.create_publisher(Int64, "number", 10)
	self.number_timer = slef.create_timer(1.0, self.publisher_number)
	self.get_logger().info("Number publisher has been started: \n")
		
def publish_number(self):
	msg = Int64()
	msg.data = self.number
	self.number_publisher.publish(msg)
	
```


In the publish_number() method, we publish on the topic:
1. We create an object from the **Int64** class. This is the interface - in other words, the message to send.
2. This object contains a **data** field. How do we know this? We found this previously when we ran `ros2 interface show example_interfaces/msg/Int64`. Thus, we provide a number in the **data** field of the message. For simplicity, we specify the same number every time we run the callback function.
3. We publish the message using the publish() method from the publisher.




### Building the publisher

-> We used a new dependency - `example_interfaces`  package. 
Since all our dependencies are mentioned in the package.xml file, we need to add this new dependency too.


In your package.xml:
```xml
<depend>example_interfaces</depend>
```


In your setup.py:
```python
entry_points={
	'console_scripts': [
		"test_node = my_py_pkg.my_first_node:main",
		"number_publisher = my_py_pkg.number_publisher:main",
	],
},
```


Now build your publisher and run it.


Subscriber to the topic directly from the terminal :
```bash
ros2 topic echo /number
```





---

### Writing a Python Subscriber


1) Create a node 

Now,


```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class NumberCounterNode(Node):
	def __init__(self):
		super().__init__("number_counter")
		self.counter_ = 0
		self.number_subscriber_ = self.create_subscription(Int64, "number",
		self.callback_number, 10)
		self.get_logger().info("Number Counter has been started.")
		
	
	def callback_number(self, msg: Int64):
	self.counter_ += msg.data
	self.get_logger().info("Counter: " + str(self.counter_))
```



`create_subscription()`

**Topic interface**: Int64. This needs to be the same for both the publisher and subscriber.

**Topic name**: number. This is the same name as for the publisher. Note that I don’t provide any additional slash here. This will be added automatically, so the topic name will become /number.

**Callback function**:  We use a callback method for the subscriber here as well. When the node is spinning, it will stay alive and all registered callbacks will be ready to be called. Whenever a message is published on the /number topic, it will be received here, and we will be able to use it and process it inside the callback method (that we need to implement)

**Queue size**: As seen previously, you can set it to 10 and forget about it for now.



`callback_number()`   :

In a subscriber callback, you receive the message directly in the parameters of the function. Since we know that **Int64** contains a data field, we can access it with **msg.data**.

Now, we add the received number to a counter_ attribute and print the counter every time with a ROS2 log.




-> Now, finally, add the executable for the subscriber in setup.py:

```python
entry_points={
	'console_scripts': [
	"test_node = my_py_pkg.my_first_node:main",
	"number_publisher = my_py_pkg.number_publisher:main",
	"number_counter = my_py_pkg.number_counter:main"
	],
},
```



Then, finally build & source the workspace and run your node




---

**Note**: Just like you can listen to a topic directly from the terminal, you can also publish to a topic straight from the terminal

These things are very crucial when developing large ROS2 applications where you don't want to create a node and go through the entire process just to check whether your current application is running.


To publish to a topic directly from the terminal, follow this Template:
```bash
ros2 topic pub -r <frequency> <topic_name> <interface> <message_in_json>
```



Example:
```bash
ros2 topic pub -r 2.0 /number example_interfaces/msg/Int64 "{data: 7}"
```


---

### Additional Content:

