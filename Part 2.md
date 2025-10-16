
> [!Info] Meta
> Notes prepared by [Tanay Shah | LinkedIn](https://www.linkedin.com/in/tanay-shah-/)

# Installation and understanding Gazebo version
- Various Gazebo versions are supported with varying amount of support with different ROS versions
![[Pasted image 20251014075242.png|500]]
- For our example, we are using Fortress with ROS2 Humble inside the Devcontainer
- The focus of this example is to explain the interchangeability of simulation environments.
## If your Devcontainer is working
1. Connect to your devcontainer - ensure Docker Desktop says "Engine Running" and open folder as devcontainer
2. Run `ign gazebo` inside the terminal and open the VNC viewer (web) from Ports tab. You should see the Gazebo window
![[Pasted image 20251014082843.png|500]]
## If your Devcontainer is not working - ROS and Gazebo installation in WSL
1. Open VSCode
2. Press `ctrl+shift+p` and select **Connect to WSL**
3. ![[Pasted image 20251014075921.png|500]]
4. Press `ctrl+j` to open the terminal
5. Install ROS2 Humble from this link - [Ubuntu (deb packages) — ROS 2 Documentation: Humble documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
6. Once installation is complete, make sure to run the command in the terminal `echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc`
7. Install Gazebo Fortress from this link: [Binary Installation on Ubuntu — Gazebo fortress documentation](https://gazebosim.org/docs/fortress/install_ubuntu/)
8. Type `ign gazebo` in terminal to verify installation
## If you have version Ubuntu version 22.04 in Dual boot/VM
- Install ROS Humble from this link: [Ubuntu (deb packages) — ROS 2 Documentation: Humble documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). 
- Once installation is complete, make sure to run the command in the terminal `echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc`
- Install Gazebo Fortress from this link: [Binary Installation on Ubuntu — Gazebo fortress documentation](https://gazebosim.org/docs/fortress/install_ubuntu/)
- Type `ign gazebo` in terminal to verify installation
- Install helper libraries: `sudo apt-get install ros-${ROS_DISTRO}-ros-gz`

## If you have Ubuntu version 24.04 in Dual boot/VM
- Install ROS Jazzy from this link: [Ubuntu (deb packages) — ROS 2 Documentation: Jazzy documentation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- Once installation is complete, make sure to run the command in the terminal `echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc`
- Install Gazebo Harmonic from this link: [Binary Installation on Ubuntu — Gazebo harmonic documentation](https://gazebosim.org/docs/harmonic/install_ubuntu/)
- Type `ign gazebo` in terminal to verify installation
- Install helper libraries: `sudo apt-get install ros-${ROS_DISTRO}-ros-gz`

# Creating a package
## Command to create a ROS2 package
- First run `mkdir ~/ros2_ws`
- Open this folder in vscode

![[Pasted image 20251014095216.png]]
- run `mkdir src`
- Run the below command inside of `~/ros2_ws/src`

```bash
ros2 pkg create yolo_gazebo --build-type ament_python --node-name yolo_node
```

- `yolo_gazebo` will be the name of the package.
- node named `yolo_node` will be automatically created
## Understanding folder structure
![[Pasted image 20251014083443.png]]
- `package.xml` contains metadata about the file, `setup.py` contains information about how to build the package, and entry points for running nodes
## Editing the node file
Right now there is nothing in this file
```python
def main():
    print('Hi from yolo_gazebo.')


if __name__ == '__main__':
    main()

```
>yolo_gazebo/yolo_node.py

To test this, run the following commands
```bash
cd ~/ros2_ws # Go to the root workshop directory
colcon build
source install/setup.bash # Source all the executable files
ros2 run yolo_gazebo yolo_node.py
```
The expected output should be `Hi from yolo_gazebo`

# Creating our ROS2 nodes
## Download assets and required libraries
- Downgrade numpy version
- Download ultralytics for YOLO - `pip install ultralytics`
- Download the video file inside of resource folder:
```bash
cd ~/ros2_ws/src/yolo_gazebo/resource
git clone https://github.com/ChikkiPikki/ros2_ws_yolo_gazebo_classic_example
```

## Creating the image publisher node
- To test our setup before connecting to Gazebo, we will create an image publisher in our `image_publisher` node that will publish frames of a video on a topic
- Create the file `yolo_gazebo/image_publisher.py` and update the `setup.py` file to add this node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
from ament_index_python.packages import get_package_share_directory
import os


class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')

        # Publisher for raw image frames
        self.publisher_ = self.create_publisher(Image, '/video_frames', 10)
        self.bridge = CvBridge()

        # Locate video file inside the package
        yolo_package_share = get_package_share_directory('yolo_gazebo')
        self.video_path = os.path.join(
            yolo_package_share,
            'resource',
            'ros2_ws_yolo_gazebo_classic_example',
            'video.mp4'
        )

        self.cap = cv2.VideoCapture(self.video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open video: {self.video_path}')
            raise RuntimeError("Video not found")

        fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.timer_period = 1.0 / fps if fps > 0 else 0.03  # ~30 FPS fallback
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info(f"Publishing raw video '{self.video_path}' on /video_frames")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info('End of video reached. Restarting...')
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return

        # Convert OpenCV frame to ROS2 Image message (no compression)
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)

    def destroy(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Video publisher stopped')
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

```
>yolo_gazebo/image_publisher.py

Update setup.py
```python
data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install the video resource folder
        ('share/' + package_name + '/resource/ros2_ws_yolo_gazebo_classic_example', [
            'resource/ros2_ws_yolo_gazebo_classic_example/video.mp4'
        ]),
    ],
...    
entry_points={
        'console_scripts': [
            'yolo_node = yolo_gazebo.yolo_node:main', # Add comma here
            'image_publisher = yolo_gazebo.image_publisher:main' # Add this line
            
        ],
    }
```
>setup.py

- Now go back to the root directory and run `colcon build`
- Since we have created a new executable, we will need to source again

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 run yolo_gazebo image_publisher
```

- You'll be able to view this published image inside of `rviz`. Type `rviz2` in the terminal, create add, select "by topic", add image
## YOLO integration
### Install dependencies:
```shell
pip install "numpy<2" opencv-python
pip instal ultralytics
```

### New node - yolo_node
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import time

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        # Load YOLOv8n (lightest model)
        self.get_logger().info("Loading YOLOv8n model...")
        self.model = YOLO('yolov8n.pt')  # Automatically downloads if missing

        # CV Bridge for converting ROS <-> OpenCV
        self.bridge = CvBridge()

        # Subscriber for incoming raw frames
        self.subscription = self.create_subscription(
            Image,
            '/video_frames',
            self.listener_callback,
            10)
        self.subscription  # prevent unused var warning

        # Publisher for processed (annotated) frames
        self.publisher_ = self.create_publisher(
            Image,
            '/video_frames/yolo_result',
            10)

        self.get_logger().info("YOLO node initialized and ready.")

    def listener_callback(self, msg: Image):
        try:
            # Convert ROS Image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Failed to convert image: {e}")
            return

        start_time = time.time()

        # Run YOLO inference
        results = self.model.predict(source=frame, verbose=False)

        # Draw detections
        annotated_frame = results[0].plot()

        # Convert back to ROS Image
        out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        out_msg.header = msg.header

        # Publish annotated frame
        self.publisher_.publish(out_msg)

        fps = 1.0 / (time.time() - start_time)
        self.get_logger().info(f"Published YOLO result ({fps:.2f} FPS)")

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

- You can now view this image inside of `rviz`
![[Pasted image 20251014131140.png|700]]


> [!Warning] Alternative
> If you are unable to run YOLO, you can test a simple computer vision algorithm that comes built-in with openCV - Edge Detection
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class YoloNode(Node):  # keeping the same node name
    def __init__(self):
        super().__init__('yolo_node')

        # CV Bridge for converting ROS <-> OpenCV
        self.bridge = CvBridge()

        # Subscriber for incoming raw frames
        self.subscription = self.create_subscription(
            Image,
            '/video_frames',
            self.listener_callback,
            10)
        self.subscription  # prevent unused var warning

        # Publisher for processed (edge-detected) frames
        self.publisher_ = self.create_publisher(
            Image,
            '/video_frames/yolo_result',  # keep same topic
            10)

        self.get_logger().info("Edge Detection node initialized and ready.")

    def listener_callback(self, msg: Image):
        try:
            # Convert ROS Image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Failed to convert image: {e}")
            return

        start_time = time.time()

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply Canny edge detection
        edges = cv2.Canny(gray, threshold1=50, threshold2=150)

        # Convert single-channel edge image to 3-channel BGR
        annotated_frame = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

        # Convert back to ROS Image
        out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        out_msg.header = msg.header

        # Publish annotated frame
        self.publisher_.publish(out_msg)

        fps = 1.0 / (time.time() - start_time)
        self.get_logger().info(f"Published edge-detected frame ({fps:.2f} FPS)")

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

> [!Example] Applications of this
> This feature of ROS is incredibly useful in getting video streams out of simulations, and running algorithms on them. 
> Modularity is the primary reason why we're keeping the YOLO and `image_publisher` separate
> In the next section, you'll see how we have used the same YOLO node to connect with the Gazebo camera output
> **Example use case:** Robofest problem statement - swarm based mine detection and safe navigation - done by fine-tuning YOLO on a synthetic dataset
> ![[Pasted image 20251014132219.png|500]]



# Getting started with Gazebo
## Installing the template repository
- Navigate to the `src` directory and clone the template repository for Gazebo (source: [Guide to ros_gz_project_template for ROS 2 and Gazebo Development — Gazebo fortress documentation](https://gazebosim.org/docs/fortress/ros_gz_project_template_guide/))
```bash
cd ~/ros2_ws/src
git clone https://github.com/gazebosim/ros_gz_project_template
cd ros_gz_project_template
git checkout fortress # for Fortress users. For harmonic users, refer the template repository given in the docs
```
- Navigate to the root directory, build the project and source it, then run it
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch ros_gz_example_bringup diff_drive.launch.py 
```
- Open the VNC Web view and see the Gazebo simulation
- Before running, ensure no other instance of Gazebo is active
- You can view the active topics using `ros2 topic list` or the `rqt_graph`
## rviz2
- Open Rviz2 by typing `rviz2` in the terminal
```bash
rviz2
```
- You can add various items by topic to view them in the window

> [!Warning] "Playing" before running
> Make sure to play the Gazebo simulation before observing topics. The below will not work, you will need to click the play button.
> ![[Pasted image 20251014165654.png]]

## Editing the simulation environment
- Inside the template directory, you'll be able to see 4 packages - application, bringup, description, gazebo
- Application contains general files and configs
- Bringup contains code to launch various simulations
- Description contains information about the robot
- Gazebo contains information about the simulated gazebo environemt

- Inside of `ros_gz_example_gazebo/worlds`, you will find a `diff_drive.sdf` file. SDF stands for simulation description format. 
- You can edit this file to make changes to the environement. We'll add a box to the environment by editing this file
```xml
	<model name="box">
      <pose>2 0 0.5 0 0 0</pose> <!-- x y z roll pitch yaw -->
      <static>false</static>

      <link name="box_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>  <!-- 1 m cube -->
            </box>
          </geometry>
          <surface>
            <contact>
              <collide_without_contact>false</collide_without_contact>
            </contact>
          </surface>
        </collision>

        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.2 0.4 0.8 1</ambient>
            <diffuse>0.2 0.4 0.8 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>
```

- To see changes in the environment, you can run the build command again
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch ros_gz_example_bringup diff_drive.launch.py 
```

## Editing the robot description file
We'll now add a camera to our robot.
- Inside of `ros_gz_example_description/models/diff_drive`, you'll be able to find `model.sdf`. This file describes the structure of our bot.
- An interesting snippet is plugins:
```xml
    <plugin
      filename="ignition-gazebo-physics-system"
      name="ignition::gazebo::systems::Physics">
    </plugin>
    <plugin
      filename="ignition-gazebo-sensors-system"
      name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin
      filename="ignition-gazebo-scene-broadcaster-system"
      name="ignition::gazebo::systems::SceneBroadcaster">
    </plugin>
    <plugin
      filename="ignition-gazebo-user-commands-system"
      name="ignition::gazebo::systems::UserCommands">
    </plugin>
    <plugin
      filename="BasicSystem"
      name="ros_gz_example_gazebo::BasicSystem">
    </plugin>
    <plugin
      filename="FullSystem"
      name="ros_gz_example_gazebo::FullSystem">
    </plugin>
```
>model.sdf
- In this, `BasicSystem` and `FullSystem` are our custom plugins. They are used for system configurations
- You can similarly write your own plugins and compile them for your Gazebo simulations. However, that is beyond the scope of this tutorial
### ROS-Gazebo bridge
- Ignition Gazebo also has its own topics. The ROS-Gazebo bridge broadcasts them as ROS2 topics
- The file at `ros_gz_project_template/ros_gz_example_bringup/config/ros_gz_example_bridge.yaml` shows how Ignitions topics are being remapped to compatible ROS topics
- Example - we'll add an IMU sensor to the robot using plugins and view its output over ROS
	1. Inside `ros_gz_project_description/model.sdf`, add the following lines of code:
		```xml
		<sdf version="1.8">
    <model name='diff_drive'>
      <link name='chassis'>
			<sensor name="imu_sensor" type="imu">
				<always_on>1</always_on>
				<update_rate>1</update_rate>
				<visualize>true</visualize>
				<topic>imu</topic>
			</sensor>
			...
		```
	2. Now update the ROS-Gazebo bridge configuration file and the following lines (`ros_gz_example_bridge.yaml):
		```yaml
		- gz_topic_name: "/imu"
		  ros_topic_name: "/imu"
		  ros_type_name: "sensor_msgs/msg/Imu"
		  gz_type_name: "gz.msgs.IMU"
		  direction: GZ_TO_ROS
		```

	3. Update the `<world>` tag inside of  `ros_gz_example_gazebo/worlds/diff_drive.sdf` and include the plugin
		```xml
		<sdf version="1.8">
	  <world name="demo">
		    <plugin filename="libignition-gazebo-imu-system.so"
	        name="ignition::gazebo::systems::Imu">
	    </plugin>
	    ...
		```
4. Now type `ros2 topic list`. You should be able to see `/imu`. You can also try typing `ign topic -l` to see available topics being published from Gazebo
5. You can add the IMU topic inside of Rviz to visualize it


- You can also explore more sensors on this link : [Sensors — Gazebo fortress documentation](https://gazebosim.org/docs/fortress/sensors/)

> [!Question] Task - Add camera link
> Explore available sensors and add camera to the simulation. (Fun fact - AI tools don't give correct changes (I have tried all). You'll have to go through the documentation :) )
> Steps to update simulation:
> 1. Add sensor to model
> 2. Update ROS-Gazebo bridge configuration
> 3. Update the world tag to include the plugin


> [!info] Insight
> If you publish the camera topic on the same topic as you published the video and run the YOLO node, the algorithm is still performed on it. This is an example of how modular architecture helps development

___
# Mini project - Lidar based obstacle detection and stopping
## **Goal**: Keep car moving forward. Stop if an obstacle is detected within 1 meter range in any direction

> [!Info] This project is a codealong!
> I'll be coding alongside with you guys. 
> >
>Control logic of code 
>1. [ ] Make a custom subscriber and subscribe to the Lidar topic
>2. [ ] Check the interface of the LaserScan data type.
>3. [ ] Loop over all provided ranges and their respective angles and check the minimum one
>4. [ ] Move forward if all ranges are less than 1. Publish to /diff_drive/cmd_vel (you will need to check its interface as well)
>   Bonus: If all ranges are less than 1, change the /diff_drive/cmd_vel to move in direction of maximum distance 

