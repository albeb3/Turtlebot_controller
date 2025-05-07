 # 📦 turtlebot_controller

## 📖 Introduction

`turtlebot_controller` is a ROS Noetic package designed to control the `turtlesim` robot through custom C++ nodes. The package includes:

- **`turtle` class**: This class provides methods to move the robot to specific positions without using the pen and also offers functionality for aligning the robot to a target and drawing straight lines.
- **`draw` class**: This class includes methods to draw certain shapes while considering the window's limits.

This package is designed for **ROS Noetic** running on **Ubuntu 20.04** and is useful for learning about ROS publishers, subscribers, actions, and the control of a simple robot.

---


## 🔧 Setup

### 1. Crea un workspace:

```bash
mkdir -p ~/my_ros/src
cd ~/my_ros/
catkin_make
```
### 2. Source the workspace
Add this line to your ~/.bashrc:
source ~/my_ros/devel/setup.bash
then apply it:
source ~/.bashrc

### 3. Clone this repository 
cd ~/my_ros/src
git clone <REPO_URL>
### 4. Build the workspace
cd ~/my_ros
catkin_make

## 📦 Dependencies

Make sure the following dependencies are installed:

- `turtlesim`
- `actionlib`

If they are not installed, you can install them by running:

```bash
sudo apt install ros-noetic-turtlesim ros-noetic-actionlib
```
## 🚀 Usage

To launch the package and run the nodes, use the following commands:

1. Launch the package with a launch file:

```bash
roslaunch turtlebot_controller turtle_class.launch
```
2. run the node:

```bash
rosrun turtlebot_controller tutorial_draw_node 
```
##✍️ Author
Alberto Bono – alberto_bono@hotmail.it

> ⚠️ This project may include code or content generated with the assistance of AI tools (e.g., ChatGPT by OpenAI). All generated outputs have been reviewed and adapted by the author. If any part unintentionally infringes copyright, please contact me and I will promptly make the necessary corrections.


