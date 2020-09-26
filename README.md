## Product "Pick and Sort" with real-time cloud-based object classification using AWS SageMaker . 
<p align="center">
<img src="./media/gif_1.gif" width="600">

### 1. Introduction
This project is a simulation for a (simple) sorting scenario using a UR5 robot with an eye-in-hand setup. It consists of three main components: A web interface for placing order requests, a deployed endpoint through AWS SageMaker for real-time cloud-based inference of images of the product, and a simulation environment using ROS and Gazebo. The following are the steps starting from request till dispatch:
1. A user first requests a specific product through the web interface communicating with Gazebo through ROS (Red or Blue box)
2. The system then classifies the image of the current product in queue through a cloud-deployed endpoint in AWS Sagemaker, where the deep learning-based classification model was trained, evaluated, and deployed fully on the cloud and using data collected from Gazebo.
3. According to the decoded result from the cloud inference, the robot then picks the product up, and then either discards or dispatches the product in the orange box depending on whether it matches the user request.

A simple demo can be seen at the top

### 2. Environment Setup
The project uses [ROS Kinetic](http://wiki.ros.org/kinetic) running on [Ubuntu 16.04](http://releases.ubuntu.com/16.04/).

The following tools are used for **simulation**, motion planning:

* [Gazebo](http://gazebosim.org/): a physics based 3D simulator extensively used in the robotics world
* [MoveIt!](http://moveit.ros.org/): a ROS based software framework for motion planning, kinematics and robot control


The following tools are used for training, evaluation, and deployment of deep learning-based image **classification** model on the **cloud**:

* [AWS SageMaker](https://aws.amazon.com/sagemaker/): a cloud machine-learning platform that enables developers to create, train, and deploy machine-learning models in the cloud. 

The following tools are used for development of the **web interface**:
* [Flask](https://flask.palletsprojects.com/en/1.1.x/): a lightweight Web Server Gate Interface web application framework
* [Bootstrap](https://getbootstrap.com/): an open-source front-end framework used to create modern websites and web apps using HTML and CSS templates for UI interface


### 3. Installation and Usage

1. Install ROS Kinetic, Gazebo, OpenCV
2. Clone and build the project as follows:

```
cd ~
mkdir -p catkin_ws/src && cd ~/catkin_ws/src

git clone https://gitlab.com/mabouseif/pick_and_sort_with_aws_sagemaker.git

cd ~/catkin_ws

rosdep update
rosdep install --rosdistro kinetic --ignore-src --from-paths src

# building
catkin_make

# activate this workspace
echo "alias devel=source devel/setup.bash" >> ~/.bashrc 
source ~/.bashrc
devel
```

3. In another terminal, Create a virtual environment for the web interface:

```
cd ros_web

python3 -m venv web_env

source web_env/bin/activate
pip install -r requirements.txt

cd ros_website

python ros_interface.py
```

4. Open up the browser at http://0.0.0.0:3001/
5. Run the following to launch the simulation:
```
catkin_make && devel && roslaunch pick_and_sort main.launch
```
6. Followed by the following in another terminal 
```
catkin_make && devel && rosrun pick_and_sort main
```
7. Replace the AWS Credentials and endpoint name in `aws_sage_maker_client.py` after deploying the endpoint with the proper configuration and respective model.
8. Use the web interface to place the requests.


