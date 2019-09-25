# Capstone - System Integration

This is the project repo includes my solution for the capstone project of the Udacity Self-Driving Car Nanodegree. 
This System Integration project was mainly done using ROS and included 4 main components:
1. Waypoint Updating
2. Controls
3. Traffic Light Detection
4. Traffic Light Classification

### Waypoint Updating

To update the waypoints, I localized the image on the map. Based on the distances and relative position of the waypoints in the map, the closest waypoint in direction of the road was found.

This waypoint and the consecutive 50 waypoints were used for path planning.

After the traffic light detection and classification algorithm have been created, their output was used to change the desired velocity of the waypoints.
If the light was red, the velocity of all waypoints up to the stop line was decreased with the waypoint near the stopline being 0.
Once the light turned green, the speed of all waypoints was set to 10 m/s again.

### Controls

The controls was broken down in lateral and longitudinal controls.  
The reference signal were provided by Autoware's waypoint_follower node.

For longitudinal control, a PID controller was used. The PID controller is reset every time there is a driver takeover.
The velocity value used for longitudinal controller was parsed through a low pass filter to decrease noise.
For lateral control, the provided Yaw controller has been implemented.

While the controllers can be found in the twist_controller.py file, the dbw_node.py includes the node that publishes the torque, brake, and steering controls to the vehicle.

### Traffic Light Detection

The information about the pose of the vehicle was used to find the position of the nearest traffic light according to the map.
If the traffic light is within 50 meters of the vehicle and ahead of the vehicle, the image from the camera is parsed to the Traffic sign classification algorithm.
If the traffic light is classified as red twice in a row, the waypoint index of the stop line associated with the traffic light will be published.
This topic will be used by the waypoint_updater to adjust the reference velocity of the waypoints. 

### Traffic Light Classification

After unsuccessfully trying to train a CNN on the entire camera image for classification, I changed my strategy and decided that transfer learning might be a better approach. 
I retrained a faster RCNN model (faster_rcnn_inception_v2_coco_2019_01_28). This network is part of the the Tensorflow Detection API that can be found in this [repository](https://github.com/tensorflow/models).
I trained the model for 20000 steps. I used the dataset kindly made available by [Vatsal Srivastava](https://github.com/coldKnight).
I also want to thank [Alex Lechner](https://github.com/alex-lechner) for his great tutorial on Traffic Light Classification using the Tensoflow Detection API.
The repository can be found [here](https://github.com/alex-lechner/Traffic-Light-Classification). His tutorial was extremely useful.
Furthermore, I used the following repositories and articles to learn about traffic light classification:

* [Article 1 ](https://medium.com/@anthony_sarkis/self-driving-cars-implementing-real-time-traffic-light-detection-and-classification-in-2017-7d9ae8df1c58)
* [Article 2 ](https://towardsdatascience.com/faster-r-cnn-object-detection-implemented-by-keras-for-custom-data-from-googles-open-images-125f62b9141a)
* [Article 3 ](https://medium.com/@UdacityINDIA/self-driving-vehicles-traffic-light-detection-and-classification-with-tensorflow-object-detection-d6a4d25e99c2)


## Team Members
While I did not work with other student's on the project, I do want to thank my Co-workers who have been a great resource.
Thank you to [Sagar](https://www.linkedin.com/in/sagarmanglani) for helping me fix latency issues with the simulator.
And thank you to [Lily](https://www.linkedin.com/in/alchemz) for helping me set up the environment for training the Traffic Light Classifier.

Name: Philipp Waeltermann  
Email: pwaelte1@ford.com


## Instructions for Running the code

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the "uWebSocketIO Starter Guide" found in the classroom (see Extended Kalman Filter Project lesson).

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Other library/driver information
Outside of `requirements.txt`, here is information on other driver/library versions used in the simulator and Carla:

Specific to these libraries, the simulator grader and Carla use the following:

|        | Simulator | Carla  |
| :-----------: |:-------------:| :-----:|
| Nvidia driver | 384.130 | 384.130 |
| CUDA | 8.0.61 | 8.0.61 |
| cuDNN | 6.0.21 | 6.0.21 |
| TensorRT | N/A | N/A |
| OpenCV | 3.2.0-dev | 2.4.8 |
| OpenMP | N/A | N/A |

We are working on a fix to line up the OpenCV versions between the two.
