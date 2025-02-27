# Quest2ROS

Ros 2 code to connect the occulus quest to ROS and receive the controler Position/velocitys and send haptic feedback to the controllers.

## setup

1. Install Quest2ROS on your occulus VR headset [https://quest2ros.github.io/q2r-web/](https://quest2ros.github.io/q2r-web/)

2. Clone [ROS TCP enpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) into your workspace src

`git clone -b ROS2v0.7.0 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git`

3. Clone quest2ros ROS package in your workspace src

`git clone git@github.com:LiUziYu07/quest2ros.git`

4. Build your workspace

`colcon build`

5. Make sure ROS PC and Occulus Headset are on the same (WIFI) network

6. Start ROS TCP endpoint (replace <YOUR_IP>)

`ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=172.168.0.181 -p tcp_port:=10000`

7. Set <YOUR_IP> and the same port in the VR and press **apply**

8. You should see [INFO] in the terminal that the connection is established

9. Run ros2quest demo

`ros2 run quest2ros ros2quest.py`

10. You can now move the dice and Q2R logo in the VR by pressing the lower controller button of left and right controller respectivly

## Cordinate frame allignment

By pressing A + B on the right and X+Y on the left the relative cordinatframe gets alligned to the current controller position.
To use this for robot teleop allign the controller with the base frame of the robot and press the buttons.

## Bibtex

```
@inproceedings{@software{q2r2023},
  title={Quest2ROS: An App to Facilitate Teleoperating Robots},
  author={Welle, Michael C and Ingelhag, Nils and Lippi, Martina and Wozniak, Maciej K. and Gasparri, Andrea and Kragic, Danica},
  url = {https://quest2ros.github.io/q2r-web/},
  version = {1.0},
  date={2023-12-01}
}
```
