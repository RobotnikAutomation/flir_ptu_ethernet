# flir_ptu_ethernet

The flir_ptu_ethernet package, based on RComponent structure. Package for controlling the FLIR PTU using ROS.

---

## Dependencies

- robotnik_msgs [🔗](https://github.com/RobotnikAutomation/robotnik_msgs/)

```bash
git clone https://github.com/RobotnikAutomation/robotnik_msgs/
```

- rcomponent [🔗](https://github.com/RobotnikAutomation/rcomponent/)

```bash
git clone https://github.com/RobotnikAutomation/rcomponent/
```

- robotnik_sensors [🔗](https://github.com/RobotnikAutomation/robotnik_sensors/)

```bash
git clone https://github.com/RobotnikAutomation/robotnik_sensors/
```

---

## ROS

### Parameters

- **desired_freq (double, default: 10.0)**: The frequency of the node.

- **ip_address (string, default: 192.168.0.180)**: The IP address of the device.

- **max_pan_speed (double, default: 120.0)**: The pan speed limit.

- **max_tilt_speed (double, default: 120.0)**: The tilt speed limit.

   
### Subscribed Topics

* **joint_pan_position_controller/command (std_msgs/Float64)**:
  Topic for send a new pan position goal to the device.

* **joint_tilt_position_controller/command (std_msgs/Float64)**:
  Topic for send a new tilt position goal to the device.

* **joint_pan_speed_controller/command (std_msgs/Float64)**:
  Topic for send a new pan velocity to the device.

* **joint_tilt_speed_controller/command (std_msgs/Float64)**:
  Topic for send a new tilt velocity to the device.

### Published Topics

* **~state (robotnik_msgs/State)**:
  Information of the current state of the node.

* **~data (std_msgs/String)**:
  Information of the current position of the device, in degrees.

* **~data_stamped (robotnik_msgs/StringStamped)**:
  Same as ~data topic, with a timestamp.

* **joint_states (sensor_msgs/JointState)**:
  This topic publishes the joint states.

### Bringup

```
roslaunch flir_ptu_ethernet flir_ptu_ethernet.launch ip_address:=192.168.0.180
```
