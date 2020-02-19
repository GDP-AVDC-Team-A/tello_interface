# Tello Interface

The ROS node "tello interface" implements a ROS bridge for the DJI Tello drone. 


# Subscribed topics

- **tello_command** ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))  
String representation of a command sent directly to the drone. This command is used constantly by the interface to mantain an alive connection with the drone.

- **actuator_command/roll_pitch** ([geometry_msgs/PoseStamped](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/PoseStamped.html))  
Standard Roll and Pitch command to control the drone. Note: this topic is synchronized with <actuator_command/altitude_rate_yaw_rate>, and in order to perform any move, both commands must be sent.

- **actuator_command/altitude_rate_yaw_rate** ([geometry_msgs/TwistStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html))      
Standard Altitude Rate and Yaw Rate command to control the drone. Note: this topic is synchronized with <actuator_command/roll_pitch>, and in order to perform any move, both commands must be sent.

# Published topics

- **sensor_measurement/rotation_angles** ([geometry_msgs/Vector3Stamped](http://docs.ros.org/api/geometry_msgs/html/msg/Vector3Stamped.html))           
Roll, pitch and yaw representation of the orientation of the drone. Reference axis is defined the first time that the connection is established.

- **sensor_measurement/speed** ([geometry_msgs/TwistStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html))           
Linear and angular speeds along roll, pitch and yaw axes. Units are m/s and rad/s respectively.

- **sensor_measurement/accel** ([geometry_msgs/AccelStamped](http://docs.ros.org/api/geometry_msgs/html/msg/AccelStamped.html))           
Acceleration along roll, pitch and yaw axes. Units are m/s².

- **sensor_measurement/imu** ([sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html))           
IMU measurement.

- **sensor_measurement/imu** ([sensor_msgs/BatteryState](http://docs.ros.org/api/sensor_msgs/html/msg/BatteryState.html))           
Battery measurement.

- **sensor_measurement/temperature** ([sensor_msgs/Temperature](http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html))           
Temperature measurement.

- **sensor_measurement/altitude** ([geometry_msgs/PointStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PointStamped.html))           
Altitude measurement in meters.

- **sensor_measurement/sea_level** ([geometry_msgs/PointStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PointStamped.html))           
Sea level measurement in meters.

- **sensor_measurement/camera** ([sensor_msgs/Camera](http://docs.ros.org/api/sensor_msgs/html/msg/Camera.html))           
Camera video.