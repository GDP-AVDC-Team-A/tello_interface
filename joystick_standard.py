from pynput.keyboard import Key, Listener
import rospy
from std_msgs.msg import String
from droneMsgsROS.msg import droneCommand
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped


def on_press(key):
    print('{0} pressed'.format(key))
    if str(key) == "'t'":
        msg = droneCommand()
        msg.command = msg.TAKE_OFF
        pub_highLevel.publish(msg)
    if str(key) == "'y'":
        msg = droneCommand()
        msg.command = msg.LAND
        pub_highLevel.publish(msg)
    if (key == Key.up):
        msg1 = PoseStamped()
        msg1.pose.orientation.x = 0
        msg1.pose.orientation.y = 0.3
        msg1.pose.orientation.z = 0
        msg1.pose.orientation.w = 0.997
        msg2 = TwistStamped()
        msg2.twist.angular.z = 0 #YAW
        msg2.twist.linear.z = 0 #ALTITUDE
        pub_pitchRoll.publish(msg1)
        pub_yawAltitude.publish(msg2)
    elif (key == Key.down):
        msg1 = PoseStamped()
        msg1.pose.orientation.x = 0
        msg1.pose.orientation.y = -0.3
        msg1.pose.orientation.z = 0
        msg1.pose.orientation.w = 0.997
        msg2 = TwistStamped()
        msg2.twist.angular.z = 0 #YAW
        msg2.twist.linear.z = 0 #ALTITUDE
        pub_pitchRoll.publish(msg1)
        pub_yawAltitude.publish(msg2)
    elif(key == Key.left):
        msg1 = PoseStamped()
        msg1.pose.orientation.x = -0.3
        msg1.pose.orientation.y = 0
        msg1.pose.orientation.z = 0
        msg1.pose.orientation.w = 0.997
        msg2 = TwistStamped()
        msg2.twist.angular.z = 0 #YAW
        msg2.twist.linear.z = 0 #ALTITUDE
        pub_pitchRoll.publish(msg1)
        pub_yawAltitude.publish(msg2)
    elif(key == Key.right):
        msg1 = PoseStamped()
        msg1.pose.orientation.x = 0.3
        msg1.pose.orientation.y = 0
        msg1.pose.orientation.z = 0
        msg1.pose.orientation.w = 0.997
        msg2 = TwistStamped()
        msg2.twist.angular.z = 0 #YAW
        msg2.twist.linear.z = 0 #ALTITUDE
        pub_pitchRoll.publish(msg1)
        pub_yawAltitude.publish(msg2)
    elif(str(key) == "'w'"):
        msg1 = PoseStamped()
        msg1.pose.orientation.x = 0
        msg1.pose.orientation.y = 0
        msg1.pose.orientation.z = 0
        msg1.pose.orientation.w = 0
        msg2 = TwistStamped()
        msg2.twist.angular.z = 0 #YAW
        msg2.twist.linear.z = 0.5 #ALTITUDE
        pub_pitchRoll.publish(msg1)
        pub_yawAltitude.publish(msg2)
    elif(str(key) == "'s'"):
        msg1 = PoseStamped()
        msg1.pose.orientation.x = 0
        msg1.pose.orientation.y = 0
        msg1.pose.orientation.z = 0
        msg1.pose.orientation.w = 0
        msg2 = TwistStamped()
        msg2.twist.angular.z = 0 #YAW
        msg2.twist.linear.z = -0.5 #ALTITUDE
        pub_pitchRoll.publish(msg1)
        pub_yawAltitude.publish(msg2)
    elif(str(key) == "'d'"):
        msg1 = PoseStamped()
        msg1.pose.orientation.x = 0
        msg1.pose.orientation.y = 0
        msg1.pose.orientation.z = 0
        msg1.pose.orientation.w = 0
        msg2 = TwistStamped()
        msg2.twist.angular.z = 0.5 #YAW
        msg2.twist.linear.z = 0 #ALTITUDE
        pub_pitchRoll.publish(msg1)
        pub_yawAltitude.publish(msg2)
    elif(str(key) == "'a'"):
        msg1 = PoseStamped()
        msg1.pose.orientation.x = 0
        msg1.pose.orientation.y = 0
        msg1.pose.orientation.z = 0
        msg1.pose.orientation.w = 0
        msg2 = TwistStamped()
        msg2.twist.angular.z = -0.5 #YAW
        msg2.twist.linear.z = 0 #ALTITUDE
        pub_pitchRoll.publish(msg1)
        pub_yawAltitude.publish(msg2)
        

def on_release(key):
    print('{0} release'.format(
        key))
    string = "rc 0 0 0 0"
    #pub.publish(string)
    if key == Key.esc:
        # Stop listener
        return False

# Collect events until released
with Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    pub = rospy.Publisher('drone0/command', String, queue_size=10)
    pub_highLevel = rospy.Publisher('drone0/command/high_level', droneCommand, queue_size=10)
    pub_pitchRoll = rospy.Publisher('drone0/actuator_command/roll_pitch', PoseStamped, queue_size=10)
    pub_yawAltitude = rospy.Publisher('drone0/actuator_command/altitude_rate_yaw_rate', TwistStamped, queue_size=10)
    rospy.init_node('joystick', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    listener.join()


