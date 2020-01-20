from pynput.keyboard import Key, Listener
import rospy
from std_msgs.msg import String
from droneMsgsROS.msg import droneCommand
from droneMsgsROS.msg import dronePitchRollCmd
from droneMsgsROS.msg import droneDYawCmd
from droneMsgsROS.msg import droneDAltitudeCmd


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
        msg1 = dronePitchRollCmd()
        msg1.pitchCmd = 0.5
        msg1.rollCmd = 0
        msg2 = droneDYawCmd()
        msg2.dYawCmd = 0
        msg3 = droneDAltitudeCmd()
        msg3.dAltitudeCmd = 0
        pub_pitchRoll.publish(msg1)
        pub_yaw.publish(msg2)
        pub_altitude.publish(msg3)
    elif (key == Key.down):
        msg1 = dronePitchRollCmd()
        msg1.pitchCmd = -0.5
        msg1.rollCmd = 0
        msg2 = droneDYawCmd()
        msg2.dYawCmd = 0
        msg3 = droneDAltitudeCmd()
        msg3.dAltitudeCmd = 0
        pub_pitchRoll.publish(msg1)
        pub_yaw.publish(msg2)
        pub_altitude.publish(msg3)
    elif(key == Key.left):
        msg1 = dronePitchRollCmd()
        msg1.pitchCmd = 0
        msg1.rollCmd = -0.5
        msg2 = droneDYawCmd()
        msg2.dYawCmd = 0
        msg3 = droneDAltitudeCmd()
        msg3.dAltitudeCmd = 0
        pub_pitchRoll.publish(msg1)
        pub_yaw.publish(msg2)
        pub_altitude.publish(msg3)
    elif(key == Key.right):
        msg1 = dronePitchRollCmd()
        msg1.pitchCmd = 0
        msg1.rollCmd = 0.5
        msg2 = droneDYawCmd()
        msg2.dYawCmd = 0
        msg3 = droneDAltitudeCmd()
        msg3.dAltitudeCmd = 0
        pub_pitchRoll.publish(msg1)
        pub_yaw.publish(msg2)
        pub_altitude.publish(msg3)
    elif(str(key) == "'w'"):
        msg1 = dronePitchRollCmd()
        msg1.pitchCmd = 0
        msg1.rollCmd = 0
        msg2 = droneDYawCmd()
        msg2.dYawCmd = 0
        msg3 = droneDAltitudeCmd()
        msg3.dAltitudeCmd = 0.5
        pub_pitchRoll.publish(msg1)
        pub_yaw.publish(msg2)
        pub_altitude.publish(msg3)
    elif(str(key) == "'s'"):
        msg1 = dronePitchRollCmd()
        msg1.pitchCmd = 0
        msg1.rollCmd = 0
        msg2 = droneDYawCmd()
        msg2.dYawCmd = 0
        msg3 = droneDAltitudeCmd()
        msg3.dAltitudeCmd = -0.5
        pub_pitchRoll.publish(msg1)
        pub_yaw.publish(msg2)
        pub_altitude.publish(msg3)
    elif(str(key) == "'d'"):
        msg1 = dronePitchRollCmd()
        msg1.pitchCmd = 0
        msg1.rollCmd = 0
        msg2 = droneDYawCmd()
        msg2.dYawCmd = 0.5
        msg3 = droneDAltitudeCmd()
        msg3.dAltitudeCmd = 0
        pub_pitchRoll.publish(msg1)
        pub_yaw.publish(msg2)
        pub_altitude.publish(msg3)
    elif(str(key) == "'a'"):
        msg1 = dronePitchRollCmd()
        msg1.pitchCmd = 0
        msg1.rollCmd = 0
        msg2 = droneDYawCmd()
        msg2.dYawCmd = -0.5
        msg3 = droneDAltitudeCmd()
        msg3.dAltitudeCmd = 0
        pub_pitchRoll.publish(msg1)
        pub_yaw.publish(msg2)
        pub_altitude.publish(msg3)
        

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
    pub_pitchRoll = rospy.Publisher('drone0/command/pitch_roll', dronePitchRollCmd, queue_size=10)
    pub_yaw = rospy.Publisher('drone0/command/dYaw', droneDYawCmd, queue_size=10)
    pub_altitude = rospy.Publisher('drone0/command/dAltitude', droneDAltitudeCmd, queue_size=10)
    rospy.init_node('joystick', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    listener.join()


