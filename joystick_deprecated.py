from pynput.keyboard import Key, Listener
import rospy
from std_msgs.msg import String
from droneMsgsROS.msg import droneCommand
from droneMsgsROS.msg import dronePitchRollCmd
from droneMsgsROS.msg import droneDYawCmd
from droneMsgsROS.msg import droneDAltitudeCmd

print("PRESS Q or ESC TO EXIT")

SPEED = 0.5

def sendRc(pitch, roll, yaw, altitude):
    msg1 = dronePitchRollCmd()
    msg1.pitchCmd = pitch
    msg1.rollCmd = roll
    msg2 = droneDYawCmd()
    msg2.dYawCmd = yaw
    msg3 = droneDAltitudeCmd()
    msg3.dAltitudeCmd = altitude
    pub_pitchRoll.publish(msg1)
    pub_yaw.publish(msg2)
    pub_altitude.publish(msg3)

def emergency():
    msg = droneCommand()
    msg.command = msg.EMERGENCY_STOP
    pub_highLevel.publish(msg)

def land():
    msg = droneCommand()
    msg.command = msg.LAND
    pub_highLevel.publish(msg)

def takeoff():
    msg = droneCommand()
    msg.command = msg.TAKE_OFF
    pub_highLevel.publish(msg)

def on_press(key):
    print('{0} pressed'.format(key))

    if str(key) == "'t'":
        takeoff()
    if str(key) == "'y'":
        land()
    if str(key) == "'e'":
        emergency()
    if (key == Key.up):
        sendRc(SPEED,0,0,0)
    elif (key == Key.down):
        sendRc(-SPEED,0,0,0)
    elif(key == Key.left):
        sendRc(0,-SPEED,0,0)
    elif(key == Key.right):
        sendRc(0,SPEED,0,0)
    elif(str(key) == "'w'"):
        sendRc(0,0,0,SPEED)
    elif(str(key) == "'s'"):
        sendRc(0,0,0,-SPEED)
    elif(str(key) == "'d'"):
        sendRc(0,0,SPEED,0)
    elif(str(key) == "'a'"):
        sendRc(0,0,-SPEED,0)

def on_release(key):
    string = "rc 0 0 0 0"
    if key == Key.esc:
        # Stop listener
        return False
    if (str(key) == "'q'"):
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