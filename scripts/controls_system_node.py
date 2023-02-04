#!/usr/bin/env python
"""
Controls System Data Filter
---------------------------
Merges the velocity, orientation, position, and pressure data
from the IMU, DVL, and pressure sensor into a buffer message that is then 
published at a continuous rate.

ROS Parameters
--------------
CS_TOPIC_NAME : str
CS_NODE_NAME : str
CS_BEAM_INDEX : int
CS_PUB_INTERVAL : int
CS_QUEUE_SIZE : int

ROS Subscribed Topics
---------------
/cs/imu : sensor_msgs/Imu
/cs/dvl : uuv_sensor_ros_plugins_msgs/DVL
/cs/pressure : sensor_msgs/FluidPressure

ROS Published Topics
--------------
CS_TOPIC_NAME : controls_system/Buffer
"""
import rospy
from sensor_msgs.msg import Imu, FluidPressure
from uuv_sensor_ros_plugins_msgs.msg import DVL
from controls_system.msg import Buffer

_NODE_NAME = rospy.get_param('CS_NODE_NAME')
_TOPIC_NAME = rospy.get_param('CS_TOPIC_NAME')

class ControlsSystemDataFilter:
    """
    Controls System Data Filter Class
    ---------------------------------
    
    Methods
    -------
    __init__(TOPIC_NAME, NODE_NAME)
        Initializes the node, subscribes to the topics, and publishes the buffer
    add_to_buffer(msg)
        Adds the corresponding data to the buffer per message type
    publish_to_topic(event)
        Publishes the given buffer
    """
    def __init__(self, TOPIC_NAME, NODE_NAME):
        """
        Filter Initialization
        ---------------------
        Initializes the node using rospy, creates the buffer message,
        subscribes to the IMU, DVL, and pressure topics, and publishes the
        buffer at a continuous rate.
        
        Parameters
        ----------
        TOPIC_NAME : str
        NODE_NAME : str 
        """
        rospy.init_node(NODE_NAME, anonymous=True)

        # Create the buffer message and subscribe to the topics
        self.buffer = Buffer()
        self.beamindex = rospy.get_param('CS_BEAM_INDEX')
        rospy.Subscriber('/cs/imu', Imu, self.add_to_buffer)
        rospy.Subscriber('/cs/dvl', DVL, self.add_to_buffer)
        rospy.Subscriber('/cs/pressure', FluidPressure, self.add_to_buffer)

        # Publish the buffer at the specified interval and queue size
        interval = rospy.get_param('CS_PUB_INTERVAL')
        q_size = rospy.get_param('CS_QUEUE_SIZE')
        timer = rospy.Timer(rospy.Duration(interval), self.publish_to_topic)
        self.pub = rospy.Publisher(TOPIC_NAME, Buffer, queue_size=q_size)

        rospy.spin()
        timer.shutdown()

    def add_to_buffer(self, msg):
        """
        Add to Buffer Method
        --------------------
        Filters through the given message types, then sets the data
        that is needed for the buffer message.
        
        Parameters
        ----------
        msg : sensor_msgs/Imu, uuv_sensor_ros_plugins_msgs/DVL, sensor_msgs/FluidPressure
        """
        if msg._type == 'sensor_msgs/Imu':
            self.buffer.angular_velocity = msg.angular_velocity
            self.buffer.orientation = msg.orientation
        elif msg._type == 'uuv_sensor_ros_plugins_msgs/DVL':
            self.buffer.velocity = msg.velocity
            self.buffer.position = msg.beams[self.beamindex].pose.pose.position
        elif msg._type == 'sensor_msgs/FluidPressure':
            self.buffer.fluid_pressure = msg.fluid_pressure

    def publish_to_topic(self, event):
        """
        Publish to Topic Method
        -----------------------
        Publishes the current buffer message to the CS_TOPIC_NAME
        topic and logs the contents of it.
        
        Parameter
        ---------
        event : rospy.TimerEvent
        """
        self.pub.publish(self.buffer)
        rospy.loginfo(self.buffer)

if __name__ == '__main__':
    try:
        cs = ControlsSystemDataFilter(_NODE_NAME, _TOPIC_NAME)
    except rospy.ROSInterruptException:
        pass
