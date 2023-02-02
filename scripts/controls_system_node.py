#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu, FluidPressure
from uuv_sensor_ros_plugins_msgs.msg import DVL
from controls_system.msg import Buffer

class ControlsSystemDataFilter:
    '''
    Controls System Data Filter
    ----------------------
    Merges the velocity, orientation, position, and pressure data
    from the IMU, DVL, and pressure sensor into a buffer that is published at
    a continuous rate.
    '''
    def __init__(self):
        '''
        Filter Initialization
        ---------------------
        Initializes the node using rospy, creates the buffer message,
        subscribes to the IMU, DVL, and pressure topics, and publishes the buffer
        at a continuous rate.
        '''
        rospy.init_node("controls_system_node", anonymous=True)

        # Subscribe to each of the topics to filter through
        self.buffer = Buffer()
        rospy.Subscriber("rexrov2/imu", Imu, self.add_to_buffer)
        rospy.Subscriber("rexrov2/dvl", DVL, self.add_to_buffer)
        rospy.Subscriber("rexrov2/pressure", FluidPressure, self.add_to_buffer)

        # Publish the buffer every 0.5 seconds
        timer = rospy.Timer(rospy.Duration(0.5), self.publish_to_topic)
        self.pub = rospy.Publisher('controls_system_topic', Buffer, queue_size=1)

        rospy.spin()
        timer.shutdown()

    # Adds the corresponding data to the buffer per message type
    def add_to_buffer(self, msg):
        '''
        Add to Buffer Method
        --------------------
        Filters through the given message types, then sets the data
        that is needed for the buffer message.
        '''
        if msg._type == 'sensor_msgs/Imu':
            self.buffer.angular_velocity = msg.angular_velocity
            self.buffer.orientation = msg.orientation
        elif msg._type == 'uuv_sensor_ros_plugins_msgs/DVL':
            self.buffer.velocity = msg.velocity
            self.buffer.position = msg.beams[0].pose.pose.position
        elif msg._type == 'sensor_msgs/FluidPressure':
            self.buffer.fluid_pressure = msg.fluid_pressure

    # Publishes the given buffer
    def publish_to_topic(self, event):
        '''
        Publish to Topic Method
        -----------------------
        Publishes the current buffer to the controls_system_topic
        topic and logs the contents of it.'''
        self.pub.publish(self.buffer)
        rospy.loginfo(self.buffer)

if __name__ == '__main__':
    try:
        cs = ControlsSystemDataFilter()
    except rospy.ROSInterruptException:
        pass
