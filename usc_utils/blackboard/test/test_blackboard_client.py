#!/usr/bin/env python
import roslib
roslib.load_manifest('blackboard')
import rospy

from blackboard.blackboard_client import BlackBoardClient

rospy.init_node('BlackBoardClientTest', log_level = rospy.DEBUG)
rospy.loginfo('Started BlackBoardClientTest node.')

blackboard_client = BlackBoardClient()
blackboard_client.initialize('left')
rospy.sleep(0.5)
rospy.loginfo("Waiting for publisher to connect.")

while not rospy.is_shutdown():

    blackboard_client.debug('key1', 'value1')
    rospy.sleep(0.5)
    blackboard_client.info('key2', 'value1')
    rospy.sleep(0.5)
    blackboard_client.warn('key3', 'value1')
    rospy.sleep(0.5)
    blackboard_client.error('key4', 'value1')
    rospy.sleep(0.5)
    blackboard_client.fatal('key5', 'value1')
    rospy.sleep(0.5)
    blackboard_client.white('key1')
    rospy.sleep(0.2)
    blackboard_client.yellow('key2')
    rospy.sleep(0.2)
    blackboard_client.red('key3')
    rospy.sleep(0.2)
    blackboard_client.purple('key4')
    rospy.sleep(0.2)
    blackboard_client.green('key5')
    rospy.sleep(0.2)
    blackboard_client.prediction('whatever')
    rospy.sleep(0.2)
