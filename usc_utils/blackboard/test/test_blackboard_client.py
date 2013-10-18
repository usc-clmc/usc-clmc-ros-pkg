#!/usr/bin/env python
import roslib
roslib.load_manifest('blackboard')
import rospy

from blackboard_client import BlackBoardClient

rospy.init_node('BlackBoardClientTest', log_level = rospy.DEBUG)
rospy.loginfo('Started BlackBoardClientTest node.')

blackboard_client = LeftBlackBoardClient()

while not rospy.is_shutdown():
    blackboard_client.debug('key1', 'value1')
    rospy.sleep(0.2)
    blackboard_client.info('key2', 'value1')
    rospy.sleep(0.2)
    blackboard_client.warn('key3', 'value1')
    rospy.sleep(0.2)
    blackboard_client.error('key4', 'value1')
    rospy.sleep(0.2)
    blackboard_client.fatal('key5', 'value1')
    rospy.sleep(0.2)
    blackboard_client.info('key1')
    rospy.sleep(0.2)
    blackboard_client.warn('key1')
    rospy.sleep(0.2)
    blackboard_client.error('key1')
    rospy.sleep(0.2)
    blackboard_client.fatal('key1')
    rospy.sleep(0.2)
    blackboard_client.debug('key1')
    rospy.sleep(0.2)
    blackboard_client.info('key2')
    rospy.sleep(0.2)
    blackboard_client.warn('key2')
    rospy.sleep(0.2)
    blackboard_client.error('key2')
    rospy.sleep(0.2)
    blackboard_client.fatal('key2')
    rospy.sleep(0.2)
    blackboard_client.debug('key3')

