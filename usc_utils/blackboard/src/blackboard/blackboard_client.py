'''
Client class for displaying text in rviz.

2011.4.13 Peter Pastor
'''
#! /usr/bin/env python

import roslib; roslib.load_manifest('blackboard')
import rospy

from blackboard.msg import BlackBoardEntry

class BlackBoardClient:
    def __init__(self):
        ''' Client class for displaying text in rviz. '''
        print "BlackBoardClient"
        self.publisher = rospy.Publisher('/BlackBoard/entries', BlackBoardEntry)        
        self.board = "left"
        
    def initialize(self, board = "left"):
        self.board = "left"

    def reset(self):
        self.info(key = BlackBoardEntry.SETUP_KEY, value = '')
        self.info(key = BlackBoardEntry.RECORDING_KEY, value = '')
        self.info(key = BlackBoardEntry.STREAMING_KEY, value = '')

    def debug(self, key, value):
        ''' Display >key: value< pair in green '''
        self._updateAndChangeColor(key, value, BlackBoardEntry.GREEN)
    def info(self, key, value):
        ''' Display >key: value< pair in white '''
        self._updateAndChangeColor(key, value, BlackBoardEntry.WHITE)
    def warn(self, key, value):
        ''' Display >key: value< pair in yellow '''
        self._updateAndChangeColor(key, value, BlackBoardEntry.YELLOW)
    def error(self, key, value):
        ''' Display >key: value< pair in red '''
        self._updateAndChangeColor(key, value, BlackBoardEntry.RED)
    def fatal(self, key, value):
        ''' Display >key: value< pair in purple '''
        self._updateAndChangeColor(key, value, BlackBoardEntry.PURPLE)

    def green(self, key):
        ''' Change color of >key: value< pair indexed by key in green '''
        self._changeColor(key, BlackBoardEntry.GREEN)
    def white(self, key):
        ''' Change color of >key: value< pair indexed by key in white '''
        self._changeColor(key, BlackBoardEntry.WHITE)
    def yellow(self, key):
        ''' Change color of >key: value< pair indexed by key in warn '''
        self._changeColor(key, BlackBoardEntry.YELLOW)
    def red(self, key):
        ''' Change color of >key: value< pair indexed by key in red '''
        self._changeColor(key, BlackBoardEntry.RED)
    def purple(self, key):
        ''' Change color of >key: value< pair indexed by key in purple '''
        self._changeColor(key, BlackBoardEntry.PURPLE)

    def _updateAndChangeColor(self, key, value, color):
        if color < 0 or color >= BlackBoardEntry.NUM_COLORS:
            rospy.logerr("Invalid color >%i< specified. Not publishing visualization marker." % color)
            return False
        entry = BlackBoardEntry()
        entry.action = BlackBoardEntry.UPDATE_AND_CHANGE_COLOR
        entry.board = self.board
        entry.key = key
        entry.value = value
        entry.color = color
        self.publisher.publish(entry)

    def _changeColor(self, key, color):
        if color < 0 or color >= BlackBoardEntry.NUM_COLORS:
            rospy.logerr("Invalid color >%i< specified. Not publishing visualization marker." % color)
            return False
        entry = BlackBoardEntry()
        entry.action = BlackBoardEntry.CHANGE_COLOR
        entry.board = self.board
        entry.key = key
        entry.color = color
        self.publisher.publish(entry)

# THIS DOESN"T WORK :( why ? tell me why ??
# class RightBlackBoardClient(BlackBoardClient):
#     def __init__(self):
#         super(RightBlackBoardClient, self).__init__()
#         #self.initialize("right")
#         
# class MiddleBlackBoardClient(BlackBoardClient):
#     def __init__(self):
#         super(MiddleBlackBoardClient, self).__init__()
#         # self.initialize("middle")
# 
# class LeftBlackBoardClient(BlackBoardClient):
#     def __init__(self):
#         super(LeftBlackBoardClient, self).__init__()
#         # self.initialize("left")
        
        