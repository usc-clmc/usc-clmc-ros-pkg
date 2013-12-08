'''
Client class for displaying text in rviz.

2011.4.13 Peter Pastor
'''
#! /usr/bin/env python

import roslib; roslib.load_manifest('blackboard')
import rospy

from ktoolspy import log
from blackboard.msg import BlackBoardEntry

class BlackBoardClient(object):
    def __init__(self):
        super(BlackBoardClient,self).__init__()
        ''' Client class for displaying text in rviz. '''
        print "BlackBoardClient"
        self.publisher = rospy.Publisher('/BlackBoard/entries', BlackBoardEntry)        
        self.board = None
        
    def initialize(self, board):
        self.board = board

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
        
    def startRecording(self):
        self._updateAndChangeColor(BlackBoardEntry.RECORDING_KEY, BlackBoardEntry.STARTED_VALUE, BlackBoardEntry.YELLOW)
    def startStreaming(self):
        self._updateAndChangeColor(BlackBoardEntry.STREAMING_KEY, BlackBoardEntry.STARTED_VALUE, BlackBoardEntry.YELLOW)
    def recording(self):
        self._updateAndChangeColor(BlackBoardEntry.RECORDING_KEY, BlackBoardEntry.STARTED_VALUE, BlackBoardEntry.RED)
    def streaming(self):
        self._updateAndChangeColor(BlackBoardEntry.RECORDING_KEY, BlackBoardEntry.STARTED_VALUE, BlackBoardEntry.RED)
    def logging(self):
        self._updateAndChangeColor(BlackBoardEntry.LOGGING_KEY, BlackBoardEntry.ON_VALUE, BlackBoardEntry.GREEN)
    def stopRecording(self):
        self._updateAndChangeColor(BlackBoardEntry.RECORDING_KEY, BlackBoardEntry.STOPPING_VALUE, BlackBoardEntry.YELLOW)
    def interruptRecording(self):
        self._updateAndChangeColor(BlackBoardEntry.RECORDING_KEY, BlackBoardEntry.INTERRUPTING_VALUE, BlackBoardEntry.YELLOW)
    def continueRecording(self):
        self._updateAndChangeColor(BlackBoardEntry.RECORDING_KEY, BlackBoardEntry.CONTINUING_VALUE, BlackBoardEntry.YELLOW)
    def stopStreaming(self):
        self._updateAndChangeColor(BlackBoardEntry.STREAMING_KEY, BlackBoardEntry.STOPPING_VALUE, BlackBoardEntry.YELLOW)
        
    def recordigStopped(self):
        self._updateAndChangeColor(BlackBoardEntry.RECORDING_KEY, BlackBoardEntry.STOPPED_VALUE, BlackBoardEntry.WHITE)
    def recordigInterrupted(self):
        self._updateAndChangeColor(BlackBoardEntry.RECORDING_KEY, BlackBoardEntry.INTERRUPTED_VALUE, BlackBoardEntry.YELLOW)
    def recordigContinued(self):
        self._updateAndChangeColor(BlackBoardEntry.RECORDING_KEY, BlackBoardEntry.CONTINUED_VALUE, BlackBoardEntry.BLUE)
    def streamingStopped(self):
        self._updateAndChangeColor(BlackBoardEntry.STREAMING_KEY, BlackBoardEntry.STOPPED_VALUE, BlackBoardEntry.WHITE)
    def loggingStopped(self):
        self._updateAndChangeColor(BlackBoardEntry.LOGGING_KEY, BlackBoardEntry.OFF_VALUE, BlackBoardEntry.WHITE)
        
    def recordingFailed(self):
        self._updateAndChangeColor(BlackBoardEntry.RECORDING_KEY, BlackBoardEntry.FAILED_VALUE, BlackBoardEntry.PURPLE)
    def streamingFailed(self):
        self._updateAndChangeColor(BlackBoardEntry.STREAMING_KEY, BlackBoardEntry.FAILED_VALUE, BlackBoardEntry.PURPLE)
        
    def activatingControl(self):
        self._updateAndChangeColor(BlackBoardEntry.CONTROL_KEY, 0, BlackBoardEntry.YELLOW) # not set
    def freezingControl(self):
        self._updateAndChangeColor(BlackBoardEntry.CONTROL_KEY, 0, BlackBoardEntry.YELLOW) # not set
        
    def controlActive(self):
        self._updateAndChangeColor(BlackBoardEntry.CONTROL_KEY, BlackBoardEntry.ACTIVE_VALUE, BlackBoardEntry.GREEN)
    def controlFrozen(self):
        self._updateAndChangeColor(BlackBoardEntry.CONTROL_KEY, BlackBoardEntry.FROZEN_VALUE, BlackBoardEntry.RED)
    
    def setRecording(self,is_recording):
        if is_recording:
            self.recording()
        else:
            self.recordingStopped()
            
    def setStreaming(self,is_streaming):
        if is_streamning:
            self.streaming()
        else:
            self.streamingStopped()
            
    def setLogging(self,is_logging):
        if is_loggging:
            self.logging()
        else:
            self.loggingStopped()
            
    def setLoggingUnkown(self):
        self._updateAndChangeColor(BlackBoardEntry.LOGGING_KEY, BlackBoardEntry.UNKNOWN_VALUE, BlackBoardEntry.PURPLE)
    def description(self,value):
        self._updateAndChangeColor(BlackBoardEntry.DESCRIPTION_KEY, value, 0) # 0 means color not used
    def descriptionRunning(self,value):
        self._updateAndChangeColor(BlackBoardEntry.DESCRIPTION_KEY, value, BlackBoardEntry.GREEN) 
    def descriptionFinished(self,value):
        self._updateAndChangeColor(BlackBoardEntry.DESCRIPTION_KEY, value, BlackBoardEntry.YELLOW) 
    def descriptionSwapped(self,value):
        self._updateAndChangeColor(BlackBoardEntry.DESCRIPTION_KEY, value, BlackBoardEntry.RED) 
    def descriptionFailed(self,value):
        self._updateAndChangeColor(BlackBoardEntry.DESCRIPTION_KEY, value, BlackBoardEntry.PURPLE) 
    def descriptionStopped(self,value):
        self._updateAndChangeColor(BlackBoardEntry.DESCRIPTION_KEY, value, BlackBoardEntry.RED) 
    def descriptionContinued(self,value):
        self._updateAndChangeColor(BlackBoardEntry.DESCRIPTION_KEY, value, BlackBoardEntry.GREEN) 
        
    def prediction(self,value):
        log.Error('prediction',value)
        self._updateAndChangeColor(BlackBoardEntry.PREDICTION_KEY, value, BlackBoardEntry.GREEN) # 0 means color not used
        
    def predictionFiltered(self,value):
        log.Error('prediction',value)
        self._updateAndChangeColor(BlackBoardEntry.PREDICTION_FILTERED_KEY, value, BlackBoardEntry.GREEN) # 0 means color not used
        
    def predictionProgress(self,value):
        self._updateAndChangeColor(BlackBoardEntry.PREDICTION_PROGRESS_KEY, "%.2f"%value, BlackBoardEntry.GREEN) # 0 means color not used
        
    def predictionProgressFiltered(self,value):
        self._updateAndChangeColor(BlackBoardEntry.PREDICTION_PROGRESS_FILTERED_KEY, "%.2f"%value, BlackBoardEntry.GREEN) # 0 means color not used
    
    def predictionLeft(self,value):
        self._updateAndChangeColor(BlackBoardEntry.LEFT_PROGRESS_KEY, "%.2f"%value, BlackBoardEntry.GREEN) # 0 means color not used
    def predictionRight(self,value):
        self._updateAndChangeColor(BlackBoardEntry.RIGHT_PROGRESS_KEY, "%.2f"%value, BlackBoardEntry.GREEN) # 0 means color not used

    def _updateAndChangeColor(self, key, value, color):
        assert(self.board is not None)
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
        assert(self.board is not None)
        if color < 0 or color >= BlackBoardEntry.NUM_COLORS:
            rospy.logerr("Invalid color >%i< specified. Not publishing visualization marker." % color)
            return False
        entry = BlackBoardEntry()
        entry.action = BlackBoardEntry.CHANGE_COLOR
        entry.board = self.board
        entry.key = key
        entry.color = color
        self.publisher.publish(entry)

class RightBlackBoardClient(BlackBoardClient):
    
     def __init__(self):
         super(RightBlackBoardClient, self).__init__()
         self.initialize("right")
         
class MiddleBlackBoardClient(BlackBoardClient):
    def __init__(self):
        super(MiddleBlackBoardClient, self).__init__()
        self.initialize("middle")
 
class LeftBlackBoardClient(BlackBoardClient):
    
    def __init__(self):
        super(LeftBlackBoardClient, self).__init__()
        self.initialize("left")
       
        