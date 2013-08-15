#!/usr/bin/python

PKG="task_recorder2_file_io"
import roslib; roslib.load_manifest(PKG)
import rospy
import rosbag

# import description_utilities as du
import task_recorder2_msgs.msg

import threading
import csv
import os, time
import re
import tables as tb
import numpy as np

import subprocess, yaml

class DataSampleBagFileSplitter:
    def __init__(self):
        self._bag_lock = threading.RLock()
        self._bags = []
    
    def get_entries_with_bags(self, topic, start_stamp, end_stamp):
        from rosbag import bag   # for _mergesort
        bag_entries = []
        bag_by_iter = {}
        for b in self._bags:
            bag_start_time = bag_helper.get_start_stamp(b)
            if bag_start_time is not None and bag_start_time > end_stamp:
                continue

            bag_end_time = bag_helper.get_end_stamp(b)
            if bag_end_time is not None and bag_end_time < start_stamp:
                continue

            connections = list(b._get_connections(topic))                
            it = iter(b._get_entries(connections, start_stamp, end_stamp))
            bag_by_iter[it] = b
            bag_entries.append(it)

        for entry, it in bag._mergesort(bag_entries, key=lambda entry: entry.time):
            yield bag_by_iter[it], entry

#    def export_region(self, input_filename, output_filename, start_stamp, end_stamp):
        
    def _export_region(self, path, topics, start_stamp, end_stamp):
        bag_entries = list(self.get_entries_with_bags(topics, start_stamp, end_stamp))

        # Get the total number of messages to copy
        total_messages = len(bag_entries)
        
        # If no messages, prompt the user and return
        if total_messages == 0:
            wx.MessageDialog(None, 'No messages found', 'rxbag', wx.OK | wx.ICON_EXCLAMATION).ShowModal()
            wx.CallAfter(self.frame.StatusBar.gauge.Hide)
            return
        
        # Open the path for writing
        try:
            export_bag = rosbag.Bag(path, 'w')
        except Exception, ex:
            wx.MessageDialog(None, 'Error opening bag file [%s] for writing' % path, 'rxbag', wx.OK | wx.ICON_EXCLAMATION).ShowModal()
            wx.CallAfter(self.frame.StatusBar.gauge.Hide)
            return

        # Run copying in a background thread
        self._export_thread = threading.Thread(target=self._run_export_region, args=(export_bag, topics, start_stamp, end_stamp, bag_entries))
        self._export_thread.start()