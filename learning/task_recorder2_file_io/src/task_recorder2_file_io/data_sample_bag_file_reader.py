#!/usr/bin/python

PKG="task_recorder2_file_io"
import roslib; roslib.load_manifest(PKG)
import rospy
import rosbag

# import description_utilities as du

import task_recorder2_msgs.msg
import csv
import os, time
import re
import tables as tb
import numpy as np

import subprocess, yaml

class DataSampleBagFileReader:
    def __init__(self):
        self.data_samples_topic = "data_samples"
        self.data_sample_label_topic = "data_sample_label"
        self.base_fname = "TaskRecorderManager_data_samples"
        self.trial_base = "trial"
        self.variable_name = "variable_names"
        self.label = "label"
        self.bag_extension = ".bag"
        self.txt_extension = ".dat"
        self.connector = "_"
        self.trial_counter_extension = "trial_counter.txt"
        self.package_name = 'arm_recorder_data'
        self.data_dir_name = 'recorder_data'
        self.directory = roslib.packages.get_pkg_dir(self.package_name) + '/' + self.data_dir_name + '/'
        rospy.loginfo("Reading directory >%s<." % self.directory)
        
    def readDataSamples(self, description):

        if not isinstance(description, task_recorder2_msgs.msg.Description):
            raise Exception('ERROR: Provided description must be of type task_recorder2_msgs.msg.Description. Cannot read data sample from bag file.')
        if description.trial < 0:
            raise Exception('ERROR: Provided trial >%i< is invalid.' % description.trial)

        start_time = time.time()

        # generate bag file name
        # bag_filename = self.directory + du.getN(description) + '/' + self.base_fname + self.connector + self.trial_base + self.connector + str(description.trial) + self.bag_extension;
        bag_filename = self.directory + description.description + '_' + str(description.id) + '/' + self.base_fname + self.connector + self.trial_base + self.connector + str(description.trial) + self.bag_extension;

        info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', bag_filename], stdout=subprocess.PIPE).communicate()[0])
        num_data_samples = 0
        for topic in info_dict['topics']:
            if topic['topic'][topic['topic'].rfind("/")+1:] == self.data_samples_topic:
                num_data_samples = int(topic['messages'])
                # rospy.loginfo('Found >%i< data sample messages. ' % num_data_samples)
        
        # when was the bag file created
        creation_time = time.ctime(os.path.getmtime(bag_filename))

        # open bag file
        bag = rosbag.Bag(bag_filename);

        # TODO: make this faster
        data = np.array([])
        # names = np.array([])
        names = list()
        names_written = False
        data_written = False

        # read bag file
        index = 0
        for (topic, msg, t) in bag.read_messages():

            # read data sample topic
            if topic[topic.rfind("/")+1:] == self.data_samples_topic:

                # write variable_names
                if not names_written:
                    names_written = True
                    names = list(msg.names)
                    names.insert(0, "time")

                # write data
                data_list = list(msg.data)
                data_list.insert(0, msg.header.stamp.to_sec())
                if not data_written:
                    rospy.logdebug('Allocating memory for >%i< x >%i< array.' %(num_data_samples, len(names)))
                    data = np.zeros([num_data_samples, len(names)])
                    data_written = True
                    #data = np.array(data_list)

                data[index] = np.array(data_list).transpose()
                index += 1
                #data = np.vstack((data, np.array(data_list)))

        bag.close()
        rospy.logdebug("Reading bag file took >%.2f< seconds" % (time.time()-start_time))
        rospy.logdebug("Read >%i< variables." % len(names))
        for i,name in enumerate(names):
            rospy.logdebug('Variable >%i< is >%s<.' % (i,name) )
        return (names, data)

    def getAllBagFileDescriptions(self):
        names = []
        for name in os.listdir(self.directory):
            if os.path.isdir(os.path.join(self.directory, name)):
                names.append(name)
        descriptions = []
        for name in names:
            iters = re.finditer(pattern = '_', string = name)
            list = re.split(pattern = '_', string = name)
            if len(list) >= 2:
                description = ''
                for (i, s) in enumerate(list[:-1]):
                    description = description + s                   
                    if i != len(list)-2:
                        description = description + '_'
                        
                d = task_recorder2_msgs.msg.Description()
                d.description = description
                d.id = list[-1]
                d.trial = -1
                # descriptions.append(du.getO(description, list[-1]))
                descriptions.append(d)
        return descriptions 
            
    def getNumberOfTrials(self, description):
        description_directory = description.description + self.connector + str(description.id)
        directory = self.directory + description_directory
        directory = self.addTrailingSlash(directory)
        rospy.logdebug('Reading >%s<' % directory)

        if not os.path.exists(self.directory):
            raise Exception('Directory >%s< does not exist!' % directory)

        if not os.path.exists(directory):
            all_available_descriptions = self.getAllBagFileDescriptions()
            for available_description in all_available_descriptions:
                rospy.logerr('Available descriptions are >%s<.' % available_description)
            raise Exception('Requested Description directory >%s< does not exist!' % directory)

        counter_fname = directory + self.base_fname + self.connector + self.trial_counter_extension
        try:
            counter_fd = open(counter_fname, 'r')
        except IOError as e:
            raise('Cannot open counter file')
        num_files = int(counter_fd.readline())
        rospy.logdebug('Reading >%i< files in >%s<.' % (num_files, description_directory))

        return num_files

    def getIds(self, description):
        descriptions = self.getAllBagFileDescriptions()
        ids = []
        querry_desc_string = ""
        if isinstance(description, str):
            querry_desc_string = description
        else:
            querry_desc_string = description.description
        for d in descriptions:
            if d.description == querry_desc_string:
                ids.append(d.id)
        return ids

    def getDescriptionStrings(self):
        descriptions = self.getAllBagFileDescriptions()
        description_strings = []
        for d in descriptions:
            description_strings.append(d.description)
        description_strings = list(set(description_strings))
        return description_strings

    def getTopic(self, topic):
        return topic[topic.rfind("/")+1:]
    
    def addTrailingSlash(self, directory_name):
        if directory_name[-1:] != '/':
            directory_name = directory_name + '/'
        return directory_name
        
