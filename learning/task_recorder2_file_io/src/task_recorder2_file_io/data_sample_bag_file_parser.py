#!/usr/bin/python

PKG="task_recorder2_file_io"
import roslib; roslib.load_manifest(PKG)
import rospy
import rosbag

import task_recorder2_msgs.msg
import csv
import os.path, time

class DataSampleBagFileParser:
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
        self.num_labels = 0
        
    def convert(self, i):
        bag_filename = self.recorder_directory + self.base_fname + self.connector + self.trial_base + self.connector + str(i) + self.bag_extension;
        bag = rosbag.Bag(bag_filename);
        txt_filename_base = self.recorder_directory + self.base_fname + self.connector + self.trial_base + self.connector

        data_fd = open(txt_filename_base + str(i) + self.txt_extension, 'wb')
        label_fd = open(txt_filename_base + str(i) + self.connector + self.label + self.txt_extension, 'w')
        variable_name_fd = open(txt_filename_base + str(i) + self.connector + self.variable_name + self.txt_extension, 'w')

        names_writen = False
        data_writer = csv.writer(data_fd, quoting=csv.QUOTE_NONE)
        label_writer = csv.writer(label_fd, quoting=csv.QUOTE_NONE)
        name_writer = csv.writer(variable_name_fd, quoting=csv.QUOTE_NONE)

        creation_time = time.ctime(os.path.getmtime(bag_filename))

        # read bag file
        for (topic, msg, t) in bag.read_messages():

            # read data sample topic
            if self.getTopic(topic) == self.data_samples_topic:
                # write data
                data = list(msg.data)
                data.insert(0, msg.header.stamp.to_sec())
                data_writer.writerow(data)
                # write variable_names
                if not names_writen:
                    names_writen = True
                    names = list(msg.names)
                    names.insert(0, "time")
                    name_writer.writerow(names)

            # read label
            if self.getTopic(topic) == self.data_sample_label_topic:
                self.num_labels = self.num_labels+1
                self.writeLabel(label_writer, msg)

        bag.close()
        data_fd.close()
        label_fd.close()
        variable_name_fd.close()
        
    def writeLabel(self, writer, msg):
        data = [msg.type, msg.binary_label.label, msg.cost_label.cost]
        writer.writerow(data)
        
    def parse(self, description):    
        description_directory = description.description + self.connector + str(description.id)
        self.recorder_directory = self.directory + description_directory 
        self.recorder_directory = self.addTrailingSlash(self.recorder_directory)
        rospy.loginfo("Reading >%s<." % self.recorder_directory);

        counter_fname = self.recorder_directory + self.base_fname + self.connector + self.trial_counter_extension
        counter_fd = open(counter_fname, 'r')

        self.num_files = int(counter_fd.readline())
        rospy.loginfo("Reading >%i< files in >%s<." % (self.num_files, description_directory));

        self.num_labels = 0
        for i in range(self.num_files):
            self.convert(i)
            
        if self.num_labels != self.num_files:
            rospy.logwarn("Number of files >%i< does not correspond to number of labels >%i< read." % (self.num_files, self.num_labels))

    def getTopic(self, topic):
        return topic[topic.rfind("/")+1:]
    
    def addTrailingSlash(self, directory_name):
        if directory_name[-1:] != '/':
            directory_name = directory_name + '/'
        return directory_name
        
def main():
    
    rospy.init_node('DataSampleBagFileParser')

    description = task_recorder2_msgs.msg.Description()
    if rospy.has_param('~description'):
        description.description = rospy.get_param('~description')
    else:
        rospy.logerr('Parameter >description< does not exist.')
        exit(-1)
        
    if rospy.has_param('~id'):
        description.id = rospy.get_param('~id')
    else:
        rospy.logwarn('Parameter >id< does not exist. Using >0< instead.')
        description.id = 0

    data_sample_bag_file_parser = DataSampleBagFileParser()
    data_sample_bag_file_parser.parse(description)
        
if __name__ == "__main__":
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.signal_shutdown('Done parsing file')
