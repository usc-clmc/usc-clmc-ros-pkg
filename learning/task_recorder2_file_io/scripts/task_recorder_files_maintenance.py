#!/usr/bin/python

PKG="task_recorder2_file_io"
import roslib; roslib.load_manifest(PKG)
import rospy

import task_recorder2_msgs.msg

import task_recorder2_file_io.description_utilities as du
from copy import deepcopy

import os
import os.path
import re as re
from optparse import OptionParser

class TaskRecorderFileMaintenance:
    def __init__(self):
        self.data_samples_topic = "data_samples"
        self.data_sample_label_topic = "data_sample_label"
        self.trial_base = "trial"
        self.connector = "_"
        self.trial_counter_extension = "trial_counter.txt"
        self.package_name = 'arm_recorder_data'
        self.data_dir_name = 'recorder_data'
        self.directory = roslib.packages.get_pkg_dir(self.package_name) + '/' + self.data_dir_name + '/'
        self.parameters = dict();

    def getDirectory(self, description, extra_directory = ""):
        dir = self.directory;
        if isinstance(description, str):
            dir = dir + description
        else:
            dir = dir + du.getN(description)
        dir = dir  + extra_directory
        if not os.path.exists(dir):
            rospy.logerr("Directory >%s< does not exist. Cannot read counter. Probably wrong input." % dir )
            return None
        return dir

    def getCounterFileNames(self, description, extra_directory = ""):
        dir = self.getDirectory(description, extra_directory)
        base_fnames = self.getTopicNames(description, extra_directory)
        filenames = list()
        for base_fname in base_fnames:
            filenames.append(dir + '/' + base_fname + self.connector + self.trial_counter_extension)
        return filenames

    def readCounter(self, filename):
        counter_fd = open(filename, 'r')
        counter = int(counter_fd.readline())
        counter_fd.close()
        return counter

    def writeCounter(self, filename, counter):
        counter_fd = open(filename, 'w')
        counter_fd.write(str(counter))
        counter_fd.close()

    def decrementCounters(self, description, extra_directory = ""):
        filenames = self.getCounterFileNames(description, extra_directory)
        for i, filename in enumerate(filenames):
            counter = self.readCounter(filename)
            if counter <= 1:
                rospy.logwarn("Directory >%s< is now empty.", du.getN(description))
            self.writeCounter(filename, counter - 1)
            
    def incrementCounters(self, description, extra_directory = ""):
        filenames = self.getCounterFileNames(description, extra_directory)
        for i, filename in enumerate(filenames):
            counter = self.readCounter(filename)
            self.writeCounter(filename, counter + 1)
    
    def getTopicNames(self, description, extra_directory = ""):
        filenames = self.getAllFiles(description, extra_directory)
        return self.getTrunkFileNames(filenames)
    def getResampledTopicNames(self, description, extra_directory = "/resampled"):
        return self.getTopicNames(description, extra_directory)
    
    def getAllFiles(self, description, extra_directory = ""):
        dir = self.directory;
        if isinstance(description, str):
            dir = dir + description + extra_directory
        else:
            dir = dir + du.getN(description) + extra_directory
        return self.readAllFileNames(directory = dir)        

    def readAllFileNames(self, directory):
        names = []
        rospy.logdebug('Reading >%s<' %directory)
        for name in os.listdir(directory):
            names.append(name)
        return names

    def removeBagFileEnding(self, filename):
      if filename.endswith('.bag'):
          return filename[:-len('.bag')]
      return filename

    def getTrunkFileNames(self, filenames):
        trunk_filenames = list()
        pattern = self.connector + self.trial_counter_extension
        for filename in filenames:
            filename = self.removeBagFileEnding(filename)
            if filename.endswith(pattern):
                trunk_filenames.append(filename[:-len(pattern)])
        return trunk_filenames
  
    def requireDescriptionAndIdAndTrial(self, options):
        if options.description is None or options.description == "":
            rospy.logerr("No description string specified. You need to specify a description using -d.")            
            return False
        if options.id is None or int(options.id) == -1:
            rospy.logerr("No id specified. You need to specify an id using -i.")
            return False
        if options.trial is None or int(options.trial) == -1:
            rospy.logerr("No trial specified. You need to specify a trial using -t.")
            return False
        return True

    def requireDescriptionAndId(self, options):
        if options.description is None or options.description == "":
            rospy.logerr("No description string specified. You need to specify a description using -d.")
            return False
        if options.id is None or int(options.id) == -1:
            rospy.logerr("No id specified. You need to specify an id using -i.")
            return False
        return True

    def requireTrial(self, options):
        if options.trial is None or int(options.trial) == -1:
            rospy.logerr("No trial specified. You need to specify a trial using -t.")
            return False
        return True
    
    def requireTrialIndex(self, options):
        if int(options.trial) < 0:
            rospy.logerr("No trial index specified. You need to specify a trial index.")
            return False
    
    def getCLMCFilename(self, counter):
        clmc_filename = "d"
        counter_string = str(counter)
        for n in range(0, 5 - len(counter_string)):
            clmc_filename = clmc_filename + "0"
        return clmc_filename + counter_string
    
    def remove(self, description):
        if not isinstance(description, task_recorder2_msgs.msg.Description):
            rospy.logerr("Invalid description. Cannot remove it.")
            return False
        
        counter_filenames = self.getCounterFileNames(description)
        if len(counter_filenames) < 1:
            rospy.logerr("Something is fishy. Is there a counter file in >%s<?", du.getN(description))
            return False
        counter = self.readCounter(counter_filenames[0])

        if description.trial >= counter:
            rospy.logerr("Trial >%i< does not exist. There are only >%i<. Cannot remove it.", description.trial, counter)
            return False
        # if trial is -1 then we copy the last one
        if description.trial < 0:
            description.trial = counter - 1;

        rospy.loginfo("Removing trial >%i< in description >%s<.", description.trial, du.getN(description))
        
        # if it is not the last one
        if description.trial != counter - 1:
            source_description = deepcopy(description)
            source_description.trial = counter - 1
            if not self.copy(source_description, description):
                rospy.logerr("Problem while copying... not removing anything.")
                return False
            self.decrementCounters(description)
            self.decrementCounters(description, extra_directory = '/resampled')

        description.trial = counter - 1;

        rm_fname_extension = self.connector + self.trial_base + self.connector + str(description.trial) + '.bag'
        dir = self.directory + du.getN(description) + '/'
        os.system('rm ' + dir + self.getCLMCFilename(description.trial))
        for filename in self.getTopicNames(description):
            # print "Removing " +  dir + filename + rm_fname_extension
            os.system('rm ' + dir + filename + rm_fname_extension)
        for filename in self.getResampledTopicNames(description):
            # print "Removing " +  dir + 'raw/' + filename + rm_fname_extension
            os.system('rm ' + dir + 'raw/' + filename + rm_fname_extension)
            # print "Removing " +  dir + 'resampled/' + filename + rm_fname_extension
            os.system('rm ' + dir + 'resampled/' + filename + rm_fname_extension)

        self.decrementCounters(description)
        self.decrementCounters(description, extra_directory = '/resampled')
        return True
        
    def copy(self, source_description, destination_description):
        if not isinstance(source_description, task_recorder2_msgs.msg.Description):
            rospy.logerr("Invalid source description. Cannot copy it.")
            return False
        if not isinstance(destination_description, task_recorder2_msgs.msg.Description):
            rospy.logerr("Invalid destination description. Cannot copy it.")
            return False

        source_counter_filenames = self.getCounterFileNames(source_description)
        source_counter = self.readCounter(source_counter_filenames[0])
        # if trial is -1 then we copy the last one
        if source_description.trial < 0:
            source_description.trial = source_counter - 1;
        if source_description.trial >= source_counter:
            rospy.logerr("Trial >%i< does not exist. There are only >%i<. Cannot copy (source) it.", source_description.trial, source_counter)
            return False

        destination_counter_filenames = self.getCounterFileNames(destination_description)
        destination_counter = self.readCounter(destination_counter_filenames[0])
        # if trial is -1 then we copy the last one
        if destination_description.trial < 0:
            destination_description.trial = destination_counter;
        if destination_description.trial > destination_counter:
            rospy.logerr("Trial >%i< does not exist. There are only >%i<. Cannot copy (destionation) it.", destination_description.trial, destination_counter)
            return False

        rospy.loginfo("Copying trial >%i< of >%s< into trial >%i< of >%s<", source_description.trial, du.getN(source_description), destination_description.trial, du.getN(destination_description))

        if source_description.description == destination_description.description and source_description.id == destination_description.id and source_description.trial == destination_description.trial:
            rospy.logerr("Source and destionation description are the same. Skipping copying.")
            return False

        source_filenames = self.getTopicNames(source_description)
        source_resampeld_filenames = self.getResampledTopicNames(source_description)
        source_raw_filenames = deepcopy(source_resampeld_filenames)
        
        destination_filenames = self.getTopicNames(destination_description)
        destination_resampeld_filenames = self.getResampledTopicNames(destination_description)
        destination_raw_filenames = deepcopy(destination_resampeld_filenames)

        if len(source_filenames) != len(destination_filenames) or len(source_raw_filenames) != len(destination_raw_filenames) or len(source_resampeld_filenames) != len(destination_resampeld_filenames):
            rospy.logerr("Problems when copying...")
            return False
        
        source_dir = self.directory + du.getN(source_description) + '/'
        dest_dir = self.directory + du.getN(destination_description) + '/'     

        cp_source_fname_extension = self.connector + self.trial_base + self.connector + str(source_description.trial) + '.bag'
        cp_destination_fname_extension = self.connector + self.trial_base + self.connector + str(destination_description.trial) + '.bag'
        
        os.system('cp ' + source_dir + self.getCLMCFilename(source_description.trial) + ' ' + dest_dir + self.getCLMCFilename(destination_description.trial))
        for i, source_filename in enumerate(source_filenames):
            #print 'cp ' + source_dir + source_filename + cp_source_fname_extension + ' ' + dest_dir + destination_filenames[i] + cp_destination_fname_extension
            os.system('cp ' + source_dir + source_filename + cp_source_fname_extension + ' ' + dest_dir + destination_filenames[i] + cp_destination_fname_extension)
        for i, source_filename in enumerate(source_raw_filenames):
            #print 'cp ' + source_dir + 'raw/' + source_filename + cp_source_fname_extension + ' ' + dest_dir + 'raw/' + destination_raw_filenames[i] + cp_destination_fname_extension
            os.system('cp ' + source_dir + 'raw/' + source_filename + cp_source_fname_extension + ' ' + dest_dir + 'raw/' + destination_raw_filenames[i] + cp_destination_fname_extension)
            #print 'cp ' + source_dir + 'resampled/' + source_filename + cp_source_fname_extension + ' ' + dest_dir + 'resampled/' + destination_resampeld_filenames[i] + cp_destination_fname_extension
            os.system('cp ' + source_dir + 'resampled/' + source_filename + cp_source_fname_extension + ' ' + dest_dir + 'resampled/' + destination_resampeld_filenames[i] + cp_destination_fname_extension)

        self.incrementCounters(destination_description)
        self.incrementCounters(destination_description, extra_directory = '/resampled')
        return True

    def parseDestinationDescription(self, parse_string):
        tmp_descr = re.split(string = parse_string, pattern = "#")
        if len(tmp_descr) != 2 or not tmp_descr[1].isdigit():
            rospy.logerr("Provided destination description >%s< is invalid.", parse_string)
            rospy.logerr("It should be 2 strings. A description and an id.")
            return False
        self.parameters['destination_description'] = du.getO(tmp_descr[0], tmp_descr[1])
        return True

    def parse(self):

        parser = OptionParser()
        parser.add_option("-d", "--description", dest="description",
                          help="description string", default = "")
        parser.add_option("-i", "--id", type="int", dest="id",
                          help="description id", default = -1)
        parser.add_option("-t", "--trial", type="int", dest="trial",
                          help="trial index", default = -1)
        parser.add_option("-r", "--remove", action="store_true", dest="remove",
                          help="remove the recording matching the description, id, and trial", default = "")
        parser.add_option("-c", "--copy", dest="copy",
                          help="copy the recording matching the description to <description#id>", default = "")
        parser.add_option("-m", "--move", dest="move",
                          help="move the recording matching the description to <description#id>", default = "")
        
        (options, args) = parser.parse_args()

        if not self.requireDescriptionAndId(options):
            parser.print_help()
            return False
        self.parameters['source_description'] = du.getO(options.description, options.id, options.trial)

        if options.copy != "" and options.remove != "" or options.copy != "" and options.move != "" or options.remove != "" and options.move != "":
            rospy.logerr("Can't copy/remove/move at the same time...")
            return False
        elif options.copy != "":
            if not self.parseDestinationDescription(options.copy):
                return False
            return self.copy(self.parameters['source_description'], self.parameters['destination_description'])
        elif options.remove != "":
            return self.remove(self.parameters['source_description'])
        elif options.move != "":
            if not self.parseDestinationDescription(options.move):
                return False
            if not self.copy(self.parameters['source_description'], self.parameters['destination_description']):
                rospy.logerr("Problems when copying... not moving >%s< to >%s<.", du.getN(self.parameters['source_description']), du.getN(self.parameters['destination_description']))
                return False
            return self.remove(self.parameters['source_description'])
            
        return True 
        
def main():
    
    rospy.init_node('TaskRecorderFileMaintenance')

    tr = TaskRecorderFileMaintenance()
    if not tr.parse():
        rospy.logerr("Problems when parsing the command line.")

    rospy.signal_shutdown('Done')
    
if __name__ == "__main__":
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.signal_shutdown('Done parsing file')
