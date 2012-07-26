#!/usr/bin/python

import re

import roslib;
PKG='task_recorder2_file_io'
roslib.load_manifest(PKG)
import task_recorder2_msgs.msg

import numpy as np

def getT(description):
    """get the trial string (e.g. t5 for trial 5)"""
    if isinstance(description, task_recorder2_msgs.msg.Description):
        return 't' + str(description.trial)
    elif isinstance(description, str):
        string_split = re.split(string = description, pattern = '_')
        if len(string_split) > 1:
            if string_split[-1].isdigit():
                return 't' + string_split[-1]
        raise Exception("ERROR: Invalid description >%s< . This should never happen !" % description)
    elif isinstance(description, int):
        return 't' + str(description)
    raise Exception("ERROR: Invalid description type >%s< . This should never happen !" % str(type(description)))

def getD(description):
    """get the description string"""
    return description.description

def getI(description):
    """get the id as string (e.g. i5 for id 5)"""
    return 'i' + str(description.id)

def getDescr(description, connector):
    """get the description as name string (e.g. flash/i5 for description flash, connecter slash and id 5)"""
    return getD(description) + connector + getI(description)
    
def getG(description):
    """get the description as group string (e.g. flash/i5 for description flash and id 5)"""
    return getDescr(description = description, connector = '/')

def getN(description):
    """get the description (file) name (e.g. flash_5 for description flash and id 5)"""
    return description.description + '_' + str(description.id)

def getDescrNode(description):
    """get the description node name (e.g. flash/i5/t0 for description flash and id 5 and trial 0)"""
    return getG(description = description) + '/' + getT(description)

def getO(description_string, description_id = None, trial = -1):
    """get a description setup with provided description info"""
    if description_id is None:
        d = description_string.rsplit('_', 1)
        if len(d) < 1:
            print "Invalid description_string provided >%s<. Got >%i< splits." % (description_string, len(d))
            return False
        if not d[1].isdigit():
            print "Invalid description_string provided >%s<. Id >%s< is invalid." % (description_string, d[1])
            return False
        description_id = int(d[1])
        description_string = d[0]
    description = task_recorder2_msgs.msg.Description()
    description.description = description_string
    description.id = description_id
    description.trial = trial
    return description

def getDir(description_string, description_id):
    return getN(getO(description_string, description_id))

def remove(string, pattern = '/'):
    """removes the provided pattern from provided string"""
    string_split = re.split(string = string, pattern = '/')
    if len(string_split) > 1:
        print 'WARNING: Removing >%s< from string >%s< to >%s<.' %(pattern, string, ''.join(string_split)) 
    return ''.join(string_split)

def check(string, pattern = '/'):
    """returns true if provided pattern is contained in provided string"""
    return len(re.search(pattern = pattern, string = string)) > 0

def getAbsWhereGroup(description, name):
    """returns the absolute group name.
    Adds a prefixing slash at the beginning if neccessary
    Removes the trailing name if neccessary
    Removes the trailing slash"""
    where = ''
    if isinstance(description, str):
        where = description
        if not where.startswith('/'):
            where = '/' + where
        if where.endswith('/' + name):
            where = where[:-len('/' + name)]
        if where.endswith('/'):
            where = where[:-len('/')]
        if where.endswith('/t-1'):
            where = where[:-len('/t-1')]
        if where.endswith('/i-1'):
            where = where[:-len('/i-1')]
    elif isinstance(description, task_recorder2_msgs.msg.Description):
        if description.id < 0:
            where = '/' + getD(description)            
        else:
            where = '/' + getG(description)            
    else:
        raise Exception("Unknown type for description " + str(type(description)) + ". Cannot return group.")

    return where

def getAbsWhereNode(description, trial, name):
    """returns the absolute node name.
    Adds a prefixing slash at the beginning if neccessary
    Removes the trailing name if neccessary
    Removes the trailing slash""" 
    where = ''
    if isinstance(description, str):
        where = description
        if not where.startswith('/'):
            where = '/' + where
        if where.endswith('/' + name):
            where = where[:-len('/' + name)]
        if where.endswith('/'):
            where = where[:-len('/')]
        if trial >= 0:
            where = where + '/' + getT(trial)
    elif isinstance(description, task_recorder2_msgs.msg.Description):
        if description.id < 0:
            where = '/' + getD(description)   
        else:
            if description.trial < 0:
                where = '/' + getG(description)            
            else:
                where = '/' + getDescrNode(description)
    else:
        raise Exception("Unknown type for description " + str(type(description)) + ". Cannot return node.")
    
    return where

def extractData(data_samples, variable_names):
    """returns the data as a list of numpy arrays in case data_samples is a list
    in case data_samples is a single DataSample than it returns a single numpy array
    """
    if isinstance(data_samples, list):
        indices = getIndices(data_samples[0].names, variable_names)
        data_matrix = np.ndarray([len(data_samples), len(indices)])
        for i,data_sample in enumerate(data_samples):
            data_matrix[i] = getData(data_sample, indices)
        return data_matrix
        
    elif isinstance(data_samples, task_recorder2_msgs.msg.DataSample):
        indices = getIndices(data_samples.names, variable_names)        
        return getData(data_samples, indices)
    raise Exception("ERROR: Invalid input >%s<." % type(data_samples))

def getData(data_samples, indices):
    """returns the data contained in data_samples based on the provided indices"""
    if isinstance(data_samples, list) or isinstance(data_samples, np.ndarray):
        data_matrix = np.ndarray([len(data_samples), len(indices)])
        for i,data in enumerate(data_samples):
            for j,index in enumerate(indices):
                data_matrix[i,j] = data.data[index]
        return data_matrix
    elif isinstance(data_samples, task_recorder2_msgs.msg.DataSample):
        data_vector = np.ndarray( len(indices) )
        for i,index in enumerate(indices):
            data_vector[i] = data_samples.data[index]
        return data_vector
    raise Exception("ERROR: Invalid input >%s<." % type(data_samples))

def getIndices(all_variable_names, querry_variable_names):
    indices = []
    for variable_name in querry_variable_names:
        try:
            indices.append(all_variable_names.index(variable_name))
        except ValueError:
            raise Exception("ERROR: Invalid variable name >%s< . Not contained. This should never happen !" % variable_name)
    return indices

