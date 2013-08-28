#!/usr/bin/env python
####!/Network/Servers/durer/Volumes/durer/kappler/projects/python/env/data/output/env/bin/python
######################################################################
# Copyright (c) 2013, Daniel Kappler
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# # Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
# # Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
# # The names of its contributors are not allowed to be used to endorse 
#     or promote products derived from this software without specific prior 
#     written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# \file dataset_imagenet.py
#
# \author Daniel Kappler
#
#######################################################################
import collections
__author__ = 'Daniel Kappler'
__copyright__ = '2013 Daniel Kappler (daniel.kappler@gmail.com)'

import roslib; roslib.load_manifest('grasp_template_planning')
import rospy
import rosbag

import sys
import os
import argparse
import yaml
import numpy as np


import matplotlib.pyplot as plt
import matplotlib as mpl

from ktoolspy import log
from ktoolspy import utils as kutils

from scripts.visualization import viewer

from scripts.data import category
from scripts.data import dataset
from scripts.data import utils
from scripts.data import dataset

    
def Combine_datasets(dir_path_source,dir_path_destination,file_name_destination):
    if not os.path.exists(dir_path_source):
        log.Error("source path does not exist",dir_path_source)
        
    dataset_data = []
    dataset_meta = []
    dataset_labels_grasp_success = []
    dataset_labels_grasp_uuid = []
    for file_name in os.listdir(dir_path_source):
        if not file_name.endswith('pkl'):
            log.Error('only pickled files are supported',file_name)
        loaded_dataset = dataset.Base.Load(os.path.join(dir_path_source,file_name))
        dataset_data.append(loaded_dataset.Get_data())
        dataset_meta.append(loaded_dataset.Get_meta())
        grasp_success,grasp_uuid=loaded_dataset.Get_labels()
        dataset_labels_grasp_success.append(grasp_success)
        dataset_labels_grasp_uuid.append(grasp_uuid)
        
    result_data = np.hstack(dataset_data)
    result_labels_grasp_success = np.hstack(dataset_labels_grasp_success)
    result_labels_grasp_uuid = np.hstack(dataset_labels_grasp_uuid)
    
    result_meta = dataset_meta[0]
    channel_dataset = dataset.Channels_supervised()
    channel_dataset_meta = channel_dataset.Get_meta()
    channel_dataset_meta['channel_size'] = result_meta['channel_size']
    channel_dataset_meta['channels'] = result_meta['channels']
    channel_dataset.Set_labels(result_labels_grasp_success,result_labels_grasp_uuid)
    channel_dataset.Set_data(result_data)
    channel_dataset.Set_meta(channel_dataset_meta)
    
    grasp_mapping = category.Grasp_mapping()
    grasp_mapping.Load_global_mapping('data/deep_learning/grasp_mapping.yaml','grasp_template')
    
    meta = dataset.Meta()
    meta.Set_batch_size(512)
    meta.Set_dataset_name('grasp_template')
    meta.Set_data_meta(channel_dataset_meta)
    meta.Set_mapping(grasp_mapping)
    meta.Set_mean(channel_dataset.Mean())
    meta.Store(os.path.join(dir_path_destination,'meta.pkl'))
    
    channel_dataset.Store(os.path.join(dir_path_destination,file_name_destination+'.pkl'))
    
        
        
if __name__ == "__main__":
    log.Enable_debug_mode()
    parser = argparse.ArgumentParser("make dataset for learning")
    parser.add_argument("--dir-path-source", "--dps", dest="dir_path_source", help="Path to source folder",required=True,type=str)
    parser.add_argument("--dir-path-destination", "--dpd", dest="dir_path_destination", help="Path to destination folder",required=True,type=str)
    parser.add_argument("--file-name", "--fn", dest="file_name", help="file name of data",required=True,type=str)
    args = parser.parse_args()
    
    try:
        Combine_datasets(args.dir_path_source,args.dir_path_destination,args.file_name)
    except:
        log.Except_error()
    