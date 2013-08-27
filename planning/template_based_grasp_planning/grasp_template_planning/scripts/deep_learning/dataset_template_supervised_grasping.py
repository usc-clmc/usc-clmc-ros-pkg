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
from scripts.utils import data_types

from scripts.visualization import viewer

from scripts.data import category
from scripts.data import  dataset
from scripts.data import  utils

def Make_dataset(dir_path_source,dir_path_destination,file_name,fast):
    if not os.path.exists(dir_path_source):
        log.Error("source path does not exist",dir_path_source)
    file_path_bag = os.path.join(dir_path_source,file_name+'.bag')
    if not os.path.exists(dir_path_source):
        log.Error("source path does not exist",dir_path_source)
    file_path_yaml = os.path.join(dir_path_source,file_name+'.yaml')
    if not os.path.exists(file_path_yaml):
        log.Error("yaml file does not exist",file_path_yaml)
        
    kutils.Make_dirs(dir_path_destination)
    file_path_pkl = os.path.join(dir_path_destination,file_name+'.pkl')
    if not os.path.exists(file_path_pkl):
        with open(file_path_yaml,'r') as fi:
            data_yaml = yaml.load_all(fi)
            data_meta = dict()
            
            data_yaml = list(data_yaml)
            len_data_yaml = len(data_yaml)
            for pos,data_doc in enumerate(data_yaml):
                log.Info('progress',pos,'vs',len_data_yaml)
                for key,value in data_doc.items():
                    data_meta[key] = value
        
        utils.Pickle_dump(file_path_pkl,data_meta)
    with kutils.Timer('Load data meta'):
        data_meta = utils.Pickle_load(file_path_pkl)
    
    import time
    disp = viewer.Viewer()
    
    
    bag = rosbag.Bag(file_path_bag)
    
    result_label_positions = collections.defaultdict(list)
    result_label_scores = collections.defaultdict(list)
    result_meta = []
    result_images = []
    
    final_channel_size = None
    final_channels = None
    
    counter = 0
    for topic,msg,t in bag.read_messages(topics=['grasp_dataset']):
        uuid_dataset = msg.uuid
        tmp_meta = data_meta[uuid_dataset]
        width =msg.num_tiles_x
        height =msg.num_tiles_y
        dim_channels =msg.dim_channels
        tmp_data = np.array(msg.data)
        assert -tmp_data.min() < 127
        assert tmp_data.max() < 127
        data = np.array(tmp_data,dtype=np.int8)
        
        assert width == height
        if final_channel_size is None:
            final_channel_size = width
        else:
            assert width == final_channel_size
            
        if final_channels is None:
            final_channels = dim_channels
        else:
            assert dim_channels == final_channels
            
        
        #disp.Reset()
        #disp.Show_image_channel(data,width,height,1)
        #time.sleep(1)
        #log.Info('t',t)
        
        result_images.append(data)
        current_position = len(result_images)-1
        result_meta.append((uuid_dataset,(width,height,dim_channels)))
        
        grasp_uuid = tmp_meta['grasp_uuid']
        result_label_scores[grasp_uuid].append(tmp_meta['grasp_success'])
        result_label_positions[grasp_uuid].append(current_position)
        
        
        if fast and counter > 100:
            break
                

    result_images = np.vstack(result_images).T
    
    for key in result_label_scores.keys():
        result_label_scores[key] = np.array(result_label_scores[key],dtype=np.float)
        result_label_positions[key] = np.array(result_label_positions[key],dtype=np.int)
        log.Debug(key,result_label_scores[key].shape)
        log.Debug(key,result_label_positions[key].shape)
        log.Debug(key,result_images.shape)
    
    
    channel_dataset = dataset.Channels()
    channel_dataset_meta = channel_dataset.Get_meta()
    channel_dataset_meta['channel_size'] = final_channel_size
    channel_dataset_meta['channels'] = final_channels
    channel_dataset.Set_labels(result_label_scores,result_label_positions)
    channel_dataset.Set_data(result_images)
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
    
    if fast:
        channel_dataset.Store(os.path.join(dir_path_destination,'dataset_0_fast.pkl'))
    else:
        channel_dataset.Store(os.path.join(dir_path_destination,'dataset_0.pkl'))
    
    
        
if __name__ == "__main__":
    log.Enable_debug_mode()
    parser = argparse.ArgumentParser("make dataset for learning")
    parser.add_argument("--dir-path-source", "--dps", dest="dir_path_source", help="Path to source folder",required=True,type=str)
    parser.add_argument("--dir-path-destination", "--dpd", dest="dir_path_destination", help="Path to destination folder",required=True,type=str)
    parser.add_argument("--file-name", "--fn", dest="file_name", help="file name of data",required=True,type=str)
    parser.add_argument("--fast",  dest="fast", help="file name of data",required=True,type=bool)
    args = parser.parse_args()
    
    try:
        Make_dataset(args.dir_path_source,args.dir_path_destination,args.file_name,args.fast)
    except:
        log.Except_error()
    