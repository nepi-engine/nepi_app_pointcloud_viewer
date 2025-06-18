#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#



import os
import time
import sys
import copy
import numpy as np
import threading
import open3d as o3d
import tf2_ros
import time
import yaml
import cv2

from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from std_msgs.msg import UInt8, Empty, String, Bool, Float32, Int32
from geometry_msgs.msg import Vector3, Transform, Quaternion 
from nepi_interfaces.msg import IDXStatus, RangeWindow, ImageSize, \
  Frame3DTransform, Frame3DTransformUpdate, BoundingBox3D
from nepi_app_pointcloud_viewer.msg import PointcloudSelectionStatus,PointcloudProcessStatus,PointcloudRenderStatus

# For Testing 
from std_msgs.msg import Header
from sensor_msgs.msg import PointField

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_pc 
from nepi_sdk import nepi_img 

from nepi_interfaces.msg import SaveDataRate, SaveDataStatus

from nepi_api.messages_if import MsgIF
from nepi_api.connect_system_if import ConnectSaveDataIF
from nepi_api.data_if import ImageIF
from nepi_api.data_if import PointcloudIF
from nepi_api.connect_node_if import ConnectNodeClassIF


APP_NODE_NAME = 'app_pointcloud_viewer'

class ConnectAppPointcloudViewer:
    msg_if = None
    ready = False
    namespace = '~'

    con_node_if = None

    connected = False
    status_msg = None
    status_connected = False

 
    #######################
    ### IF Initialization
    def __init__(self, 
                namespace = None,
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")


        ##############################    
        # Initialize Class Variables

        if namespace is None:
            namespace = nepi_sdk.create_namespace(self.base_namespace,APP_NODE_NAME)
        else:
            namespace = namespace
        self.namespace = nepi_sdk.get_full_namespace(namespace)


        ##############################   
        ## Node Setup

        # Configs Config Dict ####################
        self.CFGS_DICT = {
                'namespace': self.namespace
        }


        # Services Config Dict ####################
        self.SRVS_DICT = None


        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'reset_controls': {
                'namespace': self.node_namespace,
                'topic': 'reset_controls',
                'msg': Empty,
                'qsize': 10,
                'latch': False

            },
            'add_pointcloud': {
                'namespace': self.node_namespace,
                'topic': 'add_pointcloud',
                'msg': String,
                'qsize': 10,
                'latch': False

            },
            'remove_pointcloud': {
                'namespace': self.node_namespace,
                'topic': 'remove_pointcloud',
                'msg': String,
                'qsize': 10,
                'latch': False

            },
            'age_filter': {
                'namespace': self.node_namespace,
                'topic': 'set_age_filter',
                'msg': Float32,
                'qsize': 10,
                'latch': False
            },
            'set_primary_pointcloud': {
                'namespace': self.node_namespace,
                'topic': 'set_primary_pointcloud',
                'msg': String,
                'qsize': 10,
                'latch': False
            },
            'update_transform': {
                'namespace': self.node_namespace,
                'topic': 'update_transform',
                'msg': Frame3DTransformUpdate,
                'qsize': 10,
                'latch': False
            },
            'reset_controls': {
                'namespace': self.node_namespace,
                'topic': 'process/reset_controls',
                'msg': Empty,
                'qsize': 10,
                'latch': False
            },
            'set_clip_enable': {
                'namespace': self.node_namespace,
                'topic': 'process/set_clip_enable',
                'msg': Bool,
                'qsize': 10,
                'latch': False
            },
            'clip_selection': {
                'namespace': self.node_namespace,
                'topic': 'process/set_clip_selection',
                'msg': String,
                'qsize': 10,
                'latch': False
            },
        'range_clip_m': {
                'namespace': self.node_namespace,
                'topic': 'process/set_range_clip_m',
                'msg': RangeWindow,
                'qsize': 10,
                'latch': False
            },
            'clip_bounding_box3d_topic': {
                'namespace': self.node_namespace,
                'topic': 'process/set_clip_bounding_box3d_topic',
                'msg': String,
                'qsize': 10,
                'latch': False
            },
            'voxel_downsample_size': {
                'namespace': self.node_namespace,
                'topic': 'process/set_voxel_downsample_size',
                'msg': Float32,
                'qsize': 10,
                'latch': False
            },
            'downsample_k_points': {
                'namespace': self.node_namespace,
                'topic': 'process/uniform_downsample_k_points',
                'msg': Int32,
                'qsize': 10,
                'latch': False
            },
            'outlier_removal': {
                'namespace': self.node_namespace,
                'topic': 'process/outlier_removal_num_neighbors',
                'msg': String,
                'qsize': 10,
                'latch': False
            },

            'render_reset_controls': {
                'namespace': self.node_namespace,
                'topic': 'render/reset_controls',
                'msg': Empty,
                'qsize': 10,
                'latch': False
            },
            'set_image_size': {
                'namespace': self.node_namespace,
                'topic': 'render/set_image_size',
                'msg': ImageSize,
                'qsize': 10,
                'latch': False
            },
            'set_range_ratios': {
                'namespace': self.node_namespace,
                'topic': 'render/set_range_ratios',
                'msg': RangeWindow,
                'qsize': 10,
                'latch': False
            },
            'set_zoom_ratio': {
                'namespace': self.node_namespace,
                'topic': 'render/set_zoom_ratio',
                'msg': Float32,
                'qsize': 10,
                'latch': False
            },
            'set_rotate_ratio': {
                'namespace': self.node_namespace,
                'topic': 'render/set_rotate_ratio',
                'msg': Float32,
                'qsize': 10,
                'latch': False
            },
            'set_tilt_ratio': {
                'namespace': self.node_namespace,
                'topic': 'render/set_tilt_ratio',
                'msg': Float32,
                'qsize': 10,
                'latch': False
            },
            'set_camera_fov': {
                'namespace': self.node_namespace,
                'topic': 'render/set_camera_fov',
                'msg': Int32,
                'qsize': 10,
                'latch': False
            },
            'set_camera_view': {
                'namespace': self.node_namespace,
                'topic': 'render/set_camera_view',
                'msg': Vector3,
                'qsize': 10,
                'latch': False
            },
            'set_camera_position': {
                'namespace': self.node_namespace,
                'topic': 'render/set_camera_position',
                'msg': Vector3,
                'qsize': 10,
                'latch': False
            },
            'set_camera_rotation': {
                'namespace': self.node_namespace,
                'topic': 'render/set_camera_rotation',
                'msg': Vector3,
                'qsize': 10,
                'latch': False
            },
            'set_white_bg_enable': {
                'namespace': self.node_namespace,
                'topic': 'render/set_white_bg_enable',
                'msg': Bool,
                'qsize': 10,
                'latch': False
            },
            'set_render_enable': {
                'namespace': self.node_namespace,
                'topic': 'render/set_render_enable',
                'msg': Bool,
                'qsize': 10,
                'latch': False
            }
        }


        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'sel_status_pub': {
                'namespace': self.node_namespace,
                'topic': 'status',
                'msg': PointcloudSelectionStatus,
                'qsize': 1,
                'callback': self._statusCb
            },
            'proc_status_pub': {
                'namespace': self.node_namespace,
                'topic': 'process/status',
                'msg': PointcloudProcessStatus,
                'qsize': 1,
                'callback': self._statusCb
            },
            'render_status_pub': {
                'namespace': self.node_namespace,
                'topic': 'render/status',
                'msg': PointcloudRenderStatus,
                'qsize': 1,
                'callback': self._statusCb
            },
            'save_config': {
                'namespace': self.node_namespace,
                'topic': 'save_config',
                'msg': Empty,
                'qsize': None,
                'latch': False
            },
            'reset_config': {
                'namespace': self.node_namespace,
                'topic': 'reset_config',
                'msg': Empty,
                'qsize': None,
                'latch': False
            },
            'factory_reset_config': {
                'namespace': self.node_namespace,
                'topic': 'factory_reset_config',
                'msg': Empty,
                'qsize': None,
                'latch': False
            }                   
        }


        # Create Node Class ####################
        
        self.con_node_if = ConnectNodeClassIF(
                        namespace = self.namespace,
                        configs_dict = self.CFGS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_class_name = True,
                        msg_if = self.msg_if
        )

        

        self.con_node_if.wait_for_ready()

        self.con_save_data_if = ConnectSaveDataIF(namespace = self.namespace)

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete")
        ###############################
    

    #######################
    # Class Public Methods
    #######################


    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.ready  

    def get_namespace(self):
        return self.namespace

    def check_connection(self):
        return self.connected

    def wait_for_connection(self, timout = float('inf') ):
        if self.con_node_if is not None:
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.connected == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.connected == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.connected


    def check_status_connection(self):
        return self.status_connected

    def wait_for_status_connection(self, timout = float('inf') ):
        if self.con_node_if is not None:
            self.msg_if.pub_info("Waiting for status connection")
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.status_connected == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.status_connected == False:
                self.msg_if.pub_info("Failed to connect to status msg")
            else:
                self.msg_if.pub_info("Status Connected")
        return self.status_connected

    def get_status_dict(self):
        img_status_dict = None
        if self.status_msg is not None:
            img_status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return self.img_status_dict

    def unregister(self):
        self._unsubscribeTopic()

    def previous_image(self):
        pub_name = 'reset_controls'
        msg = Empty()
        self.con_node_if.publish_pub(pub_name,msg) 

    def add_pointcloud(self,pointcloud):
        pub_name = 'add_pointcloud'
        msg = pointcloud
        self.con_node_if.publish_pub(pub_name,msg)  

    def remove_pointcloud(self,pointcloud):
        pub_name = 'remove_pointcloud'
        msg = pointcloud
        self.con_node_if.publish_pub(pub_name,msg)  

    def set_age_filter(self,age_filter):
        pub_name = 'set_age_filter'
        msg = age_filter
        self.con_node_if.publish_pub(pub_name,msg)  

    def set_primary_pointcloud(self,pointcloud):
        pub_name = 'set_primary_pointcloud'
        msg = pointcloud
        self.con_node_if.publish_pub(pub_name,msg)  

    def update_transform(self,transform):
        pub_name = 'update_transform'
        msg = transform
        self.con_node_if.publish_pub(pub_name,msg)  

    def reset_controls(self):
        pub_name = 'reset_controls'
        msg = Empty()
        self.con_node_if.publish_pub(pub_name,msg)

    def set_clip_enable(self,clip_enable):
        pub_name = 'set_clip_enable'
        msg = clip_enable
        self.con_node_if.publish_pub(pub_name,msg)  

    def clip_selection(self,clip):
        pub_name = 'clip_selection'
        msg = clip
        self.con_node_if.publish_pub(pub_name,msg)  

    def set_range_clip_m(self,range_clip):
        pub_name = 'range_clip_m'
        msg = range_clip
        self.con_node_if.publish_pub(pub_name,msg)  

    def set_clip_bounding_box3d_topic(self,box3d_topic):
        pub_name = 'clip_bounding_box3d_topic'
        msg = box3d_topic
        self.con_node_if.publish_pub(pub_name,msg)  

    def set_voxel_downsample_size(self,voxel_size):
        pub_name = 'voxel_downsample_size'
        msg = voxel_size
        self.con_node_if.publish_pub(pub_name,msg)  

    def set_downsample_k_points(self,k_points):
        pub_name = 'downsample_k_points'
        msg = k_points
        self.con_node_if.publish_pub(pub_name,msg)  

    def outlier_removal(self,outlier):
        pub_name = 'outlier_removal'
        msg = outlier
        self.con_node_if.publish_pub(pub_name,msg)  

    def reset_controls(self):
        pub_name = 'render_reset_controls'
        msg = Empty()
        self.con_node_if.publish_pub(pub_name,msg)

    def set_image_size(self,image_size):
        pub_name = 'set_image_size'
        msg = image_size
        self.con_node_if.publish_pub(pub_name,msg)  

    def set_range_ratios(self,range_ratios):
        pub_name = 'set_range_ratios'
        msg = range_ratios
        self.con_node_if.publish_pub(pub_name,msg)  

    def set_zoom_ratio(self,zoom_ratio):
        pub_name = 'set_zoom_ratio'
        msg = zoom_ratio
        self.con_node_if.publish_pub(pub_name,msg)  

    def set_rotate_ratio(self,rotate_ratio):
        pub_name = 'set_rotate_ratio'
        msg = rotate_ratio
        self.con_node_if.publish_pub(pub_name,msg)  

    def set_tilt_ratio(self,tilt_ratio):
        pub_name = 'set_tilt_ratio'
        msg = tilt_ratio
        self.con_node_if.publish_pub(pub_name,msg)  

    def set_camera_fov(self,camera_fov):
        pub_name = 'set_camera_fov'
        msg = camera_fov
        self.con_node_if.publish_pub(pub_name,msg)  

    def set_camera_view(self,camera_view):
        pub_name = 'set_camera_view'
        msg = camera_view
        self.con_node_if.publish_pub(pub_name,msg) 

    def set_camera_position(self,camera_position):
        pub_name = 'set_camera_position'
        msg = camera_position
        self.con_node_if.publish_pub(pub_name,msg) 

    def set_camera_rotation(self,camera_rotation):
        pub_name = 'set_camera_rotation'
        msg = camera_rotation
        self.con_node_if.publish_pub(pub_name,msg) 

    def set_white_bg_enable(self,white_bg_enable):
        pub_name = 'set_white_bg_enable'
        msg = white_bg_enable
        self.con_node_if.publish_pub(pub_name,msg) 

    def set_render_enable(self,render_enable):
        pub_name = 'set_render_enable'
        msg = render_enable
        self.con_node_if.publish_pub(pub_name,msg) 

    def save_config(self):
        self.con_node_if.publish_pub('save_config',Empty())

    def reset_config(self):
        self.con_node_if.publish_pub('reset_config',Empty())

    def factory_reset_config(self):
        self.con_node_if.publish_pub('factory_reset_config',Empty())
        
    #################
    ## Save Data Functions

    def get_data_products(self):
        data_products = self.con_save_data_if.get_data_products()
        return data_products

    def get_status_dict(self):
        status_dict = self.con_save_data_if.get_status_dict()
        return status_dict

    def save_data_pub(self,enable):
        self.con_save_data_if.save_data_pub(enable)

    def save_data_prefix_pub(self,prefix):
        self.con_save_data_if.save_data_prefix_pub(prefix)

    def save_data_rate_pub(self,rate_hz, data_product = SaveDataRate.ALL_DATA_PRODUCTS):
        self.con_save_data_if.publish_pub(rate_hz, data_product = SaveDataRate.ALL_DATA_PRODUCTS)

    def snapshot_pub(self):
        self.con_save_data_if.publish_pub()

    def reset_pub(self):
        self.con_save_data_if.publish_pub(pub_name,msg)

    def factory_reset_pub(self):
        pub_name = 'factory_reset'
        msg = Empty()
        self.con_save_data_if.publish_pub(pub_name,msg)

    ###############################
    # Class Private Methods
    ###############################
   

    def _unsubscribeTopic(self):
        success = False
        self.connected = False
        if self.con_node_if is not None:
            self.msg_if.pub_warn("Unregistering topic: " + str(self.namespace))
            try:
                self.con_node_if.unregister_class()
                time.sleep(1)
                self.con_node_if = None
                self.namespace = None
                self.status_connected = False 
                self.data_dict = None
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed to unregister image:  " + str(e))
        return success


    def _statusCb(self,status_msg):      
        self.status_connected = True
        self.status_msg = status_msg
