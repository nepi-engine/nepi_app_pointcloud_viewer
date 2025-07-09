#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_apps) repo
# (see https://https://github.com/nepi-engine/nepi_apps)
#
# License: nepi applications are licensed under the "Numurus Software License", 
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#
import os
# ROS namespace setup
NEPI_BASE_NAMESPACE = '/nepi/s2x/'
#os.environ["ROS_NAMESPACE"] = NEPI_BASE_NAMESPACE[0:-1]


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


from nepi_api.node_if import NodeClassIF
from nepi_api.messages_if import MsgIF
from nepi_api.system_if import SaveDataIF
from nepi_api.data_if import ImageIF
from nepi_api.data_if import PointcloudIF




#########################################

# Factory Control Values
Factory_Combine_Option = 'Add'
Factory_Frame_3d = 'nepi_frame'
Factory_Age_Filter_S = 10.0

Factory_Clip_Enabled = True
Factory_Clip_Selection = 'Range'
Factory_Clip_Min_Range_M = -20
Factory_Clip_Max_Range_M = 20
Factory_Voxel_DownSample_Size = 0.0 # Zero value skips process
Factory_Uniform_DownSample_K_Points = 0 # Zero value skips process
Factory_Outlier_Removal_Num_Neighbors = 0 # Zero value skips process

Factory_Image_Width = 955
Factory_Image_Height = 600
Factory_Start_Range_Ratio = 0.0
Factory_Stop_Range_Ratio = 1.0
Factory_Zoom_Ratio = .5
Factory_Rotate_Ratio = .5
Factory_Tilt_Ratio = .5
Factory_Cam_FOV = 60
Factory_Cam_View = [3, 0, 0]
Factory_Cam_Pos = [-5, 0, 0]
Factory_Cam_Rot = [0, 0, 1]
Factory_Render_Enable = True

UPDATE_POINTCLOUD_SUBS_RATE_HZ = 1
UPDATE_DATA_PRODUCTS_RATE_HZ = 10

ZERO_TRANSFORM = [0,0,0,0,0,0,0]

STANDARD_IMAGE_SIZES = ['630 x 900','720 x 1080','955 x 600','1080 x 1440','1024 x 768 ','1980 x 2520','2048 x 1536','2580 x 2048','3648 x 2736']

#########################################
# Node Class
#########################################

class NepiPointcloudViewerApp(object):

  node_if = None

  combine_options = ["Add"]
  data_products_list = ["pointcloud","pointcloud_image"]
  frame3d_list = ['nepi_frame','map']
  pc_subs_dict = dict()
  pc_min_range_m = 0.0
  pc_max_range_m = 0.1
  primary_pc_frame = None

  bounding_box3d_topic = "NONE"
  bounding_box3d_msg = BoundingBox3D()
  bounding_box3d_sub = None

  clip_options = ['Range','X','Y','Z']

  last_img_width = 0
  last_img_height = 0
  last_fov = 0
  clip_min_range_m = Factory_Clip_Min_Range_M
  clip_max_range_m = Factory_Clip_Max_Range_M
  img_renderer = None
  img_renderer_mtl = None

  update_pointcloud_subs_interval_sec = float(1)/UPDATE_POINTCLOUD_SUBS_RATE_HZ
  update_data_products_interval_sec = float(1)/UPDATE_DATA_PRODUCTS_RATE_HZ

  pointclouds_should_update = False

  acquiring = False

  
  last_bg_white = False

  pc_if = None
  image_if = None

  selected_pointclouds = []
  primary_pointcloud = "None"
  age_filter_s = Factory_Age_Filter_S
  transforms_dict = dict()
  combine_option = Factory_Combine_Option
  process/clip_enabled = Factory_Clip_Enabled
  process/range_min_m = Factory_Clip_Min_Range_M
  process/range_max_m = Factory_Clip_Max_Range_M
  process/voxel_downsample_size = Factory_Voxel_DownSample_Size
  process/uniform_downsample_k_points = Factory_Uniform_DownSample_K_Points
  process/outlier_removal_num_neighbors = Factory_Outlier_Removal_Num_Neighbors
  frame_3d = Factory_Frame_3d
  render/image_width = Factory_Image_Width
  render/image_height = Factory_Image_Height
  render/start_range_ratio = Factory_Start_Range_Ratio
  render/zoom_ratio = Factory_Zoom_Ratio
  render/rotate_ratio = Factory_Rotate_Ratio
  render/tilt_ratio = Factory_Tilt_Ratio
  render/cam_fov = Factory_Cam_FOV
  render/cam_view = Factory_Cam_View
  render/cam_pos = Factory_Cam_Pos
  render/cam_rot = Factory_Cam_Rot
  render/use_wbg = False
  render/render_enable = Factory_Render_Enable

  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "app_pointcloud_viewer" # Can be overwitten by luanch command
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_sdk.init_node(name= self.DEFAULT_NODE_NAME)
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


    ##############################
    ### Setup Node

    # Configs Config Dict ####################
    self.CFGS_DICT = {
            'init_callback': self.initCb,
            'reset_callback': self.resetCb,
            'factory_reset_callback': self.factoryResetCb,
            'init_configs': True,
            'namespace': self.node_namespace
    }

    # Params Config Dict ####################
    self.PARAMS_DICT = {
        'selected_pointclouds': {
            'namespace': self.node_namespace,
            'factory_val': []
        },
        'primary_pointcloud': {
            'namespace': self.node_namespace,
            'factory_val': "None"
        },
        'age_filter_s': {
            'namespace': self.node_namespace,
            'factory_val': Factory_Age_Filter_S
        },
        'transforms_dict': {
            'namespace': self.node_namespace,
            'factory_val': dict()
        },
        'combine_option': {
            'namespace': self.node_namespace,
            'factory_val': Factory_Combine_Option
        },
        'process/clip_enabled': {
            'namespace': self.node_namespace,
            'factory_val': Factory_Clip_Enabled
        },
        'process/range_min_m': {
            'namespace': self.node_namespace,
            'factory_val': Factory_Clip_Min_Range_M
        },
        'process/range_max_m': {
            'namespace': self.node_namespace,
            'factory_val': Factory_Clip_Max_Range_M
        },
        'process/voxel_downsample_size': {
            'namespace': self.node_namespace,
            'factory_val': Factory_Voxel_DownSample_Size
        },
        'process/uniform_downsample_k_points': {
            'namespace': self.node_namespace,
            'factory_val': Factory_Uniform_DownSample_K_Points
        },
        'process/outlier_removal_num_neighbors': {
            'namespace': self.node_namespace,
            'factory_val': Factory_Outlier_Removal_Num_Neighbors
        },
        'frame_3d': {
            'namespace': self.node_namespace,
            'factory_val': Factory_Frame_3d
        },
        'render/image_width': {
            'namespace': self.node_namespace,
            'factory_val': Factory_Image_Width
        },
        'render/image_height': {
            'namespace': self.node_namespace,
            'factory_val': Factory_Image_Height
        },
        'render/start_range_ratio': {
            'namespace': self.node_namespace,
            'factory_val': Factory_Start_Range_Ratio
        },
        'render/zoom_ratio': {
            'namespace': self.node_namespace,
            'factory_val': Factory_Zoom_Ratio
        },
        'render/rotate_ratio': {
            'namespace': self.node_namespace,
            'factory_val': Factory_Rotate_Ratio
        },
        'render/tilt_ratio': {
            'namespace': self.node_namespace,
            'factory_val': Factory_Tilt_Ratio
        },
        'render/cam_fov': {
            'namespace': self.node_namespace,
            'factory_val': Factory_Cam_FOV
        },
        'render/cam_view': {
            'namespace': self.node_namespace,
            'factory_val':Factory_Cam_View
        },
        'render/cam_pos': {
            'namespace': self.node_namespace,
            'factory_val':Factory_Cam_Pos
        },
        'render/cam_rot': {
            'namespace': self.node_namespace,
            'factory_val':Factory_Cam_Rot
        },
        'render/use_wbg': {
            'namespace': self.node_namespace,
            'factory_val': False
        },
        'render/render_enable': {
            'namespace': self.node_namespace,
            'factory_val':Factory_Render_Enable
        }

    }


    # Publishers Config Dict ####################
    self.PUBS_DICT = {
        'sel_status_pub': {
            'namespace': self.node_namespace,
            'topic': 'status',
            'msg': PointcloudSelectionStatus,
            'qsize': 1,
            'latch': False
        },
        'proc_status_pub': {
            'namespace': self.node_namespace,
            'topic': 'process/status',
            'msg': PointcloudProcessStatus,
            'qsize': 1,
            'latch': False
        },
        'render_status_pub': {
            'namespace': self.node_namespace,
            'topic': 'render/status',
            'msg': PointcloudRenderStatus,
            'qsize': 1,
            'latch': False
        }
    }

    # Subscribers Config Dict ####################
    self.SUBS_DICT = {
       'reset_controls': {
            'namespace': self.node_namespace,
            'topic': 'reset_controls',
            'msg': Empty,
            'qsize': 10,
            'callback': self.resetSelectionControlsCb, 
            'callback_args': ()
        },
        'add_pointcloud': {
            'namespace': self.node_namespace,
            'topic': 'add_pointcloud',
            'msg': String,
            'qsize': 10,
            'callback': self.addPointcloudCb, 
            'callback_args': ()
        },
        'remove_pointcloud': {
            'namespace': self.node_namespace,
            'topic': 'remove_pointcloud',
            'msg': String,
            'qsize': 10,
            'callback': self.removePointcloudCb, 
            'callback_args': ()
        },
        'age_filter': {
            'namespace': self.node_namespace,
            'topic': 'set_age_filter',
            'msg': Float32,
            'qsize': 10,
            'callback': self.setAgeFilterCb, 
            'callback_args': ()
        },
        'set_primary_pointcloud': {
            'namespace': self.node_namespace,
            'topic': 'set_primary_pointcloud',
            'msg': String,
            'qsize': 10,
            'callback': self.setPrimaryPointcloudCb, 
            'callback_args': ()
        },
        'update_transform': {
            'namespace': self.node_namespace,
            'topic': 'update_transform',
            'msg': Frame3DTransformUpdate,
            'qsize': 10,
            'callback': self.updateTransformCb, 
            'callback_args': ()
        },
        'reset_controls': {
            'namespace': self.node_namespace,
            'topic': 'process/reset_controls',
            'msg': Empty,
            'qsize': 10,
            'callback': self.resetProcessControlsCb, 
            'callback_args': ()
        },
        'set_clip_enable': {
            'namespace': self.node_namespace,
            'topic': 'process/set_clip_enable',
            'msg': Bool,
            'qsize': 10,
            'callback': self.clipEnableCb, 
            'callback_args': ()
        },
        'clip_selection': {
            'namespace': self.node_namespace,
            'topic': 'process/set_clip_selection',
            'msg': String,
            'qsize': 10,
            'callback': self.setClipSelectionCb, 
            'callback_args': ()
        },

       'range_clip_m': {
            'namespace': self.node_namespace,
            'topic': 'process/set_range_clip_m',
            'msg': RangeWindow,
            'qsize': 10,
            'callback': self.setRangeMetersCb, 
            'callback_args': ()
        },
        'clip_bounding_box3d_topic': {
            'namespace': self.node_namespace,
            'topic': 'process/set_clip_bounding_box3d_topic',
            'msg': String,
            'qsize': 10,
            'callback': self.setClipBoxTopicCb, 
            'callback_args': ()
        },
        'voxel_downsample_size': {
            'namespace': self.node_namespace,
            'topic': 'process/set_voxel_downsample_size',
            'msg': Float32,
            'qsize': 10,
            'callback': self.setVoxelSizeCb, 
            'callback_args': ()
        },
        'downsample_k_points': {
            'namespace': self.node_namespace,
            'topic': 'process/uniform_downsample_k_points',
            'msg': Int32,
            'qsize': 10,
            'callback': self.setUniformPointsCb, 
            'callback_args': ()
        },
        'outlier_removal': {
            'namespace': self.node_namespace,
            'topic': 'process/outlier_removal_num_neighbors',
            'msg': String,
            'qsize': 10,
            'callback': self.setOutlierNumCb, 
            'callback_args': ()
        },

        'render_reset_controls': {
            'namespace': self.node_namespace,
            'topic': 'render/reset_controls',
            'msg': Empty,
            'qsize': 10,
            'callback': self.resetRenderControlsCb, 
            'callback_args': ()
        },
        'set_image_size': {
            'namespace': self.node_namespace,
            'topic': 'render/set_image_size',
            'msg': ImageSize,
            'qsize': 10,
            'callback': self.setImageSizeCb, 
            'callback_args': ()
        },
        'set_range_ratios': {
            'namespace': self.node_namespace,
            'topic': 'render/set_range_ratios',
            'msg': RangeWindow,
            'qsize': 10,
            'callback': self.setRangeRatiosCb, 
            'callback_args': ()
        },
        'set_zoom_ratio': {
            'namespace': self.node_namespace,
            'topic': 'render/set_zoom_ratio',
            'msg': Float32,
            'qsize': 10,
            'callback': self.setZoomRatioCb, 
            'callback_args': ()
        },
        'set_rotate_ratio': {
            'namespace': self.node_namespace,
            'topic': 'render/set_rotate_ratio',
            'msg': Float32,
            'qsize': 10,
            'callback': self.setRotateRatioCb, 
            'callback_args': ()
        },
        'set_tilt_ratio': {
            'namespace': self.node_namespace,
            'topic': 'render/set_tilt_ratio',
            'msg': Float32,
            'qsize': 10,
            'callback': self.setTiltRatioCb, 
            'callback_args': ()
        },
        'set_camera_fov': {
            'namespace': self.node_namespace,
            'topic': 'render/set_camera_fov',
            'msg': Int32,
            'qsize': 10,
            'callback': self.setCamFovCb, 
            'callback_args': ()
        },
        'set_camera_view': {
            'namespace': self.node_namespace,
            'topic': 'render/set_camera_view',
            'msg': Vector3,
            'qsize': 10,
            'callback': self.setCamViewCb, 
            'callback_args': ()
        },
        'set_camera_position': {
            'namespace': self.node_namespace,
            'topic': 'render/set_camera_position',
            'msg': Vector3,
            'qsize': 10,
            'callback': self.setCamPositionCb, 
            'callback_args': ()
        },
        'set_camera_rotation': {
            'namespace': self.node_namespace,
            'topic': 'render/set_camera_rotation',
            'msg': Vector3,
            'qsize': 10,
            'callback': self.setCamRotationCb, 
            'callback_args': ()
        },
        'set_white_bg_enable': {
            'namespace': self.node_namespace,
            'topic': 'render/set_white_bg_enable',
            'msg': Bool,
            'qsize': 10,
            'callback': self.setWhiteBgCb, 
            'callback_args': ()
        },
        'set_render_enable': {
            'namespace': self.node_namespace,
            'topic': 'render/set_render_enable',
            'msg': Bool,
            'qsize': 10,
            'callback': self.setRenderEnableCb, 
            'callback_args': ()
        }
    }


    # Create Node Class ####################
    self.node_if = NodeClassIF(
                    configs_dict = self.CFGS_DICT,
                    params_dict = self.PARAMS_DICT,
                    pubs_dict = self.PUBS_DICT,
                    subs_dict = self.SUBS_DICT
    )

    ready = self.node_if.wait_for_ready()

    self.pc_if = PointcloudIF(namespace = self.node_namespace, topic = 'pointcloud')



    ##############################
    self.initCb(do_updates = True)
    # Set up save data and save config services ########################################################
    self.save_data_if = SaveDataIF(data_products = self.data_products_list)


    ##############################
    # Publish Status
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    self.publish_selection_status()
    self.publish_process_status()
    self.publish_render_status()

    ##############################
    ## Start Pointcloud Subscriber Update Process
    nepi_sdk.start_timer_process(self.update_pointcloud_subs_interval_sec, self.updatePointcloudSubsThread)
    nepi_sdk.start_timer_process(self.update_data_products_interval_sec, self.updateDataProductsThread)

    ## Initiation Complete
    self.msg_if.pub_info("Initialization Complete")
    nepi_sdk.spin()




  #######################
  ### Config Functions

  def initCb(self,do_updates = False):
    if self.node_if is not None:
      pass

    if do_updates == True:
      pass
    self.publish_status()

  def resetCb(self,do_updates = True):
      self.msg_if.pub_warn("Reseting")
      if self.node_if is not None:
        pass
      if do_updates == True:
        pass
      self.initCb(do_updates = do_updates)


  def factoryResetCb(self,do_updates = True):
      self.msg_if.pub_warn("Factory Reseting")
      if self.node_if is not None:
        pass
      if do_updates == True:
        pass
      self.initCb(do_updates = do_updates)


  ###################
  ## Selection Callbacks

  def resetSelectionControlsCb(self,msg):
    self.resetSelectionControls()
  
  def resetSelectionControls(self,do_updates = True):
    self.node_if.reset_param('selected_pointclouds')
    self.node_if.reset_param('primary_pointcloud')
    self.node_if.reset_param('age_filter_s')
    self.node_if.reset_param('transforms_dict')
    self.node_if.reset_param('combine_option')
    if do_updates:
      self.publish_process_status()

  def addPointcloudCb(self,msg):
    ##self.msg_if.pub_info(str(msg))
    pc_topic = msg.data
    pc_topics = self.node_if.get_param('selected_pointclouds')
    add_topic = False
    if nepi_sdk.check_for_topic(pc_topic):
      if pc_topic not in pc_topics:
        add_topic = True
      if add_topic:
        self.msg_if.pub_info("Adding Pointcloud topic to registered topics: " + pc_topic)
        pc_topics.append(pc_topic)
    self.node_if.set_param('selected_pointclouds',pc_topics)
    self.publish_selection_status()

  def removePointcloudCb(self,msg):
    ##self.msg_if.pub_info(str(msg))
    pc_topic = msg.data
    pc_topics = self.node_if.get_param('selected_pointclouds')
    remove_topic = False
    if pc_topic in pc_topics:
      remove_topic = True
    if remove_topic:
      self.msg_if.pub_info("Removing Pointcloud topic from registered topics: " + pc_topic)
      pc_topics.remove(pc_topic)
    self.node_if.set_param('selected_pointclouds',pc_topics)
    self.publish_selection_status()

  def setPrimaryPointcloudCb(self,msg):
    ##self.msg_if.pub_info(str(msg))
    pc_topic = msg.data
    pc_topics = self.node_if.get_param('selected_pointclouds')
    if pc_topic in pc_topics:
      self.node_if.set_param('primary_pointcloud',pc_topic)
    else:
      self.msg_if.pub_info("Ignoring Set Primary Pointcloud as it is not in selected list")
    self.publish_selection_status()

  def setAgeFilterCb(self,msg):
    #self.msg_if.pub_info(str(msg))
    val = msg.data
    if val >= 0:
      self.node_if.set_param('age_filter_s',val)
    self.publish_selection_status()


  def updateTransformCb(self,msg):
    #self.msg_if.pub_info(str(msg))
    self.addTransformToDict(msg)
    self.publish_selection_status()

  def removeTransformCb(self,msg):
    #self.msg_if.pub_info(str(msg))
    topic_namespace = msg.data
    self.removeTransformFromDict(topic_namespace)
    self.publish_selection_status()

  def removePointcloudCb(self,msg):
    #self.msg_if.pub_info(str(msg))
    pc_topic = msg.data
    pc_topics = self.node_if.get_param('selected_pointclouds')
    if pc_topic in pc_topics:
      pc_topics.remove(pc_topic)
    self.node_if.set_param('selected_pointclouds',pc_topics)
    self.publish_selection_status()   

  def setCombineOptionCb(self, msg):
      #self.msg_if.pub_info(str(msg))
      combine_option = msg.data
      if combine_option in self.combine_options:
        self.node_if.set_param('combine_option', combine_option)
      else:
        self.msg_if.pub_info('Pointcloud combine option: ' + combine_option + ' not valid option')
      self.publish_selection_status()
      

  ###################
  ## Process Callbacks
  def resetProcessControlsCb(self,msg):
    self.resetProcessControls()
  
  def resetProcessControls(self,do_updates = True):
    self.node_if.reset_param('process/clip_enabled')
    self.node_if.reset_param('process/clip_selection')
    self.node_if.reset_param('process/range_min_m')
    self.node_if.reset_param('process/range_max_m')
    self.bounding_box3d_topic = "NONE"
    self.node_if.reset_param('process/voxel_downsample_size')
    self.node_if.reset_param('process/uniform_downsample_k_points')
    self.node_if.reset_param('process/outlier_removal_num_neighbors')   
    if do_updates:
      self.publish_process_status()

  def clipEnableCb(self,msg):
    #self.msg_if.pub_info(str(msg))
    new_enable = msg.data
    self.node_if.set_param('process/clip_enabled', new_enable)
    self.publish_process_status()

  def setClipSelectionCb(self,msg):
    #self.msg_if.pub_info(str(msg))
    sel = msg.data
    if sel in self.clip_options:
      self.node_if.set_param('process/clip_selection', sel )
    self.publish_process_status()

  def setClipBoxTopicCb(self,msg):
    #self.msg_if.pub_info(str(msg))
    self.bounding_box3d_topic = msg.data
    topic = msg.data
    if topic != self.bounding_box3d_topic and self.bounding_box3d_sub is not None:
      self.bounding_box3d_sub.Unregister()
      self.bounding_box3d_sub = None
    if topic != "NONE":
      self.bounding_box3d_sub = nepi_sdk.create_subscriber('~set_clip_target_topic', String, self.setClipTargetTopicCb, queue_size = 10)
    self.bounding_box3d_msg = None
    self.publish_process_status()

  def setRangeMetersCb(self,msg):
    #self.msg_if.pub_info(str(msg))
    range_min_m = msg.start_range
    range_max_m = msg.stop_range
    if range_min_m < range_max_m:
      self.node_if.set_param('process/range_min_m', range_min_m)
      self.node_if.set_param('process/range_max_m', range_max_m)
    self.publish_process_status()

  def setVoxelSizeCb(self,msg):
    #self.msg_if.pub_info(str(msg))
    val = msg.data
    if val >= 0:
      self.node_if.set_param('process/voxel_downsample_size',val)
    self.publish_process_status()

  def setUniformPointsCb(self,msg):
    #self.msg_if.pub_info(str(msg))
    val = msg.data
    if val >= 0:
      self.node_if.set_param('process/uniform_downsample_k_points',val)
    self.publish_process_status()

  def setOutlierNumCb(self,msg):
    #self.msg_if.pub_info(str(msg))
    val = msg.data
    if val >= 0:
      self.node_if.set_param('process/outlier_removal_num_neighbors',val)
    self.publish_process_status()

  def setFrame3dCb(self, msg):
    #self.msg_if.pub_info(str(msg))
    frame_3d = msg.data
    frame3d_list = self.frame3d_list
    if frame_3d in frame3d_list:
      self.node_if.set_param('frame_3d',frame_3d)
    self.publish_process_status()

###################
## Render Callbacks

  def resetRenderControlsCb(self,msg):
    self.resetRenderControls()

  def resetRenderControls(self,do_updates = True):
    self.node_if.reset_param('render/image_width')
    self.node_if.reset_param('render/image_height')
    self.node_if.reset_param('render/start_range_ratio')
    self.node_if.reset_param('render/stop_range_ratio')
    self.node_if.reset_param('render/zoom_ratio')
    self.node_if.reset_param('render/rotate_ratio')
    self.node_if.reset_param('render/tilt_ratio')
    self.node_if.reset_param('render/cam_fov')
    self.node_if.reset_param('render/cam_view')
    self.node_if.reset_param('render/cam_pos')
    self.node_if.reset_param('render/cam_rot')
    self.node_if.reset_param('render/use_wbg')
    self.node_if.reset_param('render/render_enable')
    
    if do_updates:
      self.publish_render_status()


  def setImageSizeCb(self,msg):
    #self.msg_if.pub_info(str(msg))
    width = msg.width
    height = msg.height
    if width > 100 and width < 5000 and height > 100 and height < 5000:
      self.node_if.set_param('render/image_width',  width)
      self.node_if.set_param('render/image_height', height)
    self.publish_render_status()

  def setImageSizeIndCb(self,msg):
    #self.msg_if.pub_info(str(msg))
    index = msg.data
    if index <= len(STANDARD_IMAGE_SIZES):
      size_str = STANDARD_IMAGE_SIZES[index]
      size__split = size_str.split(" ")
      width = float(size__split[0])
      height = float(size_split[2])
      if width > 100 and width < 5000 and height > 100 and height < 5000:
        self.node_if.set_param('render/image_width',  width)
        self.node_if.set_param('render/image_height', height)
    self.publish_render_status()

  def setZoomRatioCb(self,msg):
    #self.msg_if.pub_info(str(msg))
    new_val = msg.data
    if new_val >= 0 and new_val <= 1 :
      self.node_if.set_param('render/zoom_ratio',new_val)
    self.publish_render_status()

  def setRotateRatioCb(self,msg):
    #self.msg_if.pub_info(str(msg))
    new_val = msg.data
    if new_val >= 0 and new_val <= 1 :
      self.node_if.set_param('render/rotate_ratio',new_val)
    self.publish_render_status()

  def setTiltRatioCb(self,msg):
    #self.msg_if.pub_info(str(msg))
    new_val = msg.data
    if new_val >= 0 and new_val <= 1 :
      self.node_if.set_param('render/tilt_ratio',new_val)
    self.publish_render_status()

  def setCamFovCb(self,msg):
    #self.msg_if.pub_info(str(msg))
    new_val = msg.data
    if new_val > 100:
      new_val = 100
    if new_val < 30:
      new_val = 30
    self.node_if.set_param('render/cam_fov',new_val)
    self.publish_render_status()

  def setCamViewCb(self,msg):
    #self.msg_if.pub_info(str(msg))
    new_array = []
    new_array.append(msg.x)
    new_array.append(msg.y)
    new_array.append(msg.z)
    self.node_if.set_param('render/cam_view',new_array)
    self.publish_render_status()

  def setCamPositionCb(self,msg):
    #self.msg_if.pub_info(str(msg))
    new_array = []
    new_array.append(msg.x)
    new_array.append(msg.y)
    new_array.append(msg.z)
    self.node_if.set_param('render/cam_pos',new_array)
    self.publish_render_status()

  def setCamRotationCb(self,msg):
    #self.msg_if.pub_info(str(msg))
    new_array = []
    new_array.append(msg.x)
    new_array.append(msg.y)
    new_array.append(msg.z)
    self.node_if.set_param('render/cam_rot',new_array)
    self.publish_render_status()
  

  def setRangeRatiosCb(self,msg):
    #self.msg_if.pub_info(str(msg))
    min_ratio = msg.start_range
    max_ratio = msg.stop_range
    if min_ratio < max_ratio and min_ratio >= 0 and max_ratio <= 1:
      self.node_if.set_param('render/start_range_ratio', min_ratio)
      self.node_if.set_param('render/stop_range_ratio', max_ratio)
    self.publish_render_status()


  def setWhiteBgCb(self,msg):
    enable = msg.data
    self.node_if.set_param('render/use_wbg', enable)
    self.publish_render_status()

  def setRenderEnableCb(self,msg):
    render_enable = msg.data
    self.node_if.set_param('render/render_enable', render_enable)
    self.publish_render_status()






  ###################
  ## Status Publishers
  def publish_selection_status(self):
    status_msg = PointcloudSelectionStatus()

    pointcloud_topic_list = self.node_if.get_param('selected_pointclouds')
    status_msg.selected_pointcloud_topics = (pointcloud_topic_list)

    primary_pointcloud = self.node_if.get_param('primary_pointcloud')
    if primary_pointcloud == "None" and len(pointcloud_topic_list) > 0:
      primary_pointcloud = pointcloud_topic_list[0]
    elif primary_pointcloud != "None" and len(pointcloud_topic_list) == 0:
      primary_pointcloud = "None"
    self.node_if.set_param('primary_pointcloud', primary_pointcloud)
    status_msg.primary_pointcloud_topic = primary_pointcloud
    status_msg.publishing_pointcloud_img = self.image_if is not None

    age_filter_s = self.node_if.get_param('age_filter_s')
    status_msg.age_filter_s = age_filter_s


    status_msg.available_3d_frames = (self.frame3d_list)
    status_msg.output_3d_frame = self.node_if.get_param('frame_3d') 

    transforms_dict = self.node_if.get_param('transforms_dict')
    for pc_topic in pointcloud_topic_list:
      if pc_topic not in transforms_dict.keys():
        transforms_dict[pc_topic] = ZERO_TRANSFORM
    self.node_if.set_param('transforms_dict',transforms_dict)
    [status_msg.transforms_topic_list,status_msg.transforms_list] = self.getFrame3DTransformsMsg()

    status_msg.combine_options = (self.combine_options)
    status_msg.combine_option = self.node_if.get_param('combine_option')

    range_meters = RangeWindow()
    range_meters.start_range =   self.pc_min_range_m
    range_meters.stop_range =   self.pc_max_range_m
    status_msg.range_min_max_m = range_meters

    if self.node_if is not None:
      self.node_if.publish_pub('sel_status_pub', status_msg)


  def publish_process_status(self):
    status_msg = PointcloudProcessStatus()

    status_msg.clip_enabled = bool(self.node_if.get_param('process/clip_enabled'))
    status_msg.clip_options = self.clip_options
    status_msg.clip_selection = self.node_if.get_param('process/clip_selection')
    range_meters = RangeWindow()
    range_meters.start_range =   self.node_if.get_param('process/range_min_m')
    range_meters.stop_range =   self.node_if.get_param('process/range_max_m')
    status_msg.clip_meters = range_meters

    status_msg.clip_target_topic = self.bounding_box3d_topic
    
    status_msg.voxel_downsample_size_m = self.node_if.get_param('process/voxel_downsample_size')
    status_msg.uniform_downsample_points = self.node_if.get_param('process/uniform_downsample_k_points')
    status_msg.outlier_k_points = self.node_if.get_param('process/outlier_removal_num_neighbors') 

    if self.node_if is not None:
      self.node_if.publish_pub('proc_status_pub', status_msg)

  def publish_render_status(self):
    status_msg = PointcloudRenderStatus()

    status_msg.standard_image_sizes = (STANDARD_IMAGE_SIZES)

    status_msg.image_width = self.node_if.get_param('render/image_width')
    status_msg.image_height = self.node_if.get_param('render/image_height')

    range_meters = RangeWindow()
    range_meters.start_range =  self.node_if.get_param('process/range_min_m')
    range_meters.stop_range =   self.node_if.get_param('process/range_max_m')

    status_msg.range_min_max_m = range_meters

    range_ratios = RangeWindow()
    range_ratios.start_range =   float(self.node_if.get_param('render/start_range_ratio'))
    range_ratios.stop_range =   float(self.node_if.get_param('render/stop_range_ratio'))
    status_msg.range_clip_ratios = range_ratios

    status_msg.zoom_ratio = self.node_if.get_param('render/zoom_ratio')
    status_msg.rotate_ratio = self.node_if.get_param('render/rotate_ratio')
    status_msg.tilt_ratio = self.node_if.get_param('render/tilt_ratio')

    fov = self.node_if.get_param('render/cam_fov')
    status_msg.camera_fov = fov

    view = self.node_if.get_param('render/cam_view')
    cam_view = Vector3()
    cam_view.x = view[0]
    cam_view.y = view[1]
    cam_view.z = view[2]
    status_msg.camera_view = cam_view

    pos = self.node_if.get_param('render/cam_pos')
    cam_pos = Vector3()
    cam_pos.x = pos[0]
    cam_pos.y = pos[1]
    cam_pos.z = pos[2]
    status_msg.camera_position = cam_pos

    rot = self.node_if.get_param('render/cam_rot')
    cam_rot = Vector3()
    cam_rot.x = rot[0]
    cam_rot.y = rot[1]
    cam_rot.z = rot[2]
    status_msg.camera_rotation = cam_rot
    
    use_wbg = self.node_if.get_param('render/use_wbg')
    status_msg.white_background = use_wbg
    render_enable = self.node_if.get_param('render/render_enable')
    status_msg.render_enable = render_enable

    if self.node_if is not None:
      self.node_if.publish_pub("render_status_pub", status_msg)

  #######################
  # Data Product Threads

  def updatePointcloudSubsThread(self,timer):
    # Subscribe to topic pointcloud topics if not subscribed
    sel_topics = self.node_if.get_param('selected_pointclouds')
    for sel_topic in sel_topics:
      if sel_topic != "" and sel_topic not in self.pc_subs_dict.keys():
        if nepi_sdk.check_for_topic(sel_topic):
          topic_uid = sel_topic.replace('/','')
          exec('self.' + topic_uid + '_pc = None')
          exec('self.' + topic_uid + '_timestamp = None')
          exec('self.' + topic_uid + '_frame = None')
          exec('self.' + topic_uid + '_lock = threading.Lock()')
          self.msg_if.pub_info("Subscribing to topic: " + sel_topic)
          #self.msg_if.pub_info("with topic_uid: " + topic_uid)
          pc_sub = nepi_sdk.create_subscriber(sel_topic, PointCloud2, lambda msg: self.pointcloudCb(msg, sel_topic), queue_size = 10)
          self.pc_subs_dict[sel_topic] = pc_sub
          self.msg_if.pub_info("Pointcloud: " + sel_topic + " registered")
    if len(list(self.pc_subs_dict.keys())) > 0 and self.image_if is None:
      self.msg_if.pub_info("Setting up pointcloud_image pub")
      self.image_if = ImageIF(namespace = self.node_namespace, topic = "pointcloud_image")
      time.sleep(1)
    elif len(list(self.pc_subs_dict.keys())) == 0 and self.image_if is not None:
      self.msg_if.pub_info("Taking down pointcloud_image pub")
      self.image_if.unregister()
      self.image_if = None
      time.sleep(1)
    # Unregister pointcloud subscribers if not in selected pointclouds list
    unreg_topic_list = []
    for topic in self.pc_subs_dict.keys():
      if topic not in sel_topics:
          pc_sub = self.pc_subs_dict[topic]
          pc_sub.unregister()
          self.msg_if.pub_info("Pointcloud: " + topic + " unregistered")
          unreg_topic_list.append(topic) # Can't change dictionary while looping through dictionary
    for topic in unreg_topic_list: 
          self.pc_subs_dict.pop(topic)
    # Update primary pointcloud if needed
    primary_pc = self.node_if.get_param('primary_pointcloud')
    #print(self.pc_subs_dict.keys())
    if primary_pc not in self.pc_subs_dict.keys():
      if len(self.pc_subs_dict.keys()) > 0:
        primary_pc = list(self.pc_subs_dict.keys())[0]
      else:
        primary_pc = "None"
      if primary_pc != "None":
        self.msg_if.pub_info("Primary pointcloud set to: " + primary_pc)
    self.node_if.set_param('primary_pointcloud', primary_pc)
    self.publish_selection_status()
    

  def pointcloudCb(self,msg,topic):
      if topic != "":
        topic_uid = topic.replace('/','')
        transforms_dict = self.node_if.get_param('transforms_dict')
        if topic in transforms_dict.keys():
          transform = transforms_dict[topic]
        else:
          transform = ZERO_TRANSFORM
          transforms_dict[topic] = transform  
        if topic in self.pc_subs_dict.keys():
          eval('self.' + topic_uid + '_lock').acquire()
          exec('self.' + topic_uid + '_timestamp = msg.header.stamp')
          exec('self.' + topic_uid + '_frame = msg.header.frame_id')
          o3d_pc = nepi_pc.rospc_to_o3dpc(msg, remove_nans=True)
          # ToDo: Apply Frame Transforms before assigning and releasing
          exec('self.' + topic_uid + '_pc = o3d_pc')
          eval('self.' + topic_uid + '_lock').release()


  def updateDataProductsThread(self,timer):
    # Check if new data is needed

    pc_has_subscribers = self.pc_if.has_subscribers_check()
    pc_saving_is_enabled = self.save_data_if.data_product_saving_enabled('pointcloud')
    pc_should_save = pc_saving_is_enabled and self.save_data_if.data_product_should_save('pointcloud')
    pc_snapshot_enabled = self.save_data_if.data_product_snapshot_enabled('pointcloud')
    pc_save = (pc_saving_is_enabled and pc_should_save) or pc_snapshot_enabled
    need_pc = (pc_has_subscribers is True) or (pc_save is True) 

    if self.image_if is not None:
      img_has_subscribers = self.image_if.has_subscribers_check()
    else:
      img_has_subscribers = False
    img_saving_is_enabled = self.save_data_if.data_product_saving_enabled('pointcloud_image')
    img_should_save = self.save_data_if.data_product_should_save('pointcloud_image')
    img_snapshot_enabled = self.save_data_if.data_product_snapshot_enabled('pointcloud_image')
    img_save = (img_saving_is_enabled and img_should_save) or img_snapshot_enabled
    need_img = (img_has_subscribers is True) or (img_save is True)

    ros_frame_id = self.node_if.get_param('frame_3d')
    topic_primary = self.node_if.get_param('primary_pointcloud')
    
    if (need_pc or need_img and topic_primary != "None"):
      o3d_pc = None
      # Combine selected 
      age_filter_s = self.node_if.get_param('age_filter_s')
      combine_option = self.node_if.get_param('combine_option')
      transforms_dict = self.node_if.get_param('transforms_dict')
      current_time = nepi_sdk.get_msg_time()
      pc_add_count = 0
      # Get priamary pointcloud
      topic_puid = topic_primary.replace('/','')
      if topic_primary in self.pc_subs_dict.keys():
        eval('self.' + topic_puid + '_lock').acquire()
        get_msg_timestamp_pr = eval('self.' + topic_puid + '_timestamp')
        primary_pc_frame = eval('self.' + topic_puid + '_frame')
        if get_msg_timestamp_pr is not None:
          pc_age =(current_time.to_sec() - get_msg_timestamp_pr.to_sec())
          if pc_age <= age_filter_s:
            o3d_ppc = eval('self.' + topic_puid + '_pc')
            if o3d_ppc is not None:
              o3d_pc = copy.deepcopy(o3d_ppc)
              if topic_primary in transforms_dict:
                transform = transforms_dict[topic_primary]
                all_zeros = True
                for item in transform:
                  if item != 0:
                    all_zeros = False
                if all_zeros == False:
                  o3d_pc = self.transformPointcloud(o3d_pc, transform)
              pc_add_count += 1
        eval('self.' + topic_puid + '_lock').release()

      if o3d_pc is not None:
        # Add remaining selected pointclouds
        for topic in self.pc_subs_dict.keys():
          topic_uid = topic.replace('/','')
          if topic_uid != topic_puid:  # Skip the primary pointcloud
            get_msg_timestamp_add = None
            if topic in self.pc_subs_dict.keys():
              eval('self.' + topic_uid + '_lock').acquire()
              get_msg_timestamp_add = eval('self.' + topic_uid + '_timestamp')
              pc_frame_add = eval('self.' + topic_uid + '_frame')
              o3d_pc_add = eval('self.' + topic_uid + '_pc')
              eval('self.' + topic_uid + '_lock').release()
            #else:
              #self.msg_if.pub_info("Combine pointcloud not registered yet: " + topic_puid)
            if get_msg_timestamp_add is not None:
              #pc_age = abs(current_time - get_msg_timestamp)
              #pc_age = pc_age.to_sec()
              pc_age =(current_time.to_sec() - get_msg_timestamp_add.to_sec())
              if o3d_pc_add is not None:
                if pc_age <= age_filter_s:
                  if combine_option == 'Add':
                    if topic in transforms_dict:
                      transform = transforms_dict[topic]
                      all_zeros = True
                      for item in transform:
                        if item != 0:
                          all_zeros = False
                      if all_zeros == False:
                        o3d_pc_add = self.transformPointcloud(o3d_pc_add, transform)
                    o3d_pc += o3d_pc_add
                    pc_add_count += 1

  
      if pc_add_count > 0:
        self.pc_min_range_m = nepi_pc.get_min_range(o3d_pc)
        self.pc_max_range_m = nepi_pc.get_max_range(o3d_pc)

        if self.pc_max_range_m != 0:
          # Process Combined Pointcloud
          clip_enable = self.node_if.get_param('process/clip_enabled')
          if clip_enable:
            min_m = self.node_if.get_param('process/range_min_m')
            max_m = self.node_if.get_param('process/range_max_m')
            clip_process = self.node_if.get_param('process/clip_selection')
            if clip_process == 'Range' and min_m < 0:
              min_m = 0
            clip_function = self.getClipFunction(clip_process)
            o3d_pc = clip_function(o3d_pc, min_m, max_m)

          if self.bounding_box3d_topic != "NONE" and self.bounding_box3d_msg is not None:
            clip_box_msg = copy.deepcopy(self.bounding_box3d_msg)
            center = clip_box_msg.box_center_m
            extent = clip_box_msg.box_extent_xyz_m
            rotation = clip_box_msg.box_rotation_rpy_deg
            o3d_pc = nepi_pc.clip_bounding_box(o3d_pc, center, extent, rotation)



          k_points = self.node_if.get_param('process/uniform_downsample_k_points')
          if k_points > 0:
            o3d_pc = nepi_pc.uniform_down_sampling(o3d_pc, k_points)

          num_neighbors = self.node_if.get_param('process/outlier_removal_num_neighbors')   
          if num_neighbors > 0:
            statistical_outlier_removal_std_ratio = 2.0
            [o3d_pc, ind] = nepi_pc.statistical_outlier_removal(o3d_pc, num_neighbors, statistical_outlier_removal_std_ratio)

          voxel_size_m = self.node_if.get_param('process/voxel_downsample_size')
          if voxel_size_m > 0:
            o3d_pc = nepi_pc.voxel_down_sampling(o3d_pc, voxel_size_m)

          # Publish and Save Pointcloud Data
          if pc_has_subscribers:
            # ToDo Convert to map frame if selected
            if not nepi_sdk.is_shutdown():
              self.pc_if.publish_o3d_pc(o3d_pc, timestamp=current_time, frame_id=ros_frame_id)

          if pc_save is True:
            self.save_data_if.save_pc2file('pointcloud',o3d_pc,current_time, save_check = False)
            
          render_enable = self.node_if.get_param('render/render_enable')
	  
          if need_img and render_enable:
            o3d_img = None
            # Render the pointcloud image
            img_width = self.node_if.get_param('render/image_width')
            img_height = self.node_if.get_param('render/image_height')
            start_range_ratio = self.node_if.get_param('render/start_range_ratio')
            stop_range_ratio = self.node_if.get_param('render/stop_range_ratio')
            zoom_ratio = self.node_if.get_param('render/zoom_ratio')
            rotate_ratio = self.node_if.get_param('render/rotate_ratio')
            tilt_ratio = self.node_if.get_param('render/tilt_ratio')
            cam_fov = self.node_if.get_param('render/cam_fov')
            cam_view = self.node_if.get_param('render/cam_view')
            cam_pos = self.node_if.get_param('render/cam_pos')
            cam_rot = self.node_if.get_param('render/cam_rot')

            # ToDo: Fix self pc_min_range_m and pc_max_range_m calcs
            min_range_m =  self.node_if.get_param('process/range_min_m')
            max_range_m =   self.node_if.get_param('process/range_max_m')


            delta_range_m = max_range_m - min_range_m
            self.clip_min_range_m = min_range_m + start_range_ratio  * delta_range_m
            self.clip_max_range_m = min_range_m + stop_range_ratio  * delta_range_m
            if start_range_ratio > 0 or stop_range_ratio < 1:
              o3d_pc = nepi_pc.range_clip_spherical( o3d_pc, self.clip_min_range_m, self.clip_max_range_m)

            if cam_pos[0] < 0:
              zoom_ratio = 1 - zoom_ratio
            cam_pos[0] = cam_pos[0] *zoom_ratio  # Apply IDX zoom control

            rotate_angle = (0.5 - rotate_ratio) * 2 * 180
            rotate_vector = [0, 0, rotate_angle]
            o3d_pc = nepi_pc.rotate_pc(o3d_pc, rotate_vector)

            tilt_angle = (0.5 - tilt_ratio) * 2 * 180
            tilt_vector = [0, tilt_angle, 0]
            o3d_pc = nepi_pc.rotate_pc(o3d_pc, tilt_vector)
            use_wbg = self.node_if.get_param('render/use_wbg')
            bg_color = [0,0,0,0]
            if use_wbg:
              bg_color = [1,1,1,1]
            ros_img_msg = None
            update_renderer = (self.img_renderer is None or self.img_renderer_mtl is None \
            or self.last_img_width != img_width or self.last_img_height != img_height or self.last_fov != cam_fov \
            or use_wbg != self.last_bg_white)
            self.last_bg_white = use_wbg
            if update_renderer:
              if self.img_renderer is not None:
                self.img_renderer = None
                time.sleep(1)
              # Create point cloud renderer
              self.msg_if.pub_warn("Creating new pointcloud renderer")
              self.img_renderer = nepi_pc.create_img_renderer(img_width=img_width,img_height=img_height, fov=cam_fov, background = bg_color)
              self.img_renderer_mtl = nepi_pc.create_img_renderer_mtl(shader = "defaultLit")
              self.img_renderer = nepi_pc.remove_img_renderer_geometry(self.img_renderer)
            else:
              self.img_renderer = nepi_pc.add_img_renderer_geometry(o3d_pc,self.img_renderer, self.img_renderer_mtl)
              o3d_img = nepi_pc.render_img(self.img_renderer,cam_view,cam_pos,cam_rot)
              self.img_renderer = nepi_pc.remove_img_renderer_geometry(self.img_renderer)
            self.last_img_width = img_width
            self.last_img_height = img_height
            self.last_fov = cam_fov


            # Publish and Save Pointcloud Image Data
            if o3d_img is not None:
              if self.image_if is not None:
                self.image_if.publish_cv2_img(o3d_img, timestamp=current_time, frame_id=ros_frame_id)

              if img_save is True:
                 self.save_data_if.save_ros_img2file('pointcloud_image',ros_img_msg,current_time, save_check = False)
          
      else: # Data Empty
          nepi_sdk.sleep(0.1)
    else: # No data available
        nepi_sdk.sleep(0.25)
    nepi_sdk.sleep(0.01) # Yield
  
      
 

                

  #######################
  # Utility Funcitons


  def getClipFunction(self,sel):
    if sel == "X":
      clip_function = nepi_pc.range_clip_x_axis
    elif sel == "Y":
      clip_function = nepi_pc.range_clip_y_axis
    elif sel == "Z":
      clip_function = nepi_pc.range_clip_z_axis
    else: 
      clip_function = nepi_pc.range_clip_spherical
    return clip_function

  def getAvailableFrame3DList(self):
    frames_dict = yaml.safe_load(self.tf_buffer.all_frames_as_yaml())
    frame3d_list = list(frames_dict.keys())
    frame3d_list.insert(0,'map')
    return frame3d_list

  def getFrame3DTransformsMsg(self):
    transforms_dict = self.node_if.get_param('transforms_dict')
    transforms_topic_list = []
    transforms_list = []
    for topic in transforms_dict.keys():
      transforms_topic_list.append(topic)
      transform=transforms_dict[topic]
      transforms_list.append(transform)
    return (transforms_topic_list), str(transforms_list)

  def getTransformFromMsg(self,transform_msg):
    x = transform_msg.translate_vector.x
    y = transform_msg.translate_vector.y
    z = transform_msg.translate_vector.z
    roll = transform_msg.rotate_vector.x
    pitch = transform_msg.rotate_vector.y
    yaw = transform_msg.rotate_vector.z
    heading = transform_msg.heading_offset
    transform = [x,y,z,roll,pitch,yaw,heading]
    return transform
  
  def transformPointcloud(self, o3d_pc, transform):
    x = transform[0]
    y = transform[1]
    z = transform[2]
    translation_vector = [x, y, z]
    roll = transform[3]
    pitch = transform[4]
    yaw = transform[5]
    rotate_vector = [roll, pitch, yaw]
    o3d_pc = nepi_pc.translate_pc(o3d_pc, translation_vector)
    o3d_pc = nepi_pc.rotate_pc(o3d_pc, rotate_vector)
    return o3d_pc


  def addTransformToDict(self,transform_msg):
    topic = transform_msg.topic_namespace
    transforms_dict = self.node_if.get_param('transforms_dict')
    transforms_dict[topic] = self.getTransformFromMsg(transform_msg.transform)
    self.node_if.set_param('transforms_dict',transforms_dict)

  def removeTransformFromDict(self,topic_namespace):
    transforms_dict = self.node_if.get_param('transforms_dict')
    if topic_namespace in transforms_dict:
      transforms_dict.pop(topic_namespace)
    self.node_if.get_param('transforms_dict',transforms_dict)

    
  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  NepiPointcloudViewerApp()






