#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

# Sample NEPI Process Script.
# 1. Converts ROS pointcloud2 to Open3d pointcloud
# 2. Blank area for custom code
# 3. Converts Open3d pointcloud back to ROS pointcloud2


#ToDo
#- Add pointcloud render enable control to node params, add node subscriber, add to to RenderStatus msg, and have asher add control to RUI
#- Fix the nepi_pc get min and max bounds 
#- Create add_pointclouds function in nepi_pc.py that supports combining bw and rgb pointclouds and replace in node Add process
#- Implement age_filter before pointcloud combine process
#- Apply transforms to each point cloud before combining
#- Add fit_pointclouds function to nepi_pc.py and add to node comnbine options
#- Get pointcloud node integrated and working with nepi_engine_ws codebase.  Allready in there, just not working when enabled in launch file
#- Add 3D bounding box selection and clipping group with new RUI selection box/section/page
#- Improve pointcloud pub latency. Break rendering into its own node that this node starts up.  The new node would subscribe to this noodes pointcloud topic 

import os
# ROS namespace setup
NEPI_BASE_NAMESPACE = '/nepi/s2x/'
#os.environ["ROS_NAMESPACE"] = NEPI_BASE_NAMESPACE[0:-1]
import rospy

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
from nepi_ros_interfaces.msg import IDXStatus, RangeWindow, ImageSize, \
  Frame3DTransform, Frame3DTransformUpdate, BoundingBox3D
from nepi_app_pointcloud.msg import PointcloudSelectionStatus,PointcloudProcessStatus,PointcloudRenderStatus

# For Testing 
from std_msgs.msg import Header
from sensor_msgs.msg import PointField
from sensor_msgs import point_cloud2

from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_save
from nepi_edge_sdk_base import nepi_msg
from nepi_edge_sdk_base import nepi_pc 
from nepi_edge_sdk_base import nepi_img 

from nepi_edge_sdk_base.save_data_if import SaveDataIF
from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF




#########################################

# Factory Control Values
Factory_Combine_Option = 'Add'
Factory_Frame_3d = 'nepi_center_frame'
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

Render_Background = [0, 0, 0, 0]

UPDATE_POINTCLOUD_SUBS_RATE_HZ = 1
UPDATE_DATA_PRODUCTS_RATE_HZ = 10

ZERO_TRANSFORM = [0,0,0,0,0,0,0]

STANDARD_IMAGE_SIZES = ['630 x 900','720 x 1080','955 x 600','1080 x 1440','1024 x 768 ','1980 x 2520','2048 x 1536','2580 x 2048','3648 x 2736']

#########################################
# Node Class
#########################################

class NepiPointcloudApp(object):
  combine_options = ["Add"]
  data_products = ["pointcloud","pointcloud_image"]
  frame3d_list = ['nepi_center_frame','map']
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

  view_img_pub = None
  acquiring = False

  pc_has_subscribers = False
  img_has_subscribers = False
  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "app_pointcloud" # Can be overwitten by luanch command
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    nepi_msg.createMsgPublishers(self)
    nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
    ##############################
    self.initParamServerValues(do_updates = False)
    self.resetParamServer(do_updates = False)
   

    # Set up save data and save config services ########################################################
    self.save_data_if = SaveDataIF(data_product_names = self.data_products)
    # Temp Fix until added as NEPI ROS Node
    self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.initParamServerValues, 
                                 paramsModifiedCallback=self.updateFromParamServer)


    ## App Setup ########################################################
    app_reset_app_sub = rospy.Subscriber('~reset_app', Empty, self.resetAppCb, queue_size = 10)
    self.initParamServerValues(do_updates=False)

    # Pointcloud Selection Setup ########################################################
    sel_reset_controls_sub = rospy.Subscriber("~reset_controls", Empty, self.resetSelectionControlsCb, queue_size = 10)
    sel_add_pc_sub = rospy.Subscriber('~add_pointcloud', String, self.addPointcloudCb, queue_size = 10)
    sel_remove_pc_sub = rospy.Subscriber('~remove_pointcloud', String, self.removePointcloudCb, queue_size = 10)
    sel_primary_pc_sub = rospy.Subscriber('~set_primary_pointcloud', String, self.setPrimaryPointcloudCb, queue_size = 10)
    sel_age_filter_sub = rospy.Subscriber("~set_age_filter", Float32, self.setAgeFilterCb, queue_size = 10)
    sel_upate_transform_sub = rospy.Subscriber('~update_transform', Frame3DTransformUpdate, self.updateTransformCb, queue_size = 10)
    sel_remove_transform_sub = rospy.Subscriber('~remove_transform', String, self.removeTransformCb, queue_size = 10)
    sel_combine_option = rospy.Subscriber('~set_combine_option', String, self.setCombineOptionCb, queue_size = 10)
    sel_frame_3d_sub = rospy.Subscriber('~set_frame_3d', String, self.setFrame3dCb, queue_size = 10)

    self.sel_status_pub = rospy.Publisher("~status", PointcloudSelectionStatus, queue_size=1, latch=True)

    # Pointcloud Process Setup ########################################################
    proc_reset_controls_sub = rospy.Subscriber("~process/reset_controls", Empty, self.resetProcessControlsCb, queue_size = 10)
    proc_range_clip_sub = rospy.Subscriber('~process/set_clip_enable', Bool, self.clipEnableCb, queue_size = 10)
    proc_set_clip_sel_topic_sub = rospy.Subscriber('~process/set_clip_selection', String, self.setClipSelectionCb, queue_size = 10)
    proc_range_meters_sub = rospy.Subscriber('~process/set_range_clip_m', RangeWindow, self.setRangeMetersCb, queue_size = 10)
    proc_set_clip_box_topic_sub = rospy.Subscriber('~process/set_clip_bounding_box3d_topic', String, self.setClipBoxTopicCb, queue_size = 10)
    proc_voxel_downsample_size_sub = rospy.Subscriber("~process/set_voxel_downsample_size", Float32, self.setVoxelSizeCb, queue_size = 10)
    proc_uniform_downsample_k_points_sub = rospy.Subscriber("~process/uniform_downsample_k_points", Int32, self.setUniformPointsCb, queue_size = 10) 
    proc_outlier_removal_num_neighbors_sub = rospy.Subscriber("~process/outlier_removal_num_neighbors", Int32, self.setOutlierNumCb, queue_size = 10)


    self.proc_status_pub = rospy.Publisher("~process/status", PointcloudProcessStatus, queue_size=1, latch=True)
    self.proc_pc_pub = rospy.Publisher("~pointcloud", PointCloud2, queue_size=1)

    # Pointcloud Render Subscribers ########################################################
    view_reset_controls_sub = rospy.Subscriber("~render/reset_controls", Empty, self.resetRenderControlsCb, queue_size = 10)
    view_image_size_sub = rospy.Subscriber("~render/set_image_size", ImageSize, self.setImageSizeCb, queue_size = 10)
    view_range_ratios_sub = rospy.Subscriber("~render/set_range_ratios", RangeWindow, self.setRangeRatiosCb, queue_size = 10)
    view_zoom_ratio_sub = rospy.Subscriber("~render/set_zoom_ratio", Float32, self.setZoomRatioCb, queue_size = 10)
    view_rotate_ratio_sub = rospy.Subscriber("~render/set_rotate_ratio", Float32, self.setRotateRatioCb, queue_size = 10)
    view_tilt_ratio_sub = rospy.Subscriber("~render/set_tilt_ratio", Float32, self.setTiltRatioCb, queue_size = 10)
    view_cam_fov_sub = rospy.Subscriber("~render/set_camera_fov", Int32, self.setCamFovCb, queue_size = 10)
    view_cam_view_sub = rospy.Subscriber("~render/set_camera_view", Vector3, self.setCamViewCb, queue_size = 10)
    view_cam_position_sub = rospy.Subscriber("~render/set_camera_position", Vector3, self.setCamPositionCb, queue_size = 10)
    view_cam_rotate_sub = rospy.Subscriber("~render/set_camera_rotation", Vector3, self.setCamRotationCb, queue_size = 10)
    render_enable_sub = rospy.Subscriber("~render/set_render_enable", Bool, self.setRenderEnableCb, queue_size = 10)

    self.view_status_pub = rospy.Publisher("~render/status", PointcloudRenderStatus, queue_size=1, latch=True)



    # Give publishers time to setup
    time.sleep(1)

    # Publish Status
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    self.publish_selection_status()
    self.publish_process_status()
    self.publish_render_status()


    ## Start Pointcloud Subscriber Update Process
    nepi_ros.timer(nepi_ros.duration(self.update_pointcloud_subs_interval_sec), self.updatePointcloudSubsThread)
    nepi_ros.timer(nepi_ros.duration(self.update_data_products_interval_sec), self.updateDataProductsThread)

    ## Initiation Complete
    nepi_msg.publishMsgInfo(self,"Initialization Complete")
    nepi_ros.spin()




###################
## App Callbacks

  def resetAppCb(self,msg):
    self.resetApp()

  def resetApp(self):
    nepi_ros.set_param(self,'~selected_pointclouds', [])
    nepi_ros.set_param(self,'~primary_pointcloud', "None")
    nepi_ros.set_param(self,'~age_filter_s', Factory_Age_Filter_S)
    nepi_ros.set_param(self,'~frame_3d', Factory_Frame_3d)
    nepi_ros.set_param(self,'~transforms_dict', dict())
    nepi_ros.set_param(self,'~combine_option', Factory_Combine_Option)
    
    nepi_ros.set_param(self,'~process/clip_enabled', Factory_Clip_Enabled)
    nepi_ros.set_param(self,'~process/clip_selection', Factory_Clip_Selection)
    nepi_ros.set_param(self,'~process/range_min_m',  Factory_Clip_Min_Range_M)
    nepi_ros.set_param(self,'~process/range_max_m',  Factory_Clip_Max_Range_M)
    self.bounding_box3d_topic = "NONE"
    nepi_ros.set_param(self,'~process/voxel_downsample_size',Factory_Voxel_DownSample_Size)
    nepi_ros.set_param(self,'~process/uniform_downsample_k_points',Factory_Uniform_DownSample_K_Points)
    nepi_ros.set_param(self,'~process/outlier_removal_num_neighbors',Factory_Outlier_Removal_Num_Neighbors)


    nepi_ros.set_param(self,'~render/image_width',  Factory_Image_Width)
    nepi_ros.set_param(self,'~render/image_height', Factory_Image_Height)
    nepi_ros.set_param(self,'~render/start_range_ratio',  Factory_Start_Range_Ratio)
    nepi_ros.set_param(self,'~render/stop_range_ratio', Factory_Stop_Range_Ratio)
    nepi_ros.set_param(self,'~render/zoom_ratio', Factory_Zoom_Ratio)
    nepi_ros.set_param(self,'~render/rotate_ratio', Factory_Rotate_Ratio)
    nepi_ros.set_param(self,'~render/tilt_ratio', Factory_Tilt_Ratio)
    nepi_ros.set_param(self,'~render/cam_fov', Factory_Cam_FOV)
    nepi_ros.set_param(self,'~render/cam_view', Factory_Cam_View)
    nepi_ros.set_param(self,'~render/cam_pos', Factory_Cam_Pos)
    nepi_ros.set_param(self,'~render/cam_rot', Factory_Cam_Rot)
    nepi_ros.set_param(self,'~render/render_enable', Factory_Render_Enable)
   
    self.publish_selection_status()
    self.publish_process_status()
    self.publish_render_status()

  ###################
  ## Selection Callbacks

  def resetSelectionControlsCb(self,msg):
    self.resetSelectionControls()
  
  def resetSelectionControls(self,do_updates = True):
    #nepi_ros.set_param(self,'~selected_pointclouds', self.init_selected_pointclouds)
    #nepi_ros.set_param(self,'~primary_pointcloud', self.init_primary_pointcloud)
    nepi_ros.set_param(self,'~selected_pointclouds', [])
    nepi_ros.set_param(self,'~primary_pointcloud', "None")
    # Fix for now
    nepi_ros.set_param(self,'~age_filter_s', self.init_age_filter_s)
    nepi_ros.set_param(self,'~transforms_dict', self.init_transforms_dict)
    nepi_ros.set_param(self,'~combine_option', self.init_combine_option)
    if do_updates:
      self.publish_process_status()

  def addPointcloudCb(self,msg):
    ##nepi_msg.publishMsgInfo(self,str(msg))
    pc_topic = msg.data
    pc_topics = nepi_ros.get_param(self,'~selected_pointclouds',self.init_selected_pointclouds)
    add_topic = False
    if nepi_ros.check_for_topic(pc_topic):
      if pc_topic not in pc_topics:
        add_topic = True
      if add_topic:
        nepi_msg.publishMsgInfo(self,"Adding Pointcloud topic to registered topics: " + pc_topic)
        pc_topics.append(pc_topic)
    nepi_ros.set_param(self,'~selected_pointclouds',pc_topics)
    self.publish_selection_status()

  def removePointcloudCb(self,msg):
    ##nepi_msg.publishMsgInfo(self,str(msg))
    pc_topic = msg.data
    pc_topics = nepi_ros.get_param(self,'~selected_pointclouds',self.init_selected_pointclouds)
    remove_topic = False
    if pc_topic in pc_topics:
      remove_topic = True
    if remove_topic:
      nepi_msg.publishMsgInfo(self,"Removing Pointcloud topic from registered topics: " + pc_topic)
      pc_topics.remove(pc_topic)
    nepi_ros.set_param(self,'~selected_pointclouds',pc_topics)
    self.publish_selection_status()

  def setPrimaryPointcloudCb(self,msg):
    ##nepi_msg.publishMsgInfo(self,str(msg))
    pc_topic = msg.data
    pc_topics = nepi_ros.get_param(self,'~~pc_app/selected_pointclouds',self.init_primary_pointcloud)
    if pc_topic in pc_topics:
      nepi_ros.set_param(self,'~primary_pointcloud',pc_topic)
    else:
      nepi_msg.publishMsgInfo(self,"Ignoring Set Primary Pointcloud as it is not in selected list")
    self.publish_selection_status()

  def setAgeFilterCb(self,msg):
    #nepi_msg.publishMsgInfo(self,str(msg))
    val = msg.data
    if val >= 0:
      nepi_ros.set_param(self,'~age_filter_s',val)
    self.publish_selection_status()


  def updateTransformCb(self,msg):
    #nepi_msg.publishMsgInfo(self,str(msg))
    self.addTransformToDict(msg)
    self.publish_selection_status()

  def removeTransformCb(self,msg):
    #nepi_msg.publishMsgInfo(self,str(msg))
    topic_namespace = msg.data
    self.removeTransformFromDict(topic_namespace)
    self.publish_selection_status()

  def removePointcloudCb(self,msg):
    #nepi_msg.publishMsgInfo(self,str(msg))
    pc_topic = msg.data
    pc_topics = nepi_ros.get_param(self,'~selected_pointclouds',self.init_selected_pointclouds)
    if pc_topic in pc_topics:
      pc_topics.remove(pc_topic)
    nepi_ros.set_param(self,'~selected_pointclouds',pc_topics)
    self.publish_selection_status()   

  def setCombineOptionCb(self, msg):
      #nepi_msg.publishMsgInfo(self,str(msg))
      combine_option = msg.data
      if combine_option in self.combine_options:
        nepi_ros.set_param(self,'~combine_option', combine_option)
      else:
        nepi_msg.publishMsgInfo(self,'Pointcloud combine option: ' + combine_option + ' not valid option')
      self.publish_selection_status()
      

  ###################
  ## Process Callbacks
  def resetProcessControlsCb(self,msg):
    self.resetProcessControls()
  
  def resetProcessControls(self,do_updates = True):
    nepi_ros.set_param(self,'~process/clip_enabled', self.init_proc_clip_enabled )
    nepi_ros.set_param(self,'~process/clip_selection', self.init_proc_clip_selection )
    nepi_ros.set_param(self,'~process/range_min_m', self.init_proc_range_min_m)
    nepi_ros.set_param(self,'~process/range_max_m', self.init_proc_range_max_m)
    self.bounding_box3d_topic = "NONE"
    nepi_ros.set_param(self,'~process/voxel_downsample_size',self.init_proc_voxel_downsample_size)
    nepi_ros.set_param(self,'~process/uniform_downsample_k_points',self.init_proc_uniform_downsample_k_points)
    nepi_ros.set_param(self,'~process/outlier_removal_num_neighbors',self.init_proc_outlier_removal_num_neighbors)   
    if do_updates:
      self.publish_process_status()

  def clipEnableCb(self,msg):
    #nepi_msg.publishMsgInfo(self,str(msg))
    new_enable = msg.data
    nepi_ros.set_param(self,'~process/clip_enabled', new_enable)
    self.publish_process_status()

  def setClipSelectionCb(self,msg):
    #nepi_msg.publishMsgInfo(self,str(msg))
    sel = msg.data
    if sel in self.clip_options:
      nepi_ros.set_param(self,'~process/clip_selection', sel )
    self.publish_process_status()

  def setClipBoxTopicCb(self,msg):
    #nepi_msg.publishMsgInfo(self,str(msg))
    self.bounding_box3d_topic = msg.data
    topic = msg.data
    if topic != self.bounding_box3d_topic and self.bounding_box3d_sub is not None:
      self.bounding_box3d_sub.Unregister()
      self.bounding_box3d_sub = None
    if topic != "NONE":
      self.bounding_box3d_sub = rospy.Subscriber('~set_clip_target_topic', String, self.setClipTargetTopicCb, queue_size = 10)
    self.bounding_box3d_msg = None
    self.publish_process_status()

  def setRangeMetersCb(self,msg):
    #nepi_msg.publishMsgInfo(self,str(msg))
    range_min_m = msg.start_range
    range_max_m = msg.stop_range
    if range_min_m < range_max_m:
      nepi_ros.set_param(self,'~process/range_min_m', range_min_m)
      nepi_ros.set_param(self,'~process/range_max_m', range_max_m)
    self.publish_process_status()

  def setVoxelSizeCb(self,msg):
    #nepi_msg.publishMsgInfo(self,str(msg))
    val = msg.data
    if val >= 0:
      nepi_ros.set_param(self,'~process/voxel_downsample_size',val)
    self.publish_process_status()

  def setUniformPointsCb(self,msg):
    #nepi_msg.publishMsgInfo(self,str(msg))
    val = msg.data
    if val >= 0:
      nepi_ros.set_param(self,'~process/uniform_downsample_k_points',val)
    self.publish_process_status()

  def setOutlierNumCb(self,msg):
    #nepi_msg.publishMsgInfo(self,str(msg))
    val = msg.data
    if val >= 0:
      nepi_ros.set_param(self,'~process/outlier_removal_num_neighbors',val)
    self.publish_process_status()

  def setFrame3dCb(self, msg):
    #nepi_msg.publishMsgInfo(self,str(msg))
    frame_3d = msg.data
    frame3d_list = self.frame3d_list
    if frame_3d in frame3d_list:
      nepi_ros.set_param(self,'~frame_3d',frame_3d)
    self.publish_process_status()

###################
## Render Callbacks

  def resetRenderControlsCb(self,msg):
    self.resetRenderControls()

  def resetRenderControls(self,do_updates = True):
    nepi_ros.set_param(self,'~render/image_width',  self.init_image_width)
    nepi_ros.set_param(self,'~render/image_height', self.init_image_height)
    nepi_ros.set_param(self,'~render/start_range_ratio', self.init_view_start_range_ratio)
    nepi_ros.set_param(self,'~render/stop_range_ratio', self.init_view_stop_range_ratio)
    nepi_ros.set_param(self,'~render/zoom_ratio',self.init_view_zoom_ratio)
    nepi_ros.set_param(self,'~render/rotate_ratio',self.init_view_rotate_ratio)
    nepi_ros.set_param(self,'~render/tilt_ratio',self.init_view_tilt_ratio)
    nepi_ros.set_param(self,'~render/cam_fov', self.init_view_cam_fov)
    nepi_ros.set_param(self,'~render/cam_view',self.init_view_cam_view)
    nepi_ros.set_param(self,'~render/cam_pos',self.init_view_cam_pos)
    nepi_ros.set_param(self,'~render/cam_rot',self.init_view_cam_rot)
    nepi_ros.set_param(self,'~render/render_enable', self.init_render_enable)
    
    if do_updates:
      self.publish_render_status()


  def setImageSizeCb(self,msg):
    #nepi_msg.publishMsgInfo(self,str(msg))
    width = msg.width
    height = msg.height
    if width > 100 and width < 5000 and height > 100 and height < 5000:
      nepi_ros.set_param(self,'~render/image_width',  width)
      nepi_ros.set_param(self,'~render/image_height', height)
    self.publish_render_status()

  def setImageSizeIndCb(self,msg):
    #nepi_msg.publishMsgInfo(self,str(msg))
    index = msg.data
    if index <= len(STANDARD_IMAGE_SIZES):
      size_str = STANDARD_IMAGE_SIZES[index]
      size__split = size_str.split(" ")
      width = float(size__split[0])
      height = float(size_split[2])
      if width > 100 and width < 5000 and height > 100 and height < 5000:
        nepi_ros.set_param(self,'~render/image_width',  width)
        nepi_ros.set_param(self,'~render/image_height', height)
    self.publish_render_status()

  def setZoomRatioCb(self,msg):
    #nepi_msg.publishMsgInfo(self,str(msg))
    new_val = msg.data
    if new_val >= 0 and new_val <= 1 :
      nepi_ros.set_param(self,'~render/zoom_ratio',new_val)
    self.publish_render_status()

  def setZoomRatioCb(self,msg):
    #nepi_msg.publishMsgInfo(self,str(msg))
    new_val = msg.data
    if new_val >= 0 and new_val <= 1 :
      nepi_ros.set_param(self,'~render/zoom_ratio',new_val)
    self.publish_render_status()

  def setRotateRatioCb(self,msg):
    #nepi_msg.publishMsgInfo(self,str(msg))
    new_val = msg.data
    if new_val >= 0 and new_val <= 1 :
      nepi_ros.set_param(self,'~render/rotate_ratio',new_val)
    self.publish_render_status()

  def setTiltRatioCb(self,msg):
    #nepi_msg.publishMsgInfo(self,str(msg))
    new_val = msg.data
    if new_val >= 0 and new_val <= 1 :
      nepi_ros.set_param(self,'~render/tilt_ratio',new_val)
    self.publish_render_status()

  def setCamFovCb(self,msg):
    #nepi_msg.publishMsgInfo(self,str(msg))
    new_val = msg.data
    if new_val > 100:
      new_val = 100
    if new_val < 30:
      new_val = 30
    nepi_ros.set_param(self,'~render/cam_fov',new_val)
    self.publish_render_status()

  def setCamViewCb(self,msg):
    #nepi_msg.publishMsgInfo(self,str(msg))
    new_array = []
    new_array.append(msg.x)
    new_array.append(msg.y)
    new_array.append(msg.z)
    nepi_ros.set_param(self,'~render/cam_view',new_array)
    self.publish_render_status()

  def setCamPositionCb(self,msg):
    #nepi_msg.publishMsgInfo(self,str(msg))
    new_array = []
    new_array.append(msg.x)
    new_array.append(msg.y)
    new_array.append(msg.z)
    nepi_ros.set_param(self,'~render/cam_pos',new_array)
    self.publish_render_status()

  def setCamRotationCb(self,msg):
    #nepi_msg.publishMsgInfo(self,str(msg))
    new_array = []
    new_array.append(msg.x)
    new_array.append(msg.y)
    new_array.append(msg.z)
    nepi_ros.set_param(self,'~render/cam_rot',new_array)
    self.publish_render_status()
  
  def setRenderEnableCb(self,msg):
    render_enable = msg.data
    nepi_ros.set_param(self,'~render/render_enable', render_enable)
    self.publish_render_status()

  def setRangeRatiosCb(self,msg):
    #nepi_msg.publishMsgInfo(self,str(msg))
    min_ratio = msg.start_range
    max_ratio = msg.stop_range
    if min_ratio < max_ratio and min_ratio >= 0 and max_ratio <= 1:
      nepi_ros.set_param(self,'~render/start_range_ratio', min_ratio)
      nepi_ros.set_param(self,'~render/stop_range_ratio', max_ratio)
    self.publish_render_status()



  #######################
  ### Config Functions

  def saveConfigCb(self, msg):  # Just update class init values. Saving done by Config IF system
    pass # Left empty for sim, Should update from param server

  def setCurrentAsDefault(self):
    pass # We only use the param server, no member variables to apply to param server

  def updateFromParamServer(self):
    #nepi_msg.publishMsgWarn(self,("Debugging: param_dict = " + str(param_dict))
    #Run any functions that need updating on value change
    # Don't need to run any additional functions
    pass

  def initParamServerValues(self,do_updates = True):
      nepi_msg.publishMsgInfo(self,"Reseting param values to init values")
      self.init_selected_pointclouds = nepi_ros.get_param(self,'~selected_pointclouds', [])
      self.init_primary_pointcloud = nepi_ros.get_param(self,'~primary_pointcloud', "None")
      self.init_age_filter_s = nepi_ros.get_param(self,'~age_filter_s', Factory_Age_Filter_S)
      self.init_transforms_dict = nepi_ros.get_param(self,'~transforms_dict',dict())
      self.init_combine_option = nepi_ros.get_param(self,'~combine_option', Factory_Combine_Option)
    
      self.init_proc_clip_enabled = nepi_ros.get_param(self,'~process/clip_enabled', Factory_Clip_Enabled)
      self.init_proc_clip_selection = nepi_ros.get_param(self,'~process/clip_selection', Factory_Clip_Selection )
      self.init_proc_range_min_m = nepi_ros.get_param(self,'~process/range_min_m',  Factory_Clip_Min_Range_M)
      self.init_proc_range_max_m = nepi_ros.get_param(self,'~process/range_max_m',  Factory_Clip_Max_Range_M)
      self.init_proc_voxel_downsample_size = nepi_ros.get_param(self,'~process/voxel_downsample_size',Factory_Voxel_DownSample_Size)
      self.init_proc_uniform_downsample_k_points = nepi_ros.get_param(self,'~process/uniform_downsample_k_points',Factory_Uniform_DownSample_K_Points)
      self.init_proc_outlier_removal_num_neighbors = nepi_ros.get_param(self,'~process/outlier_removal_num_neighbors',Factory_Outlier_Removal_Num_Neighbors)
      self.init_proc_frame_3d = nepi_ros.get_param(self,'~frame_3d', Factory_Frame_3d)
    
      self.init_image_width = nepi_ros.get_param(self,'~render/image_width',  Factory_Image_Width)
      self.init_image_height = nepi_ros.get_param(self,'~render/image_height', Factory_Image_Height)
      self.init_view_start_range_ratio = nepi_ros.get_param(self,'~render/start_range_ratio',  Factory_Start_Range_Ratio)
      self.init_view_stop_range_ratio = nepi_ros.get_param(self,'~render/stop_range_ratio', Factory_Stop_Range_Ratio)
      self.init_view_zoom_ratio = nepi_ros.get_param(self,'~render/zoom_ratio', Factory_Zoom_Ratio)
      self.init_view_rotate_ratio = nepi_ros.get_param(self,'~render/rotate_ratio', Factory_Rotate_Ratio)
      self.init_view_tilt_ratio = nepi_ros.get_param(self,'~render/tilt_ratio', Factory_Tilt_Ratio)

      self.init_view_cam_fov = nepi_ros.get_param(self,'~render/cam_fov', Factory_Cam_FOV )
      self.init_view_cam_view = nepi_ros.get_param(self,'~render/cam_view', Factory_Cam_View)
      self.init_view_cam_pos = nepi_ros.get_param(self,'~render/cam_pos', Factory_Cam_Pos)
      self.init_view_cam_rot = nepi_ros.get_param(self,'~render/cam_rot', Factory_Cam_Rot)
      self.init_render_enable = nepi_ros.get_param(self,'~render/render_enable', Factory_Render_Enable)
      
      self.resetParamServer(do_updates)

  def resetParamServer(self,do_updates = True):
      nepi_ros.set_param(self,'~selected_pointclouds', self.init_selected_pointclouds)
      nepi_ros.set_param(self,'~process/clip_selection', self.init_proc_clip_selection )
      nepi_ros.set_param(self,'~age_filter_s',  self.init_age_filter_s)
      nepi_ros.set_param(self,'~transforms_dict', self.init_transforms_dict)
      nepi_ros.set_param(self,'~combine_option',  self.init_combine_option)
      self.resetSelectionControls(do_updates)
      self.resetProcessControls(do_updates)
      self.resetRenderControls(do_updates)
      if do_updates:
          self.updateFromParamServer()
          self.publish_selection_status()
          self.publish_process_status()
          self.publish_render_status()



  ###################
  ## Status Publishers
  def publish_selection_status(self):
    status_msg = PointcloudSelectionStatus()

    pointcloud_topic_list = nepi_ros.get_param(self,'~selected_pointclouds',self.init_selected_pointclouds)
    status_msg.selected_pointcloud_topics = str(pointcloud_topic_list)

    primary_pointcloud = nepi_ros.get_param(self,'~primary_pointcloud', self.init_primary_pointcloud)
    if primary_pointcloud == "None" and len(pointcloud_topic_list) > 0:
      primary_pointcloud = pointcloud_topic_list[0]
    elif primary_pointcloud != "None" and len(pointcloud_topic_list) == 0:
      primary_pointcloud = "None"
    nepi_ros.set_param(self,'~primary_pointcloud', primary_pointcloud)
    status_msg.primary_pointcloud_topic = primary_pointcloud
    status_msg.publishing_pointcloud_img = self.view_img_pub is not None

    age_filter_s = nepi_ros.get_param(self,'~age_filter_s', self.init_age_filter_s)
    status_msg.age_filter_s = age_filter_s


    status_msg.available_3d_frames = str(self.frame3d_list)
    status_msg.output_3d_frame = nepi_ros.get_param(self,'~frame_3d', self.init_proc_frame_3d) 

    transforms_dict = nepi_ros.get_param(self,'~transforms_dict',self.init_transforms_dict)
    for pc_topic in pointcloud_topic_list:
      if pc_topic not in transforms_dict.keys():
        transforms_dict[pc_topic] = ZERO_TRANSFORM
    nepi_ros.set_param(self,'~transforms_dict',transforms_dict)
    [status_msg.transforms_topic_list,status_msg.transforms_list] = self.getFrame3DTransformsMsg()

    status_msg.combine_options = str(self.combine_options)
    status_msg.combine_option = nepi_ros.get_param(self,'~combine_option', self.init_combine_option)

    range_meters = RangeWindow()
    range_meters.start_range =   self.pc_min_range_m
    range_meters.stop_range =   self.pc_max_range_m
    status_msg.range_min_max_m = range_meters

    self.sel_status_pub.publish(status_msg)


  def publish_process_status(self):
    status_msg = PointcloudProcessStatus()

    status_msg.clip_enabled = nepi_ros.get_param(self,'~process/clip_enabled', self.init_proc_clip_enabled )
    status_msg.clip_options = self.clip_options
    status_msg.clip_selection = nepi_ros.get_param(self,'~process/clip_selection', self.init_proc_clip_selection )
    range_meters = RangeWindow()
    range_meters.start_range =   nepi_ros.get_param(self,'~process/range_min_m', self.init_proc_range_min_m)
    range_meters.stop_range =   nepi_ros.get_param(self,'~process/range_max_m', self.init_proc_range_max_m)
    status_msg.clip_meters = range_meters

    status_msg.clip_target_topic = self.bounding_box3d_topic
    
    status_msg.voxel_downsample_size_m = nepi_ros.get_param(self,'~process/voxel_downsample_size',self.init_proc_voxel_downsample_size)
    status_msg.uniform_downsample_points = nepi_ros.get_param(self,'~process/uniform_downsample_k_points',self.init_proc_uniform_downsample_k_points)
    status_msg.outlier_k_points = nepi_ros.get_param(self,'~process/outlier_removal_num_neighbors',self.init_proc_outlier_removal_num_neighbors) 

    self.proc_status_pub.publish(status_msg)

  def publish_render_status(self):
    status_msg = PointcloudRenderStatus()

    status_msg.standard_image_sizes = str(STANDARD_IMAGE_SIZES)

    status_msg.image_width = nepi_ros.get_param(self,'~render/image_width',  self.init_image_width)
    status_msg.image_height = nepi_ros.get_param(self,'~render/image_height', self.init_image_height)

    range_meters = RangeWindow()
    range_meters.start_range =  nepi_ros.get_param(self,'~process/range_min_m', self.init_proc_range_min_m)
    range_meters.stop_range =   nepi_ros.get_param(self,'~process/range_max_m', self.init_proc_range_max_m)

    status_msg.range_min_max_m = range_meters

    range_ratios = RangeWindow()
    range_ratios.start_range =   nepi_ros.get_param(self,'~render/start_range_ratio', self.init_view_start_range_ratio)
    range_ratios.stop_range =   nepi_ros.get_param(self,'~render/stop_range_ratio', self.init_view_stop_range_ratio)
    status_msg.range_clip_ratios = range_ratios

    status_msg.zoom_ratio = nepi_ros.get_param(self,'~render/zoom_ratio',self.init_view_zoom_ratio)
    status_msg.rotate_ratio = nepi_ros.get_param(self,'~render/rotate_ratio',self.init_view_rotate_ratio)
    status_msg.tilt_ratio = nepi_ros.get_param(self,'~render/tilt_ratio',self.init_view_tilt_ratio)

    fov = nepi_ros.get_param(self,'~render/cam_fov', self.init_view_cam_fov )
    status_msg.camera_fov = fov

    view = nepi_ros.get_param(self,'~render/cam_view',self.init_view_cam_view)
    cam_view = Vector3()
    cam_view.x = view[0]
    cam_view.y = view[1]
    cam_view.z = view[2]
    status_msg.camera_view = cam_view

    pos = nepi_ros.get_param(self,'~render/cam_pos',self.init_view_cam_pos)
    cam_pos = Vector3()
    cam_pos.x = pos[0]
    cam_pos.y = pos[1]
    cam_pos.z = pos[2]
    status_msg.camera_position = cam_pos

    rot = nepi_ros.get_param(self,'~render/cam_rot',self.init_view_cam_rot)
    cam_rot = Vector3()
    cam_rot.x = rot[0]
    cam_rot.y = rot[1]
    cam_rot.z = rot[2]
    status_msg.camera_rotation = cam_rot
    
    render_enable = nepi_ros.get_param(self,'~render/render_enable',self.init_render_enable)
    status_msg.render_enable = render_enable

    self.view_status_pub.publish(status_msg)

  #######################
  # Data Product Threads

  def updatePointcloudSubsThread(self,timer):
    # Subscribe to topic pointcloud topics if not subscribed
    sel_topics = nepi_ros.get_param(self,'~selected_pointclouds',self.init_selected_pointclouds)
    for sel_topic in sel_topics:
      if sel_topic != "" and sel_topic not in self.pc_subs_dict.keys():
        if nepi_ros.check_for_topic(sel_topic):
          topic_uid = sel_topic.replace('/','')
          exec('self.' + topic_uid + '_pc = None')
          exec('self.' + topic_uid + '_timestamp = None')
          exec('self.' + topic_uid + '_frame = None')
          exec('self.' + topic_uid + '_lock = threading.Lock()')
          nepi_msg.publishMsgInfo(self,"Subscribing to topic: " + sel_topic)
          #nepi_msg.publishMsgInfo(self,"with topic_uid: " + topic_uid)
          pc_sub = rospy.Subscriber(sel_topic, PointCloud2, lambda msg: self.pointcloudCb(msg, sel_topic), queue_size = 10)
          self.pc_subs_dict[sel_topic] = pc_sub
          nepi_msg.publishMsgInfo(self,"Pointcloud: " + sel_topic + " registered")
    if len(list(self.pc_subs_dict.keys())) > 0 and self.view_img_pub is None:
      nepi_msg.publishMsgInfo(self,"Setting up pointcloud_image pub")
      self.view_img_pub = rospy.Publisher("~pointcloud_image", Image, queue_size=1)
      time.sleep(1)
    elif len(list(self.pc_subs_dict.keys())) == 0 and self.view_img_pub is not None:
      nepi_msg.publishMsgInfo(self,"Taking down pointcloud_image pub")
      self.view_img_pub.unregister()
      self.view_img_pub = None
      time.sleep(1)
    # Unregister pointcloud subscribers if not in selected pointclouds list
    unreg_topic_list = []
    for topic in self.pc_subs_dict.keys():
      if topic not in sel_topics:
          pc_sub = self.pc_subs_dict[topic]
          pc_sub.unregister()
          nepi_msg.publishMsgInfo(self,"Pointcloud: " + topic + " unregistered")
          unreg_topic_list.append(topic) # Can't change dictionary while looping through dictionary
    for topic in unreg_topic_list: 
          self.pc_subs_dict.pop(topic)
    # Update primary pointcloud if needed
    primary_pc = nepi_ros.get_param(self,'~primary_pointcloud', self.init_primary_pointcloud)
    #print(self.pc_subs_dict.keys())
    if primary_pc not in self.pc_subs_dict.keys():
      if len(self.pc_subs_dict.keys()) > 0:
        primary_pc = list(self.pc_subs_dict.keys())[0]
      else:
        primary_pc = "None"
      if primary_pc != "None":
        nepi_msg.publishMsgInfo(self,"Primary pointcloud set to: " + primary_pc)
    nepi_ros.set_param(self,'~primary_pointcloud', primary_pc)
    self.pc_has_subscribers = (self.proc_pc_pub.get_num_connections() > 0)
    if self.view_img_pub is not None:
      self.img_has_subscribers = (self.view_img_pub.get_num_connections() > 0)
    else:
      self.img_has_subscribers = False
    self.publish_selection_status()
    

  def pointcloudCb(self,msg,topic):
      if topic != "":
        topic_uid = topic.replace('/','')
        transforms_dict = nepi_ros.get_param(self,'~transforms_dict',self.init_transforms_dict)
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

    pc_has_subscribers = self.pc_has_subscribers
    pc_saving_is_enabled = self.save_data_if.data_product_saving_enabled('pointcloud')
    pc_should_save = pc_saving_is_enabled and self.save_data_if.data_product_should_save('pointcloud')
    pc_snapshot_enabled = self.save_data_if.data_product_snapshot_enabled('pointcloud')
    pc_save = (pc_saving_is_enabled and pc_should_save) or pc_snapshot_enabled
    need_pc = (pc_has_subscribers is True) or (pc_save is True) 

    img_has_subscribers = self.img_has_subscribers
    img_saving_is_enabled = self.save_data_if.data_product_saving_enabled('pointcloud_image')
    img_should_save = self.save_data_if.data_product_should_save('pointcloud_image')
    img_snapshot_enabled = self.save_data_if.data_product_snapshot_enabled('pointcloud_image')
    img_save = (img_saving_is_enabled and img_should_save) or img_snapshot_enabled
    need_img = (img_has_subscribers is True) or (img_save is True)

    ros_frame_id = nepi_ros.get_param(self,'~frame_3d', self.init_proc_frame_3d)
    topic_primary = nepi_ros.get_param(self,'~primary_pointcloud', self.init_primary_pointcloud)
    
    if (need_pc or need_img and topic_primary != "None"):
      o3d_pc = None
      # Combine selected 
      age_filter_s = nepi_ros.get_param(self,'~age_filter_s', self.init_age_filter_s)
      combine_option = nepi_ros.get_param(self,'~combine_option',self.init_combine_option)
      transforms_dict = nepi_ros.get_param(self,'~transforms_dict',self.init_transforms_dict)
      current_time = nepi_ros.get_rostime()
      pc_add_count = 0
      # Get priamary pointcloud
      topic_puid = topic_primary.replace('/','')
      if topic_primary in self.pc_subs_dict.keys():
        eval('self.' + topic_puid + '_lock').acquire()
        ros_timestamp_pr = eval('self.' + topic_puid + '_timestamp')
        primary_pc_frame = eval('self.' + topic_puid + '_frame')
        if ros_timestamp_pr is not None:
          pc_age =(current_time.to_sec() - ros_timestamp_pr.to_sec())
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
            ros_timestamp_add = None
            if topic in self.pc_subs_dict.keys():
              eval('self.' + topic_uid + '_lock').acquire()
              ros_timestamp_add = eval('self.' + topic_uid + '_timestamp')
              pc_frame_add = eval('self.' + topic_uid + '_frame')
              o3d_pc_add = eval('self.' + topic_uid + '_pc')
              eval('self.' + topic_uid + '_lock').release()
            #else:
              #nepi_msg.publishMsgInfo(self,"Combine pointcloud not registered yet: " + topic_puid)
            if ros_timestamp_add is not None:
              #pc_age = abs(current_time - ros_timestamp)
              #pc_age = pc_age.to_sec()
              pc_age =(current_time.to_sec() - ros_timestamp_add.to_sec())
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
          clip_enable = nepi_ros.get_param(self,'~process/clip_enabled', self.init_proc_clip_enabled )
          if clip_enable:
            min_m = nepi_ros.get_param(self,'~process/range_min_m', self.init_proc_range_min_m)
            max_m = nepi_ros.get_param(self,'~process/range_max_m', self.init_proc_range_max_m)
            clip_process = nepi_ros.get_param(self,'~process/clip_selection', self.init_proc_clip_selection )
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



          k_points = nepi_ros.get_param(self,'~process/uniform_downsample_k_points',self.init_proc_uniform_downsample_k_points)
          if k_points > 0:
            o3d_pc = nepi_pc.uniform_down_sampling(o3d_pc, k_points)

          num_neighbors = nepi_ros.get_param(self,'~process/outlier_removal_num_neighbors',self.init_proc_outlier_removal_num_neighbors)   
          if num_neighbors > 0:
            statistical_outlier_removal_std_ratio = 2.0
            [o3d_pc, ind] = nepi_pc.statistical_outlier_removal(o3d_pc, num_neighbors, statistical_outlier_removal_std_ratio)

          voxel_size_m = nepi_ros.get_param(self,'~process/voxel_downsample_size',self.init_proc_voxel_downsample_size)
          if voxel_size_m > 0:
            o3d_pc = nepi_pc.voxel_down_sampling(o3d_pc, voxel_size_m)

          # Publish and Save Pointcloud Data
          if pc_has_subscribers:
            # ToDo Convert to map frame if selected
            ros_pc_out_msg = nepi_pc.o3dpc_to_rospc(o3d_pc, stamp=current_time, frame_id=ros_frame_id)
            if not nepi_ros.is_shutdown():
              self.proc_pc_pub.publish(ros_pc_out_msg)

          if pc_save is True:
            nepi_save.save_pc2file(self,'pointcloud',o3d_pc,current_time, save_check = False)
            
          render_enable = nepi_ros.get_param(self,'~render/render_enable', self.init_render_enable)
	  
          if need_img and render_enable:
            # Render the pointcloud image
            img_width = nepi_ros.get_param(self,'~render/image_width',  self.init_image_width)
            img_height = nepi_ros.get_param(self,'~render/image_height', self.init_image_height)
            start_range_ratio = nepi_ros.get_param(self,'~render/start_range_ratio', self.init_view_start_range_ratio)
            stop_range_ratio = nepi_ros.get_param(self,'~render/stop_range_ratio', self.init_view_stop_range_ratio)
            zoom_ratio = nepi_ros.get_param(self,'~render/zoom_ratio',self.init_view_zoom_ratio)
            rotate_ratio = nepi_ros.get_param(self,'~render/rotate_ratio',self.init_view_rotate_ratio)
            tilt_ratio = nepi_ros.get_param(self,'~render/tilt_ratio',self.init_view_tilt_ratio)
            cam_fov = nepi_ros.get_param(self,'~render/cam_fov', self.init_view_cam_fov )
            cam_view = nepi_ros.get_param(self,'~render/cam_view',self.init_view_cam_view)
            cam_pos = nepi_ros.get_param(self,'~render/cam_pos',self.init_view_cam_pos)
            cam_rot = nepi_ros.get_param(self,'~render/cam_rot',self.init_view_cam_rot)

            # ToDo: Fix self pc_min_range_m and pc_max_range_m calcs
            min_range_m =  nepi_ros.get_param(self,'~process/range_min_m', self.init_proc_range_min_m)
            max_range_m =   nepi_ros.get_param(self,'~process/range_max_m', self.init_proc_range_max_m)


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
          
            ros_img_msg = None
            update_renderer = (self.img_renderer is None or self.img_renderer_mtl is None or self.last_img_width != img_width or self.last_img_height != img_height or self.last_fov != cam_fov)
            if update_renderer:
              # Create point cloud renderer
              self.img_renderer = nepi_pc.create_img_renderer(img_width=img_width,img_height=img_height, fov=cam_fov, background = Render_Background)
              self.img_renderer_mtl = nepi_pc.create_img_renderer_mtl()
              self.img_renderer = nepi_pc.remove_img_renderer_geometry(self.img_renderer)
            else:
              self.img_renderer = nepi_pc.add_img_renderer_geometry(o3d_pc,self.img_renderer, self.img_renderer_mtl)
              o3d_img = nepi_pc.render_img(self.img_renderer,cam_view,cam_pos,cam_rot)
              ros_img_msg = nepi_pc.o3dimg_to_rosimg(o3d_img, stamp=current_time, frame_id=ros_frame_id)
              self.img_renderer = nepi_pc.remove_img_renderer_geometry(self.img_renderer)
            self.last_img_width = img_width
            self.last_img_height = img_height
            self.last_fov = cam_fov


            # Publish and Save Pointcloud Image Data
            if ros_img_msg is not None:
              if self.view_img_pub is not None:
                if img_has_subscribers:
                  if not nepi_ros.is_shutdown():
                    self.view_img_pub.publish(ros_img_msg)

              if img_save is True:
                 nepi_save.save_ros_img2file(self,'pointcloud_image',ros_img_msg,current_time, save_check = False)
          
      else: # Data Empty
          nepi_ros.sleep(0.1)
    else: # No data available
        nepi_ros.sleep(0.25)
    nepi_ros.sleep(0.01) # Yield
  
      
 

                

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
    transforms_dict = nepi_ros.get_param(self,'~transforms_dict',self.init_transforms_dict)
    transforms_topic_list = []
    transforms_list = []
    for topic in transforms_dict.keys():
      transforms_topic_list.append(topic)
      transform=transforms_dict[topic]
      transforms_list.append(transform)
    return str(transforms_topic_list), str(transforms_list)

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
    transforms_dict = nepi_ros.get_param(self,'~transforms_dict',self.init_transforms_dict)
    transforms_dict[topic] = self.getTransformFromMsg(transform_msg.transform)
    nepi_ros.set_param(self,'~transforms_dict',transforms_dict)

  def removeTransformFromDict(self,topic_namespace):
    transforms_dict = nepi_ros.get_param(self,'~transforms_dict',self.init_transforms_dict)
    if topic_namespace in transforms_dict:
      transforms_dict.pop(topic_namespace)
    nepi_ros.set_param(self,'~transforms_dict',transforms_dict)

    
  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    nepi_msg.publishMsgInfo(self,"Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  NepiPointcloudApp()






