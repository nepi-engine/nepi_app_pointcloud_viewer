#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

APP_NAME = 'Pointcloud' # Use in display menus
DESCRIPTION = 'Application for combining, processing, and rendering pointcloud data topics'
PKG_NAME = 'nepi_app_pointcloud'
APP_FILE = 'pointcloud_app_node.py'
NODE_NAME = 'app_pointcloud'
RUI_FILES = ['NepiAppPointcloud.js','NepiAppPointcloudProcess.js','NepiAppPointcloudRender.js']
RUI_MAIN_FILE = 'NepiAppPointcloud.js'
RUI_MAIN_CLASS = 'PointcloudApp'
RUI_MENU_NAME = 'Pointcloud Viewer'

