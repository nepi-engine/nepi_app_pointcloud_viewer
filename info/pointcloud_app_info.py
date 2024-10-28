#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

APP_NAME = 'Pointcloud_Viewer' # Use in display menus
FILE_TYPE = 'APP'
APP_DICT = dict(
    description = 'Application for combining, processing, and rendering pointcloud data topics',
    pkg_name = 'nepi_app_pointcloud',
    group_name = 'POINTCLOUD',
    config_file = 'app_pointcloud.yaml',
    app_file = 'pointcloud_app_node.py',
    node_name = 'app_pointcloud'
)
RUI_DICT = dict(
    rui_menu_name = 'Pointcloud Viewer', # RUI menu name or "None" if no rui support
    rui_files = ['NepiAppPointcloud.js','NepiAppPointcloudProcess.js','NepiAppPointcloudRender.js'],
    rui_main_file ='NepiAppPointcloud.js',
    rui_main_class = 'PointcloudApp',
)


