# -*- coding: utf-8 -*-
"""
Config data management
==============================================================

Author
~~~~~~~~~~~~~
* kyunghan <kyunghah.min@gmail.com>

Description
~~~~~~~~~~~~~
* Config data management modules
    * set_dir: set data config directory
    * load_mat: load MATLAB files
    * load_xls: load Excel files

Update
~~~~~~~~~~~~~
* [18/05/31] - Initial release - kyunghan
"""
# import python lib modules
import scipy.io as io
import pandas as pd
import os

def set_dir(Data_dir):
    globals()['loc_dir'] = Data_dir
    globals()['loc_cdir'] = os.getcwd()

def load_mat(file_name):
    os.chdir(globals()['loc_dir'])
    road_data = io.loadmat(file_name)
    os.chdir(globals()['loc_dir'])
    print('========== Import config data : ' + file_name + ' ==========')
    return road_data

def load_xls(file_name):
    os.chdir(globals()['loc_dir'])
    road_data = pd.read_excel(file_name).values
    os.chdir(globals()['loc_dir'])
    print('========== Import config data : ' + file_name + ' ==========')
    return road_data
