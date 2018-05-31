# -*- coding: utf-8 -*-
"""
Road data management
==============================================================

Author
~~~~~~~~~~~~~
* kyunghan <kyunghah.min@gmail.com>

Description
~~~~~~~~~~~~~
* Road data management modules
    * set_dir: set data config directory
    * load_mat: load MATLAB files

Update
~~~~~~~~~~~~~
* [18/05/31] - Initial release - kyunghan
"""
import scipy.io as io
import os
#import os

def set_dir(Data_dir):
    globals()['loc_dir'] = Data_dir
    globals()['loc_cdir'] = os.getcwd()


def load_mat(file_name):
    os.chdir(globals()['loc_dir'])
    road_data = io.loadmat(file_name)
    os.chdir(globals()['loc_cdir'])
    print('========== Import road data : ' + file_name + ' ==========')
    return road_data
