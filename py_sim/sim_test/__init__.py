# -*- coding: utf-8 -*-
"""
sim test sub pakcage overview
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Author
-------
* kyunghan <kyunghah.min@gmail.com>

Description
-----------
* Sub package for simulation test

Test modules
------------
* Sc1: Set acc, brake position
* Sc2: Set steering angle
* Sc3: Control vehicle speed with driver
* Sc4: Control vehicle position in environment
* Sc5: Drive vehicle in imported environment
* Sc6: Cruise vehicle in imported environment (TBD)

Update
-------
* [18/05/31] - Initial release - kyunghan

Notice
-------
* Run each test scenario in main condition::

    >>> (excute) sim1_acc_in.py
    (result) matplotlib figures
    ...

"""

import os
import sys
#import py_sim
__Name__ = 'py_sim'
__Version__ = '1.0'
__Manager__ = 'Kyunghan'
# data direction config
base_dir = os.path.abspath('..')
data_dir = os.path.abspath('..\data_roadxy')
conf_dir = os.path.abspath('..\data_config')
test_dir = os.path.abspath('..\sim_test')
print('Base directory: ', base_dir)
sys.path.append(base_dir);
sys.path.append(data_dir);
sys.path.append(conf_dir);
sys.path.append(test_dir);
