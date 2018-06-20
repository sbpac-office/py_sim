'''
To be continue !!
'''
#%% 0. Import python lib modules
import matplotlib.pyplot as plt
import numpy as np
import os
import sys
import random
# Set initial path
base_dir = os.path.abspath('F:/01_GRG/03_Project/01_CX/97_Git/py_sim/py_sim')
data_dir = os.path.abspath('F:/01_GRG/03_Project/01_CX/97_Git/py_sim/py_sim/data_roadxy')
conf_dir = os.path.abspath('F:/01_GRG/03_Project/01_CX/97_Git/py_sim/py_sim/data_config')
test_dir = os.path.abspath('F:/01_GRG/03_Project/01_CX/97_Git/py_sim/py_sim/sim_test')
print('Base directory: ', base_dir)
sys.path.append(base_dir);
sys.path.append(data_dir);
sys.path.append(conf_dir);
sys.path.append(test_dir);
# Import package lib modules
from model_vehicle import Mod_Veh, Mod_Body
from model_powertrain import Mod_PowerTrain
from model_maneuver import Mod_Behavior, Mod_Driver
from model_environment import Mod_Env
from sub_type_def import type_DataLog, type_drvstate
from sub_utilities import Filt_LowPass
from data_roadxy import get_roadxy

#%%
class 
