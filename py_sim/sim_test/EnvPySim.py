# -*- coding: utf-8 -*-
"""
Created on Wed Jun 20 10:40:52 2018

@author: Haksu
"""

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
class EnvCarMaker(object):
    def __init__(self, udp=False, IP='166.104.169.15', pyPort=20002, sendPort=20004):                
        self.IP = IP
        self.pyPort = pyPort            
        self.sendPort = sendPort           
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((IP, pyPort))
        
        self.distEgo2Pred_old = 80
        
    def sendAction(self, action, action_dim, resume_sig, start_sig):
        Bytesend = bytearray((action_dim + 2) * 16)
        for i in range(action_dim):
            Bytesend[i*8:(i+1)*8] = struct.pack('<d',action[i])            
            
        Bytesend[action_dim*8:action_dim*8+8] = struct.pack('<d',resume_sig)
        Bytesend[action_dim*8+8:action_dim*8+16] = struct.pack('<d',start_sig)
        SendData = bytes(Bytesend)
        self.sock.sendto(SendData,(self.IP, self.sendPort))
        
    def recvState(self,):
        names = ['spdEgoVeh','distSet','distEgo2Pred','spdRel','collision','done']
        Observation = col.namedtuple('Observation',names)
        data, addr = self.sock.recvfrom(160) 
        
        return Observation(spdEgoVeh= float(struct.unpack('<d' , data[0:8])[0]),
                           distSet= float(struct.unpack('<d' , data[8:16])[0]),
                           distEgo2Pred= float(struct.unpack('<d' , data[16:24])[0]),
                           spdRel= float(struct.unpack('<d' , data[24:32])[0]) ,
                           collision= float(struct.unpack('<d' , data[32:40])[0]),
                           done= float(struct.unpack('<d' , data[40:48])[0]))
    
    def getReward(self, ob=[]):        
        self.distSet = ob.distSet
        self.distEgo2Pred = ob.distEgo2Pred                         
        self.distErr = abs(self.distEgo2Pred - self.distSet)    
        
        if (self.distEgo2Pred - self.distEgo2Pred_old) < 0: #Decreasing inter-vehicle distance
           if self.distErr < 5:
               reward_tmp = 50
           elif self.distErr < 15:
              reward_tmp = 5
           else:
               if self.distEgo2Pred < 10:
                   reward_tmp = -100
               elif self.distEgo2Pred < 30:
                   reward_tmp = -5
               elif self.distEgo2Pred < 80:
                   reward_tmp = 2
               else:
                   reward_tmp = 1  
        else: #Increasing inter-vehicle distance
           if self.distErr < 5:
               reward_tmp = 50
           elif self.distErr < 15:
              reward_tmp = 5
           else:
               if self.distEgo2Pred < 10:
                   reward_tmp = -100
               elif self.distEgo2Pred < 30:
                   reward_tmp = 1
               elif self.distEgo2Pred < 80:
                   reward_tmp = -1
               else:
                   reward_tmp = -100
              
        self.distEgo2Pred_old = self.distEgo2Pred
        
        return reward_tmp
    
    def step(self, action, action_dim, resume_sig, start_sig):        
        self.sendAction(action, action_dim, resume_sig, start_sig)
        ob = self.recvState()      
        reward = self.getReward(ob)
        done = ob.done
        
        return ob, reward, done,
    
    