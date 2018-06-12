# -*- coding: utf-8 -*-
"""
Application: Smart regenerative braking based on reinforment learning
======================================================================

Author
~~~~~~~~~~~~~
* kyunghan <kyunghah.min@gmail.com>

Description
~~~~~~~~~~~~~
* Design smart regenerative braking module using RL algorithm
* Reflect driver characteristics in braking situation

Update
~~~~~~~~~~~~~
* [18/05/31] - Initial release - kyunghan
"""
#%% 0. Import python lib modules
import matplotlib.pyplot as plt
import numpy as np
import os
import sys
# Set initial path
base_dir = os.path.abspath('..')
data_dir = os.path.abspath('..\data_roadxy')
conf_dir = os.path.abspath('..\data_config')
test_dir = os.path.abspath('..\sim_test')
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
from data_roadxy import get_roadxy
#%%
class IDM_brake:
    def __init__(self,mod_param):
        self.t_cst = mod_param[0]
        self.t_init = mod_param[1]
        self.acc_slope_init = mod_param[2]
        self.acc_ref_adj = mod_param[3]    
        self.t_step = 0
        self.acc_set_old = 0
        self.flag_adj = 0
        
    def set_acc(self,acc_ref):    
        if self.t_step <= self.t_cst:
            acc_set = 0
            stIdm = 0
        elif (self.t_step <= self.t_init + self.t_cst) and (self.flag_adj == 0):
            acc_set = self.acc_set_old - self.acc_slope_init
            if acc_set <= self.acc_ref_adj:
                self.flag_adj = 1
            stIdm = 1
        elif self.flag_adj == 1:
            acc_set = acc_ref        
            stIdm = 2
        self.acc_set_old = acc_set    
        self.t_step = self.t_step + 1
        return acc_set, stIdm
#%%    
class RegQ:
    def __init__(self,):
        pass
    def control(self):
        pass
    def policy_update(self):
        # Calculation MC or TD target
        reward_error
        # Calculation model error
        state_error
        update_value_policy = 
        self.policy_table()
        pass
    
    def policy_iteration(self):
        pass
    
    def policy_table(self):
        pass
    
    def env_policy(self,state):
        # e-greedy policy
        action = self.policy_table(state)
        return action

class env:
    def __init__(self):
        pass
    
        
    def env_mkv_prcess(self):
        state = veh_state
        action = self.control(state,self.pol)
        reward = self.env_reward(state,action)
    
    def env_reward(self):
        pass
#%% 1. Import model
# Powertrain import and configuration
kona_power = Mod_PowerTrain()
# ~~~~~
# Bodymodel import and configuration
kona_body = Mod_Body()
kona_body.swtRegCtl = 1
# ~~~~
# Vehicle set
kona_vehicle = Mod_Veh(kona_power, kona_body)
# ~~~~
# Driver model import
drv_kyunghan = Mod_Driver()   
drv_kyunghan.I_gain_lon = 10
drv_kyunghan.P_gain_lon = 5 
# ~~~~
# Behavior set
beh_driving = Mod_Behavior(drv_kyunghan)
beh_driving.conf_forecast_dis = 100
beh_driving.conf_cruise_speed_set = 20
# ~~~~
# Road data import - AMSA cycle simulation
get_roadxy.set_dir(data_dir)
RoadData = get_roadxy.load_mat('road_data_straight.mat')
road_x = np.float64(np.reshape(RoadData['sn_X'],max(np.shape(RoadData['sn_X']))))
road_y = np.float64(np.reshape(RoadData['sn_Y'],max(np.shape(RoadData['sn_X']))))
# ~~~~
# Environment setup
env_st = Mod_Env(road_x,road_y)
# Add traffic light
env_st.Obj_add('Tl', 'red', 1000)
# Add traffic light
road_env_obj = env_st.object_list
road_env_road_len = env_st.road_len
# Set initial vehicle state at environment
env_st.Vehicle_init_config(kona_vehicle, 2)
pos_x = kona_vehicle.pos_x_veh
pos_y = kona_vehicle.pos_y_veh
pos_s = kona_vehicle.pos_s_veh
psi_veh = kona_vehicle.psi_veh
veh_vel = kona_body.vel_veh
# Sim param set
#%% 2. Simulation config
Ts = 0.01
sim_time = 200
sim_time_range = np.arange(0,sim_time,0.01)
# Logging data
data_log_list = ['veh_vel','vel_set','acc_in','brk_in','pos_x','pos_y','pos_s','soc',
                 'trq_mot_set','trq_mot','trq_mot_reg','trq_brk','drv_state',
                 'acc_ref','acc_set','rel_dis','acc','brk_state']

sim_data = type_DataLog(data_log_list)

veh_vel_old = 0
acc = 0
t_cst = 30
t_init = 300
acc_slope = 0.01
acc_init = -2
mod_param = [t_cst, t_init, acc_slope, acc_init]
idm_brake = IDM_brake(mod_param)
for sim_step in range(len(sim_time_range)):
    
    # Behavior control
    stStatic = beh_driving.Static_state_recog(road_env_obj, pos_s, road_env_road_len)
    stDynamic = type_drvstate()
    # Rule base acc controller    
    if stStatic.state == 'Tl_stop':
        veh_vel_set = 0
        rel_dis = stStatic.state_reldis
        acc_ref = sorted((-10, -veh_vel**2/rel_dis, 0))[1]        
        [acc_set, stIdm] = idm_brake.set_acc(acc_ref)
    else:
        veh_vel_set = 15        
        acc_set = 1 * (veh_vel_set - veh_vel)
        acc_ref = acc_set
        rel_dis = 0
        stIdm = -1
    
    [u_acc_in, u_brk_in] = beh_driving.Lon_control(acc_set, acc)
    u_steer_in = beh_driving.Lat_behavior(pos_x,pos_y,psi_veh,road_x,road_y)    
    # Vehicle model sim
    [veh_vel, the_wheel] = kona_vehicle.Veh_driven(u_acc = u_acc_in, u_brake = u_brk_in, u_steer = u_steer_in)
    [pos_x, pos_y, pos_s, pos_n, psi_veh] = kona_vehicle.Veh_position_update(veh_vel, the_wheel)
    
    acc = (veh_vel - veh_vel_old)/0.01
    veh_vel_old = veh_vel
    # Reward and state calculation     
    reward, state  = env.get_state(veh_state, action) ''' Include driver model '''
                   = env.get_reward
    # Model based reward prediction - markov process              
    
    '''
    * Model: Markov decision process
    * Action: Torque control
    * Reward: Regeneration efficiency, Driver heterogenity - calculated from driver mode
    '''
    mod_state, mod_reward = mod.model_based_idm(veh_state)
    # Action selection using predicted reward and state
    action = mod.policy(mod_state, mode_reward)
    
# Learning - Monte Carlo method                   
mod.mc_policy_evaluation(mod_state,mod_reward)
mod.policy_improvement(mod_state,mod_reward)
    # Data store
    data_set = [veh_vel, veh_vel_set, u_acc_in, u_brk_in, pos_x, pos_y, pos_s,
                kona_power.ModMotor.Battery.SOC, kona_body.t_mot_set, kona_power.ModMotor.t_mot, 
                kona_body.t_mot_reg, kona_body.t_brake, beh_driving.stStatic.state, 
                acc_ref, acc_set, rel_dis, acc, stIdm]
    sim_data.StoreData(data_set)
    
for name_var in data_log_list:
    globals()['sim_'+name_var] = sim_data.get_profile_value_one(name_var)    
#%%
fig = plt.figure(figsize=(6,5))
ax1 = plt.subplot(411)
ax2 = plt.subplot(412)
ax3 = plt.subplot(413)
ax4 = plt.subplot(414)
ax1.plot(sim_time_range, sim_veh_vel)
ax1.plot(sim_time_range, sim_vel_set)
ax2.plot(sim_time_range, sim_acc_in)
ax2.plot(sim_time_range, sim_brk_in)
ax3.plot(sim_time_range, sim_acc_set)
ax3.plot(sim_time_range, sim_acc_ref)
ax3.plot(sim_time_range, sim_acc)
ax4.plot(sim_time_range, sim_rel_dis)

fig = plt.figure()
plt.plot(sim_time_range, sim_acc_set)
plt.plot(sim_time_range, sim_acc_ref)
plt.plot(sim_time_range, sim_acc)
plt.plot(sim_time_range, sim_brk_state)

fig = plt.figure()
plt.plot(sim_time_range, sim_brk_in)
plt.plot(sim_time_range, sim_soc)
#%%    
#tmp_len = []
#for i in range(len(road_env_obj)):
#    tmp_obj = road_env_obj[i]
#    print(tmp_obj.object_class)
#
## Simplified driver model for learning test
#def IDM_BrakeModel(driver_char,vel,dis):
#    acc_ref = 
#    # 
#    acc_ref
