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
import random
import pickle
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
from sub_utilities import Filt_LowPass
from data_roadxy import get_roadxy
#%%
def int_index(index_set,data_val):
    index_list = np.where(index_set >= data_val)
    if index_list == []:
        index = len(index_set)
    else:
        index = np.min(np.where(index_set >= data_val))
    index_val = index_set[index]
    return index_val, index
#%%
class IDM_brake:
    def __init__(self,mod_param):
        self.t_cst = mod_param[0]
        self.t_init = mod_param[1]
        self.t_term = mod_param[2]
        self.acc_slope_init = mod_param[3]
        self.acc_ref_adj = mod_param[4]    
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
        elif (self.flag_adj == 1) and (self.t_step <= self.t_term):
            acc_set = self.acc_ref_adj                   
            stIdm = 2
        elif self.flag_adj == 1:
            acc_set = acc_ref
            stIdm = 3
        
        self.acc_set_old = acc_set    
        self.t_step = self.t_step + 1
        return acc_set, stIdm
#%% Agent class
class agent:
    def __init__(self, q_init_rat, act_set, st_vel_set, st_dis_set):
        self.q_table = np.ones((len(act_set),len(st_vel_set),len(st_dis_set)))*q_init_rat
        self.act_set = act_set
        self.set_learnconf(conf_dis = 0.9 , conf_learnrate = 0.3, conf_egreedy = 0.5)
        self.act_num = 0
        
    def update_q_table(self, act_list, state_list_vel, state_list_dis, reward_list):
        len_st = len(act_list)
        q_list = np.zeros(len_st)
        q_list[-1] = reward_list[-1]
        self.update_q_value(act_list[-1], state_list_vel[-1], state_list_dis[-1], q_list[-1])
        for step in range(2,len_st+1):
            current_step = len_st - step
            q_list[current_step] = reward_list[current_step] + self.conf_dis * q_list[current_step+1]            
            self.update_q_value(act_list[current_step], state_list_vel[current_step], state_list_dis[current_step], q_list[current_step])
        return q_list
            
    def update_q_value(self, state_act, state_vel, state_dis, q_val):
        old_q = self.q_table[state_act,state_vel,state_dis]
        self.q_table[state_act,state_vel,state_dis] = old_q + self.conf_learnrate * (q_val - old_q)
        return self.q_table[state_act, state_vel, state_dis]

    def set_learnconf(self, conf_dis, conf_learnrate, conf_egreedy):
        self.conf_dis = conf_dis
        self.conf_learnrate = conf_learnrate  
        self.conf_egreedy = conf_egreedy
    
    def e_greedy_policy(self,state_vel, state_dis):
        rand_prob = random.random()
        if rand_prob>=self.conf_egreedy:
            action_index = np.argmax(self.q_table[:,state_vel,state_dis])
            action_val = self.act_set[action_index]
        else:
            action_index = int(random.uniform(0,len(self.act_set)))
            action_val = self.act_set[action_index]           
        return action_val, action_index
#%% Environment class       
class env:
    def __init__(self, st_vel_set, st_dis_set):
        self.r_reg = 0
        self.r_drv = 0
        self.r_saf = 0
        self.soc_old = 0
        self.set_r_coef()
        self.st_vel_set = st_vel_set
        self.st_dis_set = st_dis_set
    
    def set_r_coef(self, rc_reg = 10, rc_drv = 1, 
                   rc_ucd = -5, rc_ucs = 4,  rc_saf_uc = -100, 
                   rc_ocd = 20, rc_ocs = 16, rc_saf_oc = -50):
        self.rc_reg = rc_reg
        self.rc_drv = rc_drv
        self.rc_ucd = rc_ucd
        self.rc_ucs = rc_ucs
        self.rc_saf_uc = rc_saf_uc
        self.rc_ocd = rc_ocd
        self.rc_ocs = rc_ocs
        self.rc_saf_oc = rc_saf_oc    
        
    def get_reward_reg(self, data_soc, data_soc_old):
        self.r_reg = self.rc_reg*(data_soc - data_soc_old)
        return self.r_reg
    
    def get_reward_saf(self, data_vel, data_dis):                
        under_cri_dis = data_vel*self.rc_ucs + self.rc_ucd
        over_cri_dis = data_vel*self.rc_ocs + self.rc_ocd               
        if data_dis <= under_cri_dis:
            self.r_saf = self.rc_saf_uc
        elif data_dis >= over_cri_dis:
            self.r_saf = self.rc_saf_oc
        else:
            self.r_saf = 1
        return self.r_saf
            
    def get_reward_drv(self, data_brk, data_acc):
        self.r_drv = self.rc_drv*(0 - data_brk - data_acc)
        return self.r_drv
        
    def get_reward(self,data_vel,data_dis,data_soc,data_soc_old,data_brk, data_acc):
        r_reg = self.get_reward_reg(data_soc,data_soc_old)
        r_saf = self.get_reward_saf(data_vel,data_dis)
        r_drv = self.get_reward_drv(data_brk, data_acc)
        self.r_sum = r_reg + r_saf + r_drv
        return self.r_sum
    
    def get_state(self, data_vel, data_dis):        
        [st_vel, st_vel_index] = int_index(self.st_vel_set,data_vel)
        [st_dis, st_dis_index] = int_index(self.st_dis_set,data_dis)
        return st_vel, st_vel_index, st_dis, st_dis_index
#%% 1. RL config
act_index = np.arange(-0.1,0.12,0.02)
vel_index = np.arange(0,25,0.5)
dis_index = np.concatenate((np.arange(-0.5,10,0.5), np.arange(10,200,5)))
env_brake = env(vel_index, dis_index)
agent_mc = agent(0, act_index, vel_index, dis_index)

#%% 2. Driver model config
t_cst = 30
t_init = 300
t_term = 700
acc_slope = 0.01
acc_init = -1.5
mod_param = [t_cst, t_init, t_term, acc_slope, acc_init]    
#%% Iterative learning for one case - MC
# Load learning agent
#with open('agent_result.p','rb') as file:
#    agent_mc = pickle.load(file)

agent_mc.conf_learnrate = 0.003
agent_mc.conf_dis = 0.999
env_brake.rc_reg = 1000
ItNumMax = 4000
Learning_result = []
Learning_control = []
Learning_qval = []
swt_valid = 1
for ItNum in range(ItNumMax):
    swt_valid = ItNum%100        
    egreedy_dis_fac = ItNum//100
    if swt_valid == 0:
        agent_mc.conf_egreedy = 0
        print('============================= validation =================================')        
    else:
        agent_mc.conf_egreedy = 0.5 - egreedy_dis_fac/100
    # 3. Import model - set the initial value
    # Powertrain import and configuration
    kona_power = Mod_PowerTrain()
    # ~~~~~
    # Bodymodel import and configuration
    kona_body = Mod_Body()
    kona_body.swtRegCtl = 2
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
    beh_driving.conf_cruise_speed_set = 20
    idm_brake = IDM_brake(mod_param)    
    # ~~~~
    # Road data import - AMSA cycle simulation
#    get_roadxy.set_dir(data_dir)
#    RoadData = get_roadxy.load_mat('road_data_straight.mat')
#    road_x = np.float64(np.reshape(RoadData['sn_X'],max(np.shape(RoadData['sn_X']))))
#    road_y = np.float64(np.reshape(RoadData['sn_Y'],max(np.shape(RoadData['sn_X']))))
#    env_st = Mod_Env(road_x, road_y)
    # ~~~~
    # Environment setup
    # Set traffic location
    sim_env_tl_loc = 1000
    # Set rel_dis to coast
    drv_set_coast_dis = 150
    
    # Set initial vehicle state at environment
#    env_st.Vehicle_init_config(kona_vehicle, 2)
    pos_x = kona_vehicle.pos_x_veh
    pos_y = kona_vehicle.pos_y_veh
    pos_s = kona_vehicle.pos_s_veh
    psi_veh = kona_vehicle.psi_veh
    veh_vel = kona_body.vel_veh
    w_mot = kona_power.ModMotor.w_mot
    the_wheel = kona_body.the_wheel
    soc = kona_power.ModBattery.SOC
    soc_old = soc
    # 2. Simulation config
    Ts = 0.01
    sim_time = 200
    sim_time_range = np.arange(0,sim_time,0.01)
    flag_stop = 0
    # Set initial simulation set    
    veh_vel_old = 0
    acc = 0
    t_mot_reg_set_old = 0
    
    # Logging data
    data_log_list = ['veh_vel','vel_set','acc_in','brk_in','pos_x','pos_y','pos_s','soc',
                     'trq_mot_set','trq_mot','trq_mot_reg','trq_brk','drv_state',
                     'acc_ref','acc_set','rel_dis','acc','brk_state','trq_mot_load','w_wheel',
                     'lon_state']
    sim_data = type_DataLog(data_log_list)
    rl_log_list = ['st_vel','st_vel_index','st_dis','st_dis_index','act','act_index','r_drv','r_reg','r_saf','reward']
    rl_data = type_DataLog(rl_log_list)
    
    for sim_step in range(len(sim_time_range)):
        '''
        Reinforcement learning
        * Agent: Select action as policy and state
            * Action: Regeneration torque set
        * Environment: Calculate reward and state
        * Algorithm
            * 1. S (st_vel, st_dis) define (env)
            * 2. A (act_regen_set) define (agent - e-greedy)
            * 3. Vehicle simulation and data calculation
            * 4. R (reward) define (env) 
            * 5. State update (soc, acc, reg_set)
            * 6. Store current (S,A,R)
        '''   
        rel_dis = sim_env_tl_loc - pos_s
        # Rule base Acc controller with driver model   
        if (rel_dis <= drv_set_coast_dis) and (veh_vel > 0.01) and (pos_s <= 1010) and (flag_stop == 0):
            veh_vel_set = 0        
            acc_ref = sorted((-10, -veh_vel**2/(rel_dis), 0))[1]        
            [acc_set, stIdm] = idm_brake.set_acc(acc_ref)
            u_brk_idm_ff = -acc_set/7
            drv_state = 'Tl'
            # 1. State define
            [st_vel, st_vel_index, st_dis, st_dis_index] = env_brake.get_state(veh_vel, rel_dis)                
            # 2. Action define
            [act_val, act_index] = agent_mc.e_greedy_policy(st_vel_index, st_dis_index)
            t_mot_reg_set = t_mot_reg_set_old + act_val        
        elif ((veh_vel <= 0.01) and (pos_s >= 500)) or (pos_s >= 1000) or (flag_stop == 1):
            drv_state = 'Stop'
            veh_vel_set = 0
            acc_set = 0
            acc_ref = acc_set
            u_brk_idm_ff = 0
            stIdm = -1
            t_mot_reg_set = 0
            flag_stop = 1
        else:
            veh_vel_set = 15        
            acc_set = 1 * (veh_vel_set - veh_vel)
            acc_ref = acc_set
            u_brk_idm_ff = 0
            stIdm = -1
            t_mot_reg_set = 0
            drv_state = 'Cruise'
        # 3. Vehicle simulation
        # Driver control inputs
        [u_acc, u_brk] = beh_driving.Lon_control(acc_set, acc)        
        u_steer_in = 0
#        u_brk = 0
        # Longitudinal drive
        [t_brk, t_mot_reg] = kona_body.Brake_system(u_brk, t_mot_reg_set)
        t_mot_des = kona_body.Acc_system(u_acc)
        t_mot_set = t_mot_des + t_mot_reg    
        [w_wheel, t_load, veh_vel] = kona_body.Lon_driven_out(t_brk, w_mot)
        [w_mot, t_mot] = kona_power.ModMotor.Motor_control(t_mot_set, t_load)
        
        # Sim vehicle driven
    #    [veh_vel, the_wheel] = kona_vehicle.Veh_driven(u_acc_filt, u_brk_filt, u_steer_in)
    #    t_mot_des = kona_body.t_mot_des
    #    t_mot_set = kona_body.t_mot_set
    #    t_mot_reg = kona_body.t_mot_reg
    #    w_mot = kona_power.ModMotor.w_mot
    #    t_mot = kona_power.ModMotor.t_mot
    #    t_load = kona_power.ModMotor.t_load
        
        # Vehicle position update    
        [pos_x, pos_y, pos_s, pos_n, psi_veh] = kona_vehicle.Veh_position_update(veh_vel, the_wheel)
        soc = kona_power.ModBattery.SOC
        
        if drv_state == 'Tl':
            # 4. Reward define
            reward = env_brake.get_reward(veh_vel, rel_dis, soc, soc_old, u_brk, u_acc)
            r_drv = env_brake.r_drv
            r_reg = env_brake.r_reg
            r_saf = env_brake.r_saf
            rl_log_data = [st_vel,st_vel_index,st_dis,st_dis_index,act_val,act_index,r_drv,r_reg,r_saf,reward]
            rl_data.StoreData(rl_log_data)        
            
        # 5. State update
        acc = (veh_vel - veh_vel_old)/0.01
        veh_vel_old = veh_vel
        soc_old = soc
        t_mot_reg_set_old = t_mot_reg_set
        
        
    
        data_set = [veh_vel, veh_vel_set, u_acc, u_brk, pos_x, pos_y, pos_s,
                    kona_power.ModMotor.Battery.SOC, t_mot_set, kona_power.ModMotor.t_mot, 
                    t_mot_reg, kona_body.t_brake, drv_state, 
                    acc_ref, acc_set, rel_dis, acc, stIdm, t_load, w_wheel, beh_driving.stLonControl]
        sim_data.StoreData(data_set)
        
    for name_var in data_log_list:
        globals()['sim_'+name_var] = sim_data.get_profile_value_one(name_var) 
    for name_var in rl_log_list:
        globals()['rl_'+name_var] = rl_data.get_profile_value_one(name_var)        
    
    q_list = agent_mc.update_q_table(rl_act_index, rl_st_vel_index, rl_st_dis_index, rl_reward)
    
    Learning_score = np.mean(np.array(rl_reward))
    print('*====== LS:',Learning_score,' , IN:', ItNum,' ====================*')
    Learning_result.append(Learning_score)    
    Learning_control.append(np.array(sim_trq_mot_reg))
    Learning_qval.append(q_list)
plt.figure()
plt.plot(rl_reward)    
plt.plot(Learning_qval[-1])    
#%%
with open('agent_result.p','wb') as file:
    pickle.dump(agent_mc, file)
        
fig = plt.figure(figsize=(6,5))
ax1 = plt.subplot(421)
ax2 = plt.subplot(422)
ax3 = plt.subplot(423)
ax4 = plt.subplot(424)
ax5 = plt.subplot(425)
ax6 = plt.subplot(426)
ax7 = plt.subplot(427)
ax8 = plt.subplot(428)
ax1.plot(sim_time_range, sim_veh_vel)
ax1.plot(sim_time_range, sim_vel_set)
ax2.plot(sim_time_range, sim_acc_in)
ax2.plot(sim_time_range, sim_brk_in)
ax3.plot(sim_time_range, sim_acc_set)
ax3.plot(sim_time_range, sim_acc_ref)
ax3.plot(sim_time_range, sim_acc)
ax4.plot(sim_time_range, sim_rel_dis)
ax5.plot(sim_time_range, sim_trq_mot_reg, label = 't_reg')
ax5.plot(sim_time_range, sim_trq_mot, label = 't_mot')
ax5.plot(sim_time_range, sim_trq_mot_set, label = 't_mot_set')
ax5.plot(sim_time_range, sim_trq_mot_load, label = 't_mot_load')
#ax5.legend()
ax7.plot(sim_time_range, sim_soc)
ax6.plot(rl_reward)
ax8.plot(rl_r_drv,label='drv')
ax8.plot(rl_r_reg,label='reg')
ax8.plot(rl_r_saf,label='saf')
#ax8.legend()
##%%
#fig = plt.figure()
#plt.plot(sim_time_range, sim_acc_set)
#plt.plot(sim_time_range, sim_acc_ref)
#plt.plot(sim_time_range, sim_acc)
#plt.plot(sim_time_range, sim_brk_state)
#
#fig = plt.figure()
#plt.plot(sim_time_range, sim_brk_in)
#plt.plot(sim_time_range, sim_soc)
#%%

under_cri_rel_dis = -5
under_cri_slope = 4
under_cri_line = np.array(rl_st_vel)*under_cri_slope + under_cri_rel_dis

over_cri_rel_dis = 20
over_cri_slope = 16
over_cri_slope = np.array(rl_st_vel)*over_cri_slope + over_cri_rel_dis

fig = plt.figure()
plt.plot(rl_st_vel,rl_st_dis,'.')
plt.plot(rl_st_vel,under_cri_line)
plt.plot(rl_st_vel,over_cri_slope)

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
