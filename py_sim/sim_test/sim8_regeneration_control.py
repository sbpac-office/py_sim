# -*- coding: utf-8 -*-
"""
Created on Tue Jun  5 16:18:32 2018

@author: Kyunghan
"""

# -*- coding: utf-8 -*-
"""
Test code: regeneration control test
==============================================================

Author
~~~~~~~~~~~~~
* kyunghan <kyunghah.min@gmail.com>

Test scenario
~~~~~~~~~~~~~
* Set acc and brake position

Test result
~~~~~~~~~~~~~
* Vehicle longitudinal velocity
* Input values

Update
~~~~~~~~~~~~~
* [18/06/11] - Initial release - kyunghan
"""
if __name__== "__main__":
    #%% 0. Import python lib modules
    import matplotlib.pyplot as plt
    import numpy as np
    import os
    import sys
    # set initial path
    base_dir = os.path.abspath('..')
    data_dir = os.path.abspath('..\data_roadxy')
    conf_dir = os.path.abspath('..\data_config')
    test_dir = os.path.abspath('..\sim_test')

    print('Base directory: ', base_dir)
    sys.path.append(base_dir);
    sys.path.append(data_dir);
    sys.path.append(conf_dir);
    sys.path.append(test_dir);
    from model_vehicle import Mod_Veh, Mod_Body
    from model_powertrain import Mod_PowerTrain
    from model_maneuver import Mod_Behavior, Mod_Driver
    from model_environment import Mod_Env
    from sub_type_def import type_DataLog
    from sub_utilities import Get_SqrVal    
    #%% 1. Import models
    # Powertrain import and configuration
    kona_power = Mod_PowerTrain()    
    kona_power_reg = Mod_PowerTrain()    
    # ~~~~~
    # Bodymodel import and configuration
    kona_body = Mod_Body()    
    kona_body_reg = Mod_Body()    
    kona_body_reg.swtRegCtl = 1
    # ~~~~
    # Vehicle set    
    kona_vehicle = Mod_Veh(kona_power, kona_body)    
    kona_vehicle_reg = Mod_Veh(kona_power_reg, kona_body_reg)    
    #%% 2. Simulation config
    Ts = 0.01
    sim_time = 100
    sim_time_range = np.arange(0,sim_time,0.01)
    # Initial state
    w_mot = kona_power.ModMotor.w_mot
    t_mot_load = kona_power.ModMotor.t_load
    # ----------------------------- select input set ---------------------------        
    u_acc_val = Get_SqrVal([0.3, 0.1, 0], [0.1,  0.3, 0.5], sim_time/Ts)
    u_brk_val = Get_SqrVal([0.3, 0], [0.55, 0.8], sim_time/Ts)
    #%% 3. Run simulation
    # Set logging data
    data_log_list = ['veh_vel','w_wheel','w_mot','u_acc','u_brk','SOC','t_mot_set','t_mot','v_mot','t_brk','t_drag','t_load']
    sim = type_DataLog(data_log_list)
    sim_reg = type_DataLog(data_log_list)
    for sim_step in range(len(sim_time_range)):
        # Arrange vehicle input
        u_acc_in = u_acc_val[sim_step]
        u_brk_in = u_brk_val[sim_step]
        # Vehicle model sim
        kona_vehicle.Veh_driven(u_acc_in, u_brk_in)        
        kona_vehicle_reg.Veh_driven(u_acc_in, u_brk_in)                                 
        # Module driven sim
#        [t_mot_des, t_brake] = kona_body.Lon_driven_in(u_acc_in, u_brk_in)
#        [w_mot, t_mot] = kona_power.ModMotor.Motor_control(t_mot_des, t_mot_load)
#        [w_wheel, t_mot_load, vel_veh] = kona_body.Lon_driven_out(t_brake, w_mot)
        
        # Store data
        mot = kona_power.ModMotor; body = kona_body
        sim.StoreData([body.vel_veh,     body.w_wheel,     mot.w_mot, 
                       u_acc_in,         u_brk_in,         kona_power.ModBattery.SOC, 
                       body.t_mot_set,   mot.t_mot,        mot.v_mot,
                       body.t_brake,     body.t_drag,      mot.t_load])
        mot = kona_power_reg.ModMotor; body = kona_body_reg
        sim_reg.StoreData([body.vel_veh,     body.w_wheel,     mot.w_mot, 
                           u_acc_in,         u_brk_in,         kona_power_reg.ModBattery.SOC, 
                           body.t_mot_set,   mot.t_mot,        mot.v_mot,
                           body.t_brake,     body.t_drag,      mot.t_load])
            
        
    for name_var in data_log_list:
        globals()['sim_'+name_var] = sim.get_profile_value_one(name_var)
        globals()['sim_reg_'+name_var] = sim_reg.get_profile_value_one(name_var)
    #%% 4. Result plot
    fig = plt.figure(figsize=(8,4))
    ax1 = plt.subplot(221)
    ax2 = plt.subplot(222)
    ax3 = plt.subplot(223)
    ax4 = plt.subplot(224)
    ax1.plot(sim_time_range, sim_veh_vel, label = 'Brk')
    ax1.plot(sim_time_range, sim_reg_veh_vel, '--',label='reg')
    ax1.set_ylabel('Vehicle_speed')
    ax1.legend()
    ax3.plot(sim_time_range, sim_SOC, label = 'Brk')    
    ax3.plot(sim_time_range, sim_reg_SOC,'--',label='reg')    
    ax3.set_ylabel('SOC')
    ax3.legend()
    ax2.plot(sim_time_range, np.array(sim_t_brk)/kona_body.conf_rd_gear, label = 't_brk')
    ax2.plot(sim_time_range, sim_t_mot_set, label = 't_mot_set')
    ax2.plot(sim_time_range, sim_reg_t_mot_set, label = 't_mot_set_reg')
    ax2.plot(sim_time_range, sim_t_mot, label = 't_mot')        
    ax2.plot(sim_time_range, sim_reg_t_mot, label = 't_mot_reg')        
    ax2.legend()
    ax2.set_ylabel('Torque')
    ax4.plot(sim_time_range, sim_u_acc,label='Acc')
    ax4.plot(sim_time_range, sim_u_brk,label='Brk')    
    ax4.legend()
