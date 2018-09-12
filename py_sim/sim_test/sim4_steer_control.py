# -*- coding: utf-8 -*-
"""
Test code: position control test
==============================================================

Author
~~~~~~~~~~~~~
* kyunghan <kyunghah.min@gmail.com>

Test scenario
~~~~~~~~~~~~~
* Import road data
* Set driver characteristics
* Set vehicle velocity

Test result
~~~~~~~~~~~~~
* Vehicle position
* Road position
* Vehicle longitudinal velocity
* Steering angle
* Driver input values

Update
~~~~~~~~~~~~~
* [18/05/31] - Initial release - kyunghan
"""
if __name__== "__main__":
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
    from model_powertrain import Mod_Power
    from model_maneuver import Mod_Behavior, Mod_Driver
    from model_environment import Mod_Env
    from sub_type_def import type_DataLog
    from sub_type_def import type_DataLog
    from data_roadxy import get_roadxy
    #%% 1. Import models
    # Powertrain import and configuration
    kona_power = Mod_Power()
    # ~~~~~
    # Bodymodel import and configuration
    kona_drivetrain = Mod_Body()
    # ~~~~
    # Vehicle set
    kona_vehicle = Mod_Veh(kona_power, kona_drivetrain)
    # ~~~~
    # Driver model
    drv_kyunghan = Mod_Driver()
    # ~~~~
    # Behavior model
    beh_steer = Mod_Behavior(drv_kyunghan)
    # Road model
    road_x = np.arange(1,4000,1)
    road_sin = 100 + 30*np.sin(np.arange(0, len(road_x)*0.8*0.003, 0.003))
    road_y = np.concatenate((100*np.ones(int(len(road_x)*0.2)), road_sin))
    # Environment set
    env_sl = Mod_Env(road_x,road_y)
    # Set initial vehicle position
    env_sl.Vehicle_init_config(kona_vehicle, 2)
    #%% 2. Simulation config
    Ts = 0.01
    sim_time = 150
    sim_time_range = np.arange(0,sim_time,0.01)
    u_speed_val = np.concatenate((0 * np.ones(int(len(sim_time_range)*0.1)), 20 * np.ones(int(len(sim_time_range)*0.9))))
    #%% 3. Run simulation
    # Set logging data
    veh_data_sim = type_DataLog(['vel_veh','the_wheel','u_acc','u_brk','u_str'])    
    veh_state_sim = type_DataLog(['pos_x','pos_y','psi_veh'])
    # Vehicle initial state
    vel_veh = kona_vehicle.vel_veh
    pos_x = kona_vehicle.pos_x_veh
    pos_y = kona_vehicle.pos_y_veh
    psi_veh = kona_vehicle.psi_veh
    
    for sim_step in range(len(sim_time_range)):
        # Arrange behavior input       
        u_vel_veh_set_in = u_speed_val[sim_step]
        # Behavior control
        u_acc_in, u_brk_in = beh_steer.Lon_control(u_vel_veh_set_in,vel_veh)
        u_steer_in = beh_steer.Lat_behavior(pos_x,pos_y,psi_veh,road_x,road_y)
        # Vehicle driven
        vel_veh,the_wheel = kona_vehicle.Veh_driven(u_acc_in,u_brk_in,u_steer_in)
        # Vehicle position update
        pos_x, pos_y, pos_s, pos_n, psi_veh = kona_vehicle.Veh_position_update(vel_veh,the_wheel)
        # Data logging        
        veh_data_sim.StoreData([vel_veh,the_wheel,u_acc_in,u_brk_in,u_steer_in])
        veh_state_sim.StoreData([pos_x,pos_y,psi_veh])
    
    for name_var in veh_state_sim.NameSet:
        globals()['sim_'+name_var] = veh_state_sim.get_profile_value_one(name_var)
    for name_var in veh_data_sim.NameSet:
        globals()['sim_'+name_var] = veh_data_sim.get_profile_value_one(name_var)    
    
    #%% 4. Result plot
    
    fig = plt.figure(figsize=(8,4))
    ax1 = plt.subplot(121)
    ax2 = plt.subplot(222)
    ax3 = plt.subplot(224)    
    ax1.plot(road_x,road_y,label = 'Road')
    ax1.plot(sim_pos_x, sim_pos_y,'--',label = 'Vehicle');ax1.set_xlabel('X position [m]');ax1.set_ylabel('Y position [m]')
    ax1.legend()
#    ax2.plot(sim_time_range, sim4_laneoff,label = 'Lane off set')
    ax2.legend()
    ax3.plot(sim_time_range, sim_u_str,label = 'Str')
    ax3.legend()
    
    fig = plt.figure()
    ax1 = plt.subplot(311)
    ax2 = plt.subplot(312)
    ax3 = plt.subplot(313)  
    ax1.plot(sim_time_range, sim_vel_veh)
#    ax1.plot(sim_time_range, u_speed_val)
    ax1.plot(sim_time_range, u_speed_val)
    
