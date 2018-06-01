# -*- coding: utf-8 -*-
"""
Test code: Battery charge
==============================================================

Author
~~~~~~~~~~~~~
* kyunghan <kyunghah.min@gmail.com>

Test scenario
~~~~~~~~~~~~~
* Import real environment
* Set driver characteristics

Test result
~~~~~~~~~~~~~
* Vehicle position
* Road position
* Driver behavior
* Vehicle longitudinal behavior
* Vehicle lateral behavior
* Driver input values

Update
~~~~~~~~~~~~~
* [18/05/31] - Initial release - kyunghan
* [18/06/01] - rev - kyuhwan
"""
if __name__ == "__main__":
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
    from sub_type_def import type_DataLog
    from data_roadxy import get_roadxy
    #%% 0. Import model
    # Powertrain import and configuration
    kona_power = Mod_PowerTrain()
    # ~~~~~
    # Bodymodel import and configuration
    kona_body = Mod_Body()
    # ~~~~
    # Vehicle set
    kona_vehicle = Mod_Veh(kona_power, kona_body)
    # ~~~~
    # Driver model import
    drv_kyunghan = Mod_Driver()
    # ~~~~
    # Behavior set
    beh_driving = Mod_Behavior(drv_kyunghan)
    # ~~~~
    # Road data import - AMSA cycle simulation
    get_roadxy.set_dir(data_dir)
    RoadData = get_roadxy.load_mat('road_data_amsa.mat')
    road_x = np.reshape(RoadData['sn_X'],max(np.shape(RoadData['sn_X'])))
    road_y = np.reshape(RoadData['sn_Y'],max(np.shape(RoadData['sn_X'])))
    plt.figure()
    plt.plot(road_x,road_y)
    plt.axis('equal')
    plt.title('Road data')
    # Environment set
    env_st = Mod_Env(road_x,road_y)
    # Set initial vehicle state at environment
    env_st.Vehicle_init_config(kona_vehicle, 2)
    road_env_obj = env_st.object_list
    road_env_road_len = env_st.road_len
    #%% 2. Simulation config
    Ts = 0.01
    sim_time = 600
    sim_time_range = np.arange(0,sim_time,0.01)
    # Set logging data
    data_log_list = ['veh_vel','vel_set','acc_in','brk_in','str_in','pos_x','pos_y','pos_s','soc']
    simdata = type_DataLog(data_log_list)
    for sim_step in range(len(sim_time_range)):
        # Arrange vehicle position
        pos_x = kona_vehicle.pos_x_veh
        pos_y = kona_vehicle.pos_y_veh
        pos_s = kona_vehicle.pos_s_veh
        psi_veh = kona_vehicle.psi_veh
        veh_vel = kona_body.vel_veh
        # Behavior control
        [u_acc_in, u_brk_in] = beh_driving.Lon_behavior(road_env_obj, pos_s, road_env_road_len, veh_vel)
        u_steer_in = beh_driving.Lat_behavior(pos_x,pos_y,psi_veh,road_x,road_y)
        # Vehicle model sim
        [veh_vel, the_wheel] = kona_vehicle.Veh_driven(u_acc = u_acc_in, u_brake = u_brk_in, u_steer = u_steer_in)
        [pos_x, pos_y, pos_s, pos_n, psi_veh] = kona_vehicle.Veh_position_update(veh_vel, the_wheel)
        soc = kona_power.Battery.SOC
        # Store data
        log_data_set = [veh_vel, beh_driving.veh_speed_set, u_acc_in, u_brk_in, u_steer_in, pos_x, pos_y, pos_s,soc]
        simdata.StoreData(log_data_set)

    for name_var in data_log_list:
        globals()['sim_'+name_var] = simdata.get_profile_value_one(name_var)
    #%%
    fig = plt.figure(figsize=(8,4))
    ax1 = plt.subplot(121)
    ax2 = plt.subplot(322)
    ax3 = plt.subplot(324)
    ax4 = plt.subplot(326)
    ax1.plot(road_x,road_y,label = 'Road')
    ax1.plot(sim_pos_x, sim_pos_y,'--',label = 'Vehicle');ax1.set_xlabel('X position [m]');ax1.set_ylabel('Y position [m]')
    ax1.legend()
    ax2.plot(sim_time_range, sim_vel_set,label = 'Velocity set')
    ax2.plot(sim_time_range, sim_veh_vel,label = 'Velocity')
    ax2.legend()
    ax3.plot(sim_time_range, sim_acc_in,label = 'Acc')
    ax3.plot(sim_time_range, sim_brk_in,label = 'Brk')
    ax3.plot(sim_time_range, sim_str_in,label = 'Str')
    ax3.legend()
    ax4.plot(sim_time_range, sim_soc,label = 'SOC')
    ax4.legend()
    
    
