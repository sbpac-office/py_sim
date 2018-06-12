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
    from model_powertrain import Mod_PowerTrain
    from model_vehicle import Mod_Veh, Mod_Body
    from model_maneuver import Mod_Behavior, Mod_Driver
    from model_environment import Mod_Env
    from sub_type_def import type_DataLog
    #%% 1. Import models
     # Powertrain import and configuration
    kona_power = Mod_PowerTrain()
    # ~~~~~
    # Bodymodel import and configuration
    kona_body = Mod_Body()
    kona_body.conf_veh_len = 1
    # ~~~~
    # Vehicle set
    kona_vehicle = Mod_Veh(kona_power, kona_body)
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

    # ----------------------------- select input set ---------------------------
    Input_index = 0
    if Input_index == 0:
        # Driver = normal
        drv_kyunghan.set_char('Normal')
        beh_steer.Drver_set(drv_kyunghan)
    elif Input_index == 1:
        # Driver = aggressive
        drv_kyunghan.set_char('Aggressive')
        beh_steer.Drver_set(drv_kyunghan)
    elif Input_index == 2:
        # Driver = defensive
        drv_kyunghan.set_char('Defensive')
        beh_steer.Drver_set(drv_kyunghan)
    elif Input_index == 3:
        # Driver = new driver with config parameter
        drv_kyuhwan = Mod_Driver()
        drv_kyuhwan.P_gain_lon = 0.2
        drv_kyuhwan.I_gain_lon = 0.05
        drv_kyuhwan.I_gain_lat = 0.0001
        drv_kyuhwan.P_gain_lat = 0.001
        drv_kyuhwan.P_gain_yaw = 0.1
        drv_kyuhwan.I_gain_yaw = 0.1
        beh_steer.Drver_set(drv_kyuhwan)
    else:
        print('입력을 똑바로 하세요 ~_~')
    #%% 3. Run simulation
    # Set logging data
    sim4 = type_DataLog(['Veh_Vel','Acc_in','Brk_in','Veh_Psi','Steer_in','Wheel_theta','LaneOff','LaneOffDir'])
    sim4_veh = type_DataLog(['PosX','PosY','VehAn','RoadAn','Andiff'])
    sim4_str = type_DataLog(['steer_an','steer_off'])
    tmp_state = []
    for sim_step in range(len(sim_time_range)):
        # Arrange vehicle position
        pos_x = kona_vehicle.pos_x_veh
        pos_y = kona_vehicle.pos_y_veh
        psi_veh = kona_vehicle.psi_veh
        veh_vel = kona_body.vel_veh
        # Arrange behavior input
        u_speed_set = u_speed_val[sim_step]
        # Behavior control
        [u_acc_in, u_brk_in] = beh_steer.Lon_control(u_speed_set, veh_vel)
        u_steer_in = beh_steer.Lat_behavior(pos_x,pos_y,psi_veh,road_x,road_y)
        # Vehicle model sim
        [veh_vel, the_wheel] = kona_vehicle.Veh_driven(u_acc = u_acc_in, u_brake = u_brk_in, u_steer = u_steer_in)
        [pos_x, pos_y, pos_s, pos_n, psi_veh] = kona_vehicle.Veh_position_update(veh_vel, the_wheel)
        # Store data
        sim4.StoreData([veh_vel, u_acc_in, u_brk_in, kona_vehicle.psi_veh, u_steer_in, the_wheel, beh_steer.lane_offset, beh_steer.stLateral.state])
        sim4_veh.StoreData([pos_x, pos_y, beh_steer.state_veh_an, beh_steer.road_an, beh_steer.psi_offset])
        sim4_str.StoreData([beh_steer.u_steer_yaw, beh_steer.u_steer_offset])
        
        beh_steer.Lon_behavior(env_sl.object_list, pos_s, env_sl.road_len, veh_vel)
        tmp_state.append(beh_steer.veh_speed_set)

    [sim4_veh_vel, sim4_u_acc, sim4_u_brk, sim4_veh_psi, sim4_steer, sim4_wheel, sim4_laneoff, sim4_laneoff_dir] = sim4.get_profile_value(['Veh_Vel','Acc_in','Brk_in','Veh_Psi','Steer_in','Wheel_theta','LaneOff','LaneOffDir'])
    [sim4_veh_x, sim4_veh_y, sim4_veh_an, sim4_road_an, sim4_an_diff] = sim4_veh.get_profile_value(['PosX','PosY','VehAn','RoadAn','Andiff'])
    [sim4_str_an, sim4_str_off] = sim4_str.get_profile_value(['steer_an','steer_off'])
    #%% 4. Result plot
    fig = plt.figure(figsize=(8,4))
    ax1 = plt.subplot(121)
    ax2 = plt.subplot(222)
    ax3 = plt.subplot(224)
    ax1.plot(road_x,road_y,label = 'Road')
    ax1.plot(sim4_veh_x, sim4_veh_y,'--',label = 'Vehicle');ax1.set_xlabel('X position [m]');ax1.set_ylabel('Y position [m]')
    ax1.legend()
    ax2.plot(sim_time_range, sim4_laneoff,label = 'Lane off set')
    ax2.legend()
    ax3.plot(sim_time_range, sim4_steer,label = 'Str')
    ax3.legend()
