# -*- coding: utf-8 -*-
"""
Test code: steering test
==============================================================

Author
~~~~~~~~~~~~~
* kyunghan <kyunghah.min@gmail.com>

Test scenario
~~~~~~~~~~~~~
* Set steering angle

Test result
~~~~~~~~~~~~~
* Vehicle position
* Vehicle longitudinal velocity
* Input values

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
    from model_powertrain import Mod_PowerTrain
    from model_maneuver import Mod_Behavior, Mod_Driver
    from model_environment import Mod_Env
    from sub_type_def import type_DataLog
    #%% 1. Import models
    # Powertrain import and configuration
    kona_power = Mod_PowerTrain()
    # ~~~~~
    # Bodymodel import and configuration
    kona_body = Mod_Body()
    # ~~~~
    # Vehicle set
    kona_vehicle = Mod_Veh(kona_power, kona_body)
    #%% 2. Simulation config
    Ts = 0.01
    sim_time = 40
    sim_time_range = np.arange(0,sim_time,0.01)
    # ----------------------------- select input set ---------------------------
    Input_index = 0
    if Input_index == 0:
        # Turn right
        u_acc_val = np.concatenate((0 * np.ones(int(len(sim_time_range)*0.1)), 0.3 * np.ones(int(len(sim_time_range)*0.9))))
        u_brk_val = 0 * np.ones(len(sim_time_range))
        u_steer_val = np.concatenate((0 * np.ones(int(len(sim_time_range)*0.3)), 0.01 * np.ones(int(len(sim_time_range)*0.7))))
    elif Input_index == 1:
        # Sin input
        u_acc_val = np.concatenate((0 * np.ones(int(len(sim_time_range)*0.1)), 0.3 * np.ones(int(len(sim_time_range)*0.9))))
        u_brk_val = 0 * np.ones(len(sim_time_range))
        u_steer_val = np.concatenate((0 * np.ones(int(len(sim_time_range)*0.3)), 0.01*np.sin(0.01*np.arange(int(len(sim_time_range)*0.7)))))
    else:
        print('입력을 똑바로 하세요 ~_~')
    #%% 3. Run simulation
    # Set logging data
    sim2 = type_DataLog(['Veh_Vel','Pos_X','Pos_Y','Acc_Set','Str_Set'])
    for sim_step in range(len(sim_time_range)):
        # Arrange vehicle input
        u_acc_in = u_acc_val[sim_step]
        u_brk_in = u_brk_val[sim_step]
        u_str_in = u_steer_val[sim_step]
        # Vehicle model sim
        [veh_vel, the_wheel] = kona_vehicle.Veh_driven(u_acc = u_acc_in, u_brake = u_brk_in, u_steer = u_str_in)
        [pos_x, pos_y, pos_s, pos_n, psi_veh] = kona_vehicle.Veh_position_update(veh_vel, the_wheel)
        # Store data
        sim2.StoreData([veh_vel, pos_x, pos_y, u_acc_in, u_str_in])

    [sim2_veh_vel, sim2_pos_x, sim2_pos_y, sim2_u_acc, sim2_u_str] = sim2.get_profile_value(['Veh_Vel','Pos_X','Pos_Y','Acc_Set','Str_Set'])
    #%% 4. Reulst plot
    fig = plt.figure(figsize=(8,4))
    ax1 = plt.subplot(121)
    ax2 = plt.subplot(222)
    ax3 = plt.subplot(224)
    ax1.plot(sim2_pos_x, sim2_pos_y);ax1.set_xlabel('X position [m]');ax1.set_ylabel('Y position [m]');ax1.axis('equal')
    ax2.plot(sim_time_range, sim2_veh_vel)
    ax3.plot(sim_time_range, sim2_u_acc,label = 'Acc')
    ax3.plot(sim_time_range, sim2_u_str,label = 'Str')
