# -*- coding: utf-8 -*-
"""
Test code: speed control test
==============================================================

Author
~~~~~~~~~~~~~
* kyunghan <kyunghah.min@gmail.com>

Test scenario
~~~~~~~~~~~~~
* Set vehicle speed
* Set driver characteristics

Test result
~~~~~~~~~~~~~
* Vehicle position
* Vehicle longitudinal velocity
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
    # ~~~~
    # Driver model
    drv_kyunghan = Mod_Driver()
    # ~~~~
    # Behavior model
    beh_cruise = Mod_Behavior(drv_kyunghan)
    # ~~~~

    #%% 2. Simulation config
    Ts = 0.01
    sim_time = 70
    sim_time_range = np.arange(0,sim_time,0.01)
    veh_vel = 0 # Initial vehicle speed

    # ----------------------------- select input set ---------------------------
    Input_index = 0
    if Input_index == 0:
        # Driver = normal
        drv_kyunghan.set_char('Normal')
        beh_cruise.Drver_set(drv_kyunghan)
    elif Input_index == 1:
        # Driver = aggressive
        drv_kyunghan.set_char('Aggressive')
        beh_cruise.Drver_set(drv_kyunghan)
    elif Input_index == 2:
        # Driver = defensive
        drv_kyunghan.set_char('Defensive')
        beh_cruise.Drver_set(drv_kyunghan)
    elif Input_index == 3:
        # Driver = new driver with config parameter
        drv_kyuhwan = Mod_Driver()
        drv_kyuhwan.P_gain_lon = 1.2
        beh_cruise.Drver_set(drv_kyuhwan)
    else:
        print('입력을 똑바로 하세요 ~_~')

    u_speed_val = np.concatenate((0 * np.ones(int(len(sim_time_range)*0.1)), 30 * np.ones(int(len(sim_time_range)*0.9))))

    #%% 3. Simulation
    # Set logging data
    sim3 = type_DataLog(['Veh_Vel','Acc_in','Brk_in','Trq_set'])

    for sim_step in range(len(sim_time_range)):
        # Arrange cruise input
        u_speed_set = u_speed_val[sim_step]
        # Behavior control
        [u_acc_in, u_brk_in] = beh_cruise.Lon_control(u_speed_set, veh_vel)
        # Vehicle model sim
        [veh_vel, the_wheel] = kona_vehicle.Veh_driven(u_acc = u_acc_in, u_brake = u_brk_in)
        # Store data
        sim3.StoreData([veh_vel, u_acc_in, u_brk_in, beh_cruise.trq_set_lon])

    [sim3_veh_vel, sim3_u_acc, sim3_u_brk, sim3_trq_set] = sim3.get_profile_value(['Veh_Vel','Acc_in','Brk_in','Trq_set'])

    #%% 4. Reulst plot
    fig = plt.figure(figsize=(6,6))
    ax1 = plt.subplot(311)
    ax2 = plt.subplot(312)
    ax3 = plt.subplot(313)
    ax1.plot(sim_time_range, u_speed_val)
    ax1.plot(sim_time_range, sim3_veh_vel)
    ax2.plot(sim_time_range, sim3_u_acc)
    ax2.plot(sim_time_range, sim3_u_brk)
    ax3.plot(sim_time_range, sim3_trq_set)
