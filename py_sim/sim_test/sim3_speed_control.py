# -*- coding: utf-8 -*-
"""
Test code: speed control test
==============================================================

Author
~~~~~~~~~~~~~
* kyunghan <kyunghah.min@gmail.com>

Test scenario
~~~~~~~~~~~~~
* Set vehicle speed with NEDC
* Set driver characteristics

Test result
~~~~~~~~~~~~~
* Vehicle longitudinal velocity
* Driver input values

Update
~~~~~~~~~~~~~
* [18/05/31] - Initial release - kyunghan
* [18/08/09] - Modification release - kyunghan
"""
if __name__== "__main__":
    #%% 0. Import python lib modules
    import matplotlib.pyplot as plt
    import matplotlib.colors as mcolors
    import numpy as np
    import os
    import sys
    import scipy.io as io
    import time
    # Set initial path
    base_dir = os.path.abspath('..')
    env_dir = os.path.abspath('..\data_roadxy')
    conf_dir = os.path.abspath('..\data_config')
    data_dir = os.path.abspath('..\data_vehmodel')
    test_dir = os.path.abspath('..\sim_test')
    print('Base directory: ', base_dir)
    sys.path.append(base_dir);
    sys.path.append(data_dir);
    sys.path.append(conf_dir);
    sys.path.append(test_dir);
    sys.path.append(env_dir)
    # Import package lib modules
    from model_vehicle import Mod_Veh, Mod_Body
    from model_powertrain import Mod_Power
    from model_maneuver import Mod_Behavior, Mod_Driver
    from model_environment import Mod_Env
    from sub_type_def import type_DataLog
    from math import pi
    os.chdir(data_dir)
    data_driver = io.loadmat('data_driver.mat')    
    data_vehicle = io.loadmat('data_vehicle.mat')
    data_motor = io.loadmat('data_emachine.mat')    
    os.chdir(test_dir)
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
    beh_cycle = Mod_Behavior(drv_kyunghan)
    # ~~~~
    #%% 2. Simulation config
    #  2. brake torque
    u_veh_vel_set = data_driver['var_Desired_Velocity_km_h_']/3.6    
    Ts = 0.01    
    sim_time_range = data_driver['var_Time_s_']      
    #%% 3. Simulation
    # Set logging data
    beh_cycle.conf_filtnum_pedal = 0.5
    sim_driver = type_DataLog(['u_acc','u_brk'])
    sim_vehicle = type_DataLog(['Veh_vel','t_mot','t_brk','w_mot'])
    veh_vel = 0
    w_mot = 0
    w_shaft = 0
    w_wheel = 0
    start_time = time.time()
    for sim_step in range(len(sim_time_range)):
        # Arrange cruise input
        u_veh_vel_set_in = u_veh_vel_set[sim_step]        
        u_acc_in, u_brk_in = beh_cycle.Lon_control(u_veh_vel_set_in,veh_vel)
        # Vehicle model sim
        #   1. Cockfit system
        t_mot = kona_vehicle.Acc_system(u_acc_in)
        t_brk, t_reg = kona_vehicle.Brake_system(u_brk_in)
        #   1. Drag force        
        t_drag, f_drag = kona_vehicle.Drag_system(veh_vel)
        #   2. Torque equivalence
        t_mot_load, t_shaft_in, t_shaft_out, t_wheel_in, t_wheel_traction_f, t_driven, f_lon = kona_vehicle.ModDrive.Lon_equivalence(t_mot,t_brk,t_drag)
        #   3. Dynamics of component
        w_mot = kona_vehicle.ModDrive.Motor_dynamics(t_mot, t_mot_load, w_mot)
        w_shaft = kona_vehicle.ModDrive.Driveshaft_dynamics(t_shaft_in, t_shaft_out, w_shaft)
        w_wheel = kona_vehicle.ModDrive.Tire_dynamics(t_wheel_in, t_wheel_traction_f, t_brk/4, w_wheel)
        #   4. Vehicle driven
        veh_vel, veh_acc = kona_vehicle.Veh_lon_dynamics(f_lon, f_drag, veh_vel)
#        veh_vel = kona_vehicle.Veh_lon_driven(u_acc_in, u_brk_in)
        
        sim_driver.StoreData([u_acc_in,u_brk_in])
        sim_vehicle.StoreData([veh_vel,t_mot,t_brk,w_mot])
    end_time = time.time()
    print('Exc time: ',end_time - start_time)
    for name_var in sim_driver.NameSet:
        globals()['sim_'+name_var] = sim_driver.get_profile_value_one(name_var)
    for name_var in sim_vehicle.NameSet:
        globals()['sim_'+name_var] = sim_vehicle.get_profile_value_one(name_var)
    #%% 4. Reulst plot
    colors = dict(mcolors.BASE_COLORS, **mcolors.CSS4_COLORS)
    
    fig = plt.figure(figsize=(5,5))
    fig_name = 'case1_nedc_simdb.png'
    ax1 = plt.subplot(211);ax1.grid(color = colors['silver'],linestyle = '--',alpha = 0.7)
    ax2 = plt.subplot(212);ax2.grid(color = colors['silver'],linestyle = '--',alpha = 0.7)    
    ax1.set_title('NEDC driving with speed control')
    ax1.plot(sim_time_range, u_veh_vel_set*3.6, lw = 3, color = colors['skyblue'],label = 'desired speed')
    ax1.plot(sim_time_range, sim_Veh_vel*3.6, lw = 1, color = colors['darkblue'], label = 'vehicle speed')
    ax1.set_ylabel('Speed [km/h]')
    ax1.legend()
    ax2.plot(sim_time_range, sim_u_acc, color = colors['forestgreen'], label = 'Acceleration')
    ax2.plot(sim_time_range, sim_u_brk, color = colors['royalblue'], label = 'Brake')
    ax2.set_ylabel('Pedal position [-]')
    ax2.set_xlabel('Time [s]')
    ax2.legend()
    plt.show()
    plt.savefig(fig_name, format='png', dpi=500) 
    #%%
    t_mot_cruise = data_motor['var_Torque_Nm_']
    w_mot_cruise = data_motor['var_Rotor_Angular_Velocity_1_min_']*2*pi/60
    fig = plt.figure(figsize=(5,5))
    fig_name = 'case1_nedc_cruise.png'
    ax1 = plt.subplot(211);ax1.grid(color = colors['silver'],linestyle = '--',alpha = 0.7)
    ax2 = plt.subplot(212);ax2.grid(color = colors['silver'],linestyle = '--',alpha = 0.7)    
    ax1.set_title('NEDC driving with speed control')
    ax1.plot(sim_time_range, t_mot_cruise, lw = 3, color = colors['gold'],label = 'Cruise')
    ax1.plot(sim_time_range, sim_t_mot, color = colors['forestgreen'], label = 'py_sim')
    ax1.set_ylabel('Motor Torque [Nm]')
    ax1.legend()
    ax2.plot(sim_time_range, w_mot_cruise, lw = 3, color = colors['darkorange'], label = 'Cruise')
    ax2.plot(sim_time_range, sim_w_mot, color = colors['navy'], label = 'py_sim')
    ax2.set_ylabel('Motor rot speed [rad/s]')
    ax2.set_xlabel('Time [s]')
    ax2.legend()
    plt.show()
    plt.savefig(fig_name, format='png', dpi=500) 
    
    
    
     