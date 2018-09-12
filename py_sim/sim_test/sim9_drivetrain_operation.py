# -*- coding: utf-8 -*-
"""
Test code: motor torque in test
==============================================================

Author
~~~~~~~~~~~~~
* kyunghan <kyunghah.min@gmail.com>

Test scenario
~~~~~~~~~~~~~
* Set motor and brake torque

Test result
~~~~~~~~~~~~~
* Vehicle longitudinal velocity
* Drive train dynamcis

Update
~~~~~~~~~~~~~
* [18/08/09] - Initial release - kyunghan
"""
if __name__== "__main__":
    #%% 0. Import python lib modules
    import matplotlib.pyplot as plt
    from matplotlib import colors as mcolors
    import numpy as np
    import os
    import sys
    import scipy.io as io
    from math import pi
    # set initial path
    base_dir = os.path.abspath('..')
    env_dir = os.path.abspath('..\data_roadxy')
    conf_dir = os.path.abspath('..\data_config')
    data_dir = os.path.abspath('..\data_vehmodel')
    test_dir = os.path.abspath('..\sim_test')
    
    print('Base directory: ', base_dir)
    sys.path.append(base_dir);
    sys.path.append(env_dir);
    sys.path.append(data_dir);
    sys.path.append(conf_dir);
    sys.path.append(test_dir);
    from model_vehicle import Mod_Veh, Mod_Drive
    from model_powertrain import Mod_Power
    from model_maneuver import Mod_Behavior, Mod_Driver
    from model_environment import Mod_Env
    from sub_type_def import type_DataLog
    #%% 1. Import vehicle data
    os.chdir(data_dir)
    data_emachine = io.loadmat('data_emachine.mat')
    data_wheel_f = io.loadmat('data_wheel_fl.mat')
    data_wheel_r = io.loadmat('data_wheel_rl.mat')
    data_vehicle = io.loadmat('data_vehicle.mat')
    data_brake_f = io.loadmat('data_brake_fl.mat')
    data_brake_r = io.loadmat('data_brake_rl.mat')
    data_shaftout = io.loadmat('data_differential.mat')
    data_sim_result = io.loadmat('data_sim_result.mat')
    os.chdir(test_dir)
    #%% 2. Import models
    # Vehicle model import and configuration
    kona_power = Mod_Power()
    kona_drive = Mod_Drive()
    
    kona_vehicle = Mod_Veh(kona_power, kona_drive)
    #%% 3. Simulation config
    # ----------------------------- select input set ---------------------------
    # input for drive-train
    #  1. motor torque
    u_t_mot = data_emachine['var_Torque_Nm_']
    #  2. brake torque
    u_t_brk = -(data_brake_f['var_Braking_Torque_Nm_'] + data_brake_r['var_Braking_Torque_Nm_'])*2        
    Ts = 0.01    
    sim_time_range = data_emachine['var_Time_s_']    
    #%% 3. Run simulation
    # Set logging data
    data_sim_torque = type_DataLog(['t_mot_load','t_wheel_load','t_traction','t_drag'])
    data_sim_dynamics = type_DataLog(['w_mot','w_shaft','w_wheel'])
    data_sim_veh = type_DataLog(['veh_acc','veh_vel','f_lon','f_drag'])
    veh_vel = 0;
    w_mot = 0;
    w_shaft = 0;
    w_wheel = 0;
    
    for sim_step in range(len(sim_time_range)):
        # Arrange vehicle input
        u_t_mot_in = u_t_mot[sim_step]
        u_t_brk_in = u_t_brk[sim_step]        
        # Vehicle model sim
        #   1. Drag force        
        t_drag, f_drag = kona_vehicle.Drag_system(veh_vel)
        #   2. Torque equivalence
        t_mot_load, t_shaft_in, t_shaft_out, t_wheel_in, t_wheel_traction_f, t_driven, f_lon = kona_vehicle.ModDrive.Lon_equivalence(u_t_mot_in,u_t_brk_in,t_drag)
        #   3. Dynamics of component
        w_mot = kona_vehicle.ModDrive.Motor_dynamics(u_t_mot_in, t_mot_load, w_mot)
        w_shaft = kona_vehicle.ModDrive.Driveshaft_dynamics(t_shaft_in, t_shaft_out, w_shaft)
        w_wheel = kona_vehicle.ModDrive.Tire_dynamics(t_wheel_in, t_wheel_traction_f, u_t_brk_in/4, w_wheel)
        #    3. Vehicle driven
        veh_vel, veh_acc = kona_vehicle.Veh_lon_dynamics(f_lon, f_drag, veh_vel)
        # Store data
        data_sim_torque.StoreData([t_mot_load,t_shaft_out,t_wheel_traction_f,t_drag])
        data_sim_dynamics.StoreData([w_mot,w_shaft,w_wheel])
        data_sim_veh.StoreData([veh_acc,veh_vel,f_lon,f_drag])       
        
    veh_data = data_sim_veh.get_profile_value()
    torque_data = data_sim_torque.get_profile_value()
    w_data = data_sim_dynamics.get_profile_value()
    #%% 4. Result plot
    colors = dict(mcolors.BASE_COLORS, **mcolors.CSS4_COLORS)
    
    veh_acc_cruise = data_vehicle['var_Acceleration_m_s_2_']
    veh_vel_cruise = data_vehicle['var_Velocity_km_h_']
    veh_acc_sim = veh_data[0]
    veh_vel_sim = veh_data[1]*3.6
    
    fig_vehicle = plt.figure(figsize = (8,7))
    ax1 = fig_vehicle.add_subplot(4,1,1)
    ax1.grid(color = colors['silver'],linestyle = '--',alpha = 0.7)
    ax1.plot(sim_time_range,veh_acc_cruise,color = colors['gold'], lw = 3, label = 'cruise')
    ax1.plot(sim_time_range,veh_acc_sim, ls = '--', color = colors['r'], lw = 1, label = 'py_sim')
    ax1.set_ylabel('Acceleration [$m/s^2$]')
#    ax1.set_xlabel('Time [$s$]')
    ax1.set_title('Vehicle simulation results')
    ax1.legend()
    ax1 = fig_vehicle.add_subplot(4,1,2)
    ax1.grid(color = colors['silver'],linestyle = '--',alpha = 0.7)    
    ax1.plot(sim_time_range,veh_acc_cruise - veh_acc_sim, color = colors['r'], lw = 1, label = 'error')
    ax1.set_ylabel('Acceleration error [$m/s^2$]')
#    ax1.set_xlabel('Time [$s$]')    
    ax2 = fig_vehicle.add_subplot(4,1,3)
    ax2.grid(color = colors['silver'],linestyle = '--',alpha = 0.7)
    ax2.plot(sim_time_range,veh_vel_cruise,color = colors['gold'], lw = 3, label = 'cruise')
    ax2.plot(sim_time_range,veh_vel_sim, ls = '--', color = colors['darkblue'], lw = 1, label = 'py_sim')
    ax2.set_ylabel('Velocity [$km/h$]')
#    ax2.set_xlabel('Time [$s$]')
    ax2.legend()
    ax2 = fig_vehicle.add_subplot(4,1,4)
    ax2.grid(color = colors['silver'],linestyle = '--',alpha = 0.7)
    ax2.plot(sim_time_range,veh_vel_cruise - veh_vel_sim, color = colors['darkblue'], lw = 1, label = 'error')    
    ax2.set_ylabel('Velocity error [$km/h$]')
    ax2.set_xlabel('Time [$s$]')
    plt.subplots_adjust(top=0.95, bottom=0.1, left=0.15, right=0.90, hspace=0.5,
                    wspace=0.35)
    
    torque_mot_input = u_t_mot
    torque_brk_input = u_t_brk

    torque_wheel_load = torque_data[1]
    torque_shaft_load = torque_data[0]
    
    rotacc_shaft = w_data[1]*60/2/pi
    rotacc_shaft_cruise = data_shaftout['var_Speed_Output_1_1_min_']
    
    force_traction = veh_data[2]
    force_resistance = veh_data[3]
    
    fig_drivetrain = plt.figure(figsize = (8,7))
    ax1 = fig_drivetrain.add_subplot(4,1,1)
    ax1.grid(color = colors['silver'],linestyle = '--',alpha = 0.7)
    ax1.plot(sim_time_range,torque_mot_input, color = colors['darkblue'], lw = 1, label = 'mot_in')
    ax1.plot(sim_time_range,torque_brk_input, color = colors['green'], lw = 1, label = 'brk_in')
    ax1.set_ylabel('Torque [$Nm$]')    
    ax1.legend()
    
    ax1 = fig_drivetrain.add_subplot(4,1,2)
    ax1.grid(color = colors['silver'],linestyle = '--',alpha = 0.7)    
    ax1.plot(sim_time_range,torque_wheel_load, color = colors['indigo'], lw = 1, label = 'wheel_load')
    ax1.plot(sim_time_range,torque_shaft_load, color = colors['c'], lw = 1, label = 'shaft_load')
    ax1.set_ylabel('Torque [$Nm$]')    
    ax1.legend()
    
    ax2 = fig_drivetrain.add_subplot(4,1,3)
    ax2.grid(color = colors['silver'],linestyle = '--',alpha = 0.7)
    ax2.plot(sim_time_range,force_traction, color = colors['olive'], lw = 1, label = 'traction')
    ax2.plot(sim_time_range,force_resistance, color = colors['coral'], lw = 1, label = 'resistance')
    ax2.set_ylabel('Force [$N$]')
#    ax2.set_xlabel('Time [$s$]')
    ax2.legend()
    ax2 = fig_drivetrain.add_subplot(4,1,4)
    ax2.grid(color = colors['silver'],linestyle = '--',alpha = 0.7)
    ax2.plot(sim_time_range,rotacc_shaft_cruise,color = colors['skyblue'], lw = 3, label = 'cruise')
    ax2.plot(sim_time_range,rotacc_shaft, color = colors['r'], lw = 1, label = 'py_sim')    
    ax2.set_ylabel('Rot vel [$rpm$]')
    ax2.set_xlabel('Time [$s$]')
    ax2.legend()
    plt.subplots_adjust(top=0.95, bottom=0.1, left=0.15, right=0.90, hspace=0.5,
                    wspace=0.35)
    
