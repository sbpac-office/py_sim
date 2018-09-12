# -*- coding: utf-8 -*-
"""
Test code: position control test
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
"""
if __name__ == "__main__":
    #%% 0. Import python lib modules
    import matplotlib.pyplot as plt
    import matplotlib.colors as mcolors
    import numpy as np
    import os
    import sys
    import time
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
    from model_powertrain import Mod_Power
    from model_vehicle import Mod_Veh, Mod_Body    
    from model_maneuver import Mod_Behavior, Mod_Driver
    from model_environment import Mod_Env
    from sub_type_def import type_DataLog
    from data_roadxy import get_roadxy
    #%% 0. Import model
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
    drv_kyunghan.I_gain_lat = 0.001
    drv_kyunghan.P_gain_lat = 0.002
    
    drv_kyunghan.I_gain_yaw = 0.3
    drv_kyunghan.P_gain_yaw = 0.5
    # ~~~~
    # Behavior model
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
#    env_st.Obj_add('Tl','red',495)
    road_env_obj = env_st.object_list
    road_env_road_len = env_st.road_len
    
    beh_driving.conf_filtnum_spdset = 3
    #%% 2. Simulation config
    Ts = 0.01
    sim_time = 668
    sim_time_range = np.arange(0,sim_time,0.01)
    # Set logging data        
    simdata_veh = type_DataLog(['veh_vel','vel_set','acc_in','brk_in','str_in','pos_x','pos_y','pos_s'])
    simdata_body = type_DataLog(['t_mot','t_wheel_load','t_drag','t_brk'])    
    envdata = type_DataLog(['stc_objec','lat_off','head_off'])    
    # initial vehicle position
    pos_x = kona_vehicle.pos_x_veh
    pos_y = kona_vehicle.pos_y_veh
    pos_s = kona_vehicle.pos_s_veh
    psi_veh = kona_vehicle.psi_veh
    veh_vel = kona_vehicle.vel_veh
    start_time = time.time()
    for sim_step in range(len(sim_time_range)):
        
        # Arrange vehicle position
       
        # Behavior control
        [u_acc_in, u_brk_in] = beh_driving.Lon_behavior(road_env_obj, pos_s, road_env_road_len, veh_vel)
        u_steer_in = beh_driving.Lat_behavior(pos_x,pos_y,psi_veh,road_x,road_y)
        # Vehicle model sim
        [veh_vel, the_wheel] = kona_vehicle.Veh_driven(u_acc_in, u_brk_in, u_steer_in)
        [pos_x, pos_y, pos_s, pos_n, psi_veh] = kona_vehicle.Veh_position_update(veh_vel, the_wheel)
        # Driving state
        stStatic = beh_driving.stStatic.state
        lat_off = beh_driving.lane_offset
        head_off = beh_driving.psi_offset
        # Store data
        log_data_set = [veh_vel, beh_driving.veh_speed_set, u_acc_in, u_brk_in, u_steer_in, pos_x, pos_y, pos_s]
        simdata_veh.StoreData(log_data_set)        
        simdata_body.StoreData([kona_vehicle.t_mot_des, kona_vehicle.ModDrive.t_wheel_load, kona_vehicle.t_drag, kona_vehicle.t_brake])
        envdata.StoreData([stStatic,lat_off,head_off])
#        simdata_ctl.StoreData(log_data_ctl_set)
    end_time = time.time()
    for name_var in simdata_veh.NameSet:
        globals()['sim_'+name_var] = simdata_veh.get_profile_value_one(name_var)        
        
    for name_var in simdata_body.NameSet:
        globals()['sim_'+name_var] = simdata_body.get_profile_value_one(name_var)   
    for name_var in envdata.NameSet:
        globals()['sim_'+name_var] = envdata.get_profile_value_one(name_var)      
    #%%    
    colors = dict(mcolors.BASE_COLORS, **mcolors.CSS4_COLORS)
    
    fig = plt.figure(figsize=(3.5,2.5))
    fig_name = 'case3_amsa_road.png'
    plt.grid(color = colors['silver'],linestyle = '--',alpha = 0.7)
    plt.plot(road_x,road_y)
    plt.axis('equal')
    plt.title('AMSA road data')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.subplots_adjust(top=0.8, bottom=0.2, left=0.2, right=0.95, hspace=0.3,
                    wspace=0.35)
    plt.show()
    plt.savefig(fig_name, format='png', dpi=500) 
    
    #%%
    road_curv = env_st.road_curve
    road_len = env_st.road_len
    fig = plt.figure(figsize=(11,6))
    fig_name = 'case3_amsa_result.png'
    ax1 = plt.subplot(231);ax1.grid(color = colors['silver'],linestyle = '--',alpha = 0.7)
    ax2 = plt.subplot(234);ax2.grid(color = colors['silver'],linestyle = '--',alpha = 0.7)
    ax3 = plt.subplot(232);ax3.grid(color = colors['silver'],linestyle = '--',alpha = 0.7)
    ax4 = plt.subplot(235);ax4.grid(color = colors['silver'],linestyle = '--',alpha = 0.7)
    ax5 = plt.subplot(233);ax5.grid(color = colors['silver'],linestyle = '--',alpha = 0.7)
    ax6 = plt.subplot(236);ax6.grid(color = colors['silver'],linestyle = '--',alpha = 0.7)
    
    ax1.plot(road_x,road_y, lw = 3, color = 'skyblue', label = 'Road')
    ax1.plot(sim_pos_x, sim_pos_y,'--',label = 'Vehicle');
    ax1.set_xlabel('X position [m]');ax1.set_ylabel('Y position [m]')
    ax1.legend()
    ax1.axis('equal')
    
    ax2.plot(road_len, road_curv, color = colors['darkorchid'])
    ax2.set_xlabel('Road length [m]')
    ax2.set_ylabel('Curvature [1/m]')
    
    ax3.plot(sim_time_range, sim_vel_set*3.6,label = 'Speed set',lw = 3, color = colors['gold'])
    ax3.plot(sim_time_range, sim_veh_vel*3.6,label = 'Speed', color = colors['navy'])
    ax3.set_ylabel('Speed [km/h]')
    ax3.legend()
    ax4.plot(sim_time_range, sim_acc_in,label = 'Acc', color = colors['c'])
    ax4.plot(sim_time_range, sim_brk_in,label = 'Brk', color = colors['royalblue'])    
    ax4.set_ylabel('Pedal position [-]')
#    ax4.plot(sim_time_range, sim_head_off)
    ax4.set_xlabel('Time [s]')
    ax4.legend()
    
    ax5.plot(sim_time_range, sim_head_off,label = 'Lateral off-set', color = colors['crimson'])    
    ax5.set_ylabel('Heading error [rad]')    
    
    ax6.plot(sim_time_range, sim_str_in,label = 'Str', color = colors['orange'])    
    ax6.set_ylabel('Steering [-]')
    ax6.set_xlabel('Time [s]')
    
    plt.subplots_adjust(top=0.9, bottom=0.1, left=0.1, right=0.95, hspace=0.3,
                    wspace=0.4)
    plt.show()
    plt.savefig(fig_name, format='png', dpi=500) 


