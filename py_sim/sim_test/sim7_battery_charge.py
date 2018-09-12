# -*- coding: utf-8 -*-
"""
Test code: battery charge simulation
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
* [18/05/31] - Initial release - kyunghan
"""
if __name__== "__main__":
    #%% 0. Import python lib modules
    import matplotlib.pyplot as plt
    import matplotlib.colors as mcolors
    import numpy as np
    import os
    import sys
    import scipy.io as io
    # set initial path

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
    from model_vehicle import Mod_Veh, Mod_Body
    from model_powertrain import Mod_Power
    from model_maneuver import Mod_Behavior, Mod_Driver
    from model_environment import Mod_Env
    from sub_type_def import type_DataLog
    os.chdir(data_dir)
    data_driver = io.loadmat('data_driver.mat')    
    data_vehicle = io.loadmat('data_vehicle.mat')    
    os.chdir(test_dir)    
    #%% 2. Simulation config
#    Ts = 0.01
#    sim_time = 400
#    sim_time_range = np.arange(0,sim_time,0.01)
#
#    # ----------------------------- select input set ---------------------------
#    veh_speed_set = np.concatenate((0 * np.ones(int(len(sim_time_range)*0.1)), 20 * np.ones(int(len(sim_time_range)*0.4)),  0 * np.ones(int(len(sim_time_range)*0.5))))
    u_veh_vel_set = data_driver['var_Desired_Velocity_km_h_']/3.6    
    Ts = 0.01    
    sim_time_range = data_driver['var_Time_s_']   
    #%% 3. Run simulation
    for test_case in range(2):
    
        
        # Set logging data
        
        veh_data = type_DataLog(['Veh_Vel','Pos_X','Pos_Y','Acc_Set','Brk_Set','t_brk'])
        motor_data = type_DataLog(['t_mot','t_mot_reg','t_mot_des','w_mot'])
        power_data = type_DataLog(['p_mot','p_loss','p_elec','soc','v_mot','i_mot'])
        shaft_data = type_DataLog(['w_shaft','w_wheel','w_veh','w_dot_mot','w_dot_veh','w_dot_shaft'])
        
        
        
        # Powertrain import and configuration
        kona_power = Mod_Power()
        # Bodymodel import and configuration
        kona_body = Mod_Body()  
        # Vehicle set
        kona_vehicle = Mod_Veh(kona_power, kona_body)
        drv_kyunghan = Mod_Driver()
        # Behavior model
        beh_speed = Mod_Behavior(drv_kyunghan)
        veh_vel = kona_vehicle.vel_veh
        
        kona_vehicle.swtRegCtl = test_case
        
                
        for sim_step in range(len(sim_time_range)):
            # Arrange vehicle input
            veh_vel_set = u_veh_vel_set[sim_step]
            u_acc_in, u_brk_in =  beh_speed.Lon_control(veh_vel_set, veh_vel)       
            # Vehicle model sim
            [veh_vel, the_wheel] = kona_vehicle.Veh_driven(u_acc = u_acc_in, u_brake = u_brk_in)
            [pos_x, pos_y, pos_s, pos_n, psi_veh] = kona_vehicle.Veh_position_update(veh_vel, the_wheel)
            # Vehicle state
            t_mot = kona_power.ModMotor.t_mot
            t_mot_des = kona_vehicle.t_mot_des
            t_mot_reg = kona_vehicle.t_mot_reg
            soc = kona_power.ModBattery.SOC
            v_mot = kona_power.ModMotor.v_mot
            i_mot = kona_power.ModMotor.i_mot
            w_mot = kona_power.ModMotor.w_mot
            w_shaft = kona_vehicle.ModDrive.w_shaft
            w_wheel = kona_vehicle.ModDrive.w_wheel
            w_veh = kona_vehicle.ModDrive.w_vehicle
            w_dot_shaft = kona_vehicle.ModDrive.w_dot_shaft
            w_dot_wheel = kona_vehicle.ModDrive.w_dot_wheel
            w_dot_mot = kona_vehicle.ModDrive.w_dot_motor
            w_dot_veh = kona_vehicle.ModDrive.w_dot_vehicle
            t_brk = kona_vehicle.t_brake
            
            p_mot = kona_power.ModMotor.p_mot_mech
            p_mot_loss = kona_power.ModMotor.p_mot_loss
            p_mot_elec = kona_power.ModMotor.p_mot_elec
            # Store data
            veh_data.StoreData([veh_vel, pos_x, pos_y, u_acc_in, u_brk_in, t_brk])
            motor_data.StoreData([t_mot, t_mot_reg, t_mot_des, w_mot])
            power_data.StoreData([p_mot, p_mot_loss, p_mot_elec, soc, v_mot, i_mot])
            shaft_data.StoreData([w_shaft,w_wheel,w_veh,w_dot_mot,w_dot_veh,w_dot_shaft])
           
        case_var = 'sim_%d_' % test_case  
        for name_var in veh_data.NameSet:
            globals()[case_var+name_var] = veh_data.get_profile_value_one(name_var)
        for name_var in motor_data.NameSet:
            globals()[case_var+name_var] = motor_data.get_profile_value_one(name_var) 
        for name_var in power_data.NameSet:
            globals()[case_var+name_var] = power_data.get_profile_value_one(name_var) 
        for name_var in shaft_data.NameSet:
            globals()[case_var+name_var] = shaft_data.get_profile_value_one(name_var) 
    #%% 4. Result plot
    t_brk_eq = sim1_t_brk/kona_vehicle.ModDrive.conf_gear
    
    fig = plt.figure(figsize=(8,6))    
    ax1 = fig.add_subplot(321)
    ax2 = fig.add_subplot(322)
    ax3 = fig.add_subplot(323)
    ax4 = fig.add_subplot(324)
    ax5 = fig.add_subplot(325)
    ax6 = fig.add_subplot(326)
    ax1.plot(sim_time_range, sim_0_Veh_Vel, label = 'braking')
    ax1.plot(sim_time_range, sim_1_Veh_Vel, label = 'regeneration')
    ax2.plot(sim_time_range, sim_Acc_Set,label='Acc')
    ax2.plot(sim_time_range, sim_Brk_Set,label='Brk')
    ax3.plot(sim_time_range, sim_soc,label='soc')
    ax4.plot(sim_time_range, sim_t_mot, label = 'mot trq')
    ax4.plot(sim_time_range, sim_t_mot_des, label = 'mot trq des')
    ax4.plot(sim_time_range, sim_t_mot_reg, label = 'mot trq reg')
    ax4.plot(sim_time_range, t_brk_eq, label = 'brk trq_eq')    
    ax4.legend()
    ax5.plot(sim_time_range, sim_w_mot)
    ax6.plot(sim_time_range, sim_i_mot)
    
    
    fig2 = plt.figure()
    plt.plot(stLonCtl)
    plt.plot(stLonCtl_delay)
    
    #%%
    colors = dict(mcolors.BASE_COLORS, **mcolors.CSS4_COLORS)
    fig = plt.figure(figsize=(5,5))
    fig_name = 'case2_vehicle_motor.png'
    ax1 = plt.subplot(311);ax1.grid(color = colors['silver'],linestyle = '--',alpha = 0.7)
    ax2 = plt.subplot(312);ax2.grid(color = colors['silver'],linestyle = '--',alpha = 0.7)    
    ax3 = plt.subplot(313);ax3.grid(color = colors['silver'],linestyle = '--',alpha = 0.7)    
    ax1.set_title('NEDC driving with regeneration')
    ax1.plot(sim_time_range, sim_0_Veh_Vel*3.6, lw = 3, color = colors['gold'],label = 'Mech braking')
    ax1.plot(sim_time_range, sim_1_Veh_Vel*3.6, color = colors['forestgreen'], label = 'Reg braking')
    ax1.set_ylabel('Vehicle speed [km/h]')
    ax1.legend()
    ax2.plot(sim_time_range, sim_0_soc, lw = 3, color = colors['darkorange'], label = 'Mech braking')
    ax2.plot(sim_time_range, sim_1_soc, color = colors['navy'], label = 'Reg braking')    
    ax2.set_ylabel('SOC [%]')    
    ax2.legend()
    ax3.plot(sim_time_range, sim_0_i_mot, lw = 3, color = colors['pink'], label = 'Mech braking')
    ax3.plot(sim_time_range, sim_1_i_mot, color = colors['darkred'], label = 'Reg braking')
    ax3.set_xlabel('Time [s]')    
    ax3.set_ylabel('Motor DC current [A]')    
    ax3.legend()
    plt.show()
    plt.savefig(fig_name, format='png', dpi=500) 
    #%%
    fig = plt.figure(figsize=(5,5))
    fig_name = 'case2_motor_torque.png'
    ax1 = plt.subplot(311);ax1.grid(color = colors['silver'],linestyle = '--',alpha = 0.7)
    ax2 = plt.subplot(312);ax2.grid(color = colors['silver'],linestyle = '--',alpha = 0.7)    
    ax3 = plt.subplot(313);ax3.grid(color = colors['silver'],linestyle = '--',alpha = 0.7)    
    ax1.set_title('NEDC driving with regeneration')
    ax1.plot(sim_time_range, sim_0_t_mot, lw = 3, color = colors['deepskyblue'],label = 'Mech braking')
    ax1.plot(sim_time_range, sim_1_t_mot, color = colors['gold'], label = 'Reg braking')    
    ax1.set_ylabel('Motor torque [Nm]')
    ax1.legend()
    ax2.plot(sim_time_range, sim_0_t_brk, color = colors['deepskyblue'], label = 'Mech braking')
    ax2.plot(sim_time_range, sim_1_t_brk, color = colors['gold'], label = 'Reg braking')    
    ax2.set_ylabel('Brake torque [Nm]')    
    ax2.legend()
    ax3.plot(sim_time_range, sim_1_p_mot/1000, color = colors['darkorchid'], label = 'Mech power')
    ax3.plot(sim_time_range, sim_1_p_elec/1000, ls = '--', color = colors['deepskyblue'], label = 'Elec power')
    ax3.plot(sim_time_range, sim_1_p_loss/1000, color = colors['red'], label = 'Loss power')
    ax3.set_xlabel('Time [s]')    
    ax3.set_ylabel('Power [kW]')    
    ax3.legend()
    plt.show()
    plt.savefig(fig_name, format='png', dpi=500) 
    plt.subplots_adjust(top=0.9, bottom=0.1, left=0.15, right=0.95, hspace=0.3,
                    wspace=0.35)