# -*- coding: utf-8 -*-
"""
Simulation model : Environment
==============================================================

Author
~~~~~~~~~~~~~
* kyunghan <kyunghah.min@gmail.com>

Description
~~~~~~~~~~~~~
* Environment model

Update
~~~~~~~~~~~~~
* [18/05/31] - Initial release - kyunghan
"""
# import python lib modules
from math import pi, sin, cos, atan
import numpy as np
from scipy.spatial import distance as dist_calc
# import package modules
from sub_utilities import Calc_Radius, Filt_MovAvg, Calc_PrDis
from sub_type_def import type_pid_controller, type_drvstate, type_objective
# import config data modules

# simulation sampling time
Ts = 0.01
"""global vairable: simulation sampling timeself.

you can declare other sampling time in application as vairable ``Ts``

"""
class Mod_Env:
    """
    Module description here

    ConfigVariables:
        * conf_rw_wheel
        * conf_jw_body
        * conf_brk_coef
        * conf_acc_coef
        * conf_veh_len
        * ...

    Submodules:
        * Body_config:
        * Dyn_config:
        * Lon_driven_in:
        * ...

    Operation:
        Description operation here::

            !!!Make operation diagram here!!!
            # Module_name(in//out)
            Motor_control(t_mot, w_mot // des_torque)
                >> Motor_driven(t_mot, w_mot // v_mot)
                    >> Motor_elec_dynamics, Motor_mech_dynamics, Drive_shaft_dynamics
                >> Motor_torque_system(v_mot // des_torque)

    """
    def __init__(self, road_array_x_in, road_array_y_in, start_road_len = 0):
        self.road_x = road_array_x_in
        self.road_y = road_array_y_in
        self.object_list = [type_objective() for _ in range(len(road_array_x_in))]
        self.Road_config(start_road_len)

    def Road_config(self, start_road_len = 0):
        """Function overview here

        Functional description

        Code example wirght follows::

            >>> [w_mot, t_mot, t_load] = Motor_control(t_mot_des)
            ...

        Args:
            * Input parameters here
            * t_mot_des:
            * w_shaft:
            * ...

        returns:
            * Return of function here
            * w_mot: motor rotational speed [rad/s]
            * t_load: load torque from body model [Nm]
        """
        road_array_x_in = self.road_x
        road_array_y_in = self.road_y
        loc_env_road_s = np.zeros(len(road_array_x_in))
        loc_env_road_s[0] = start_road_len
        loc_env_road_ang = np.zeros(len(road_array_x_in))
        loc_env_road_ang[0] = 0
        for i in range(1,len(road_array_x_in),1):
            old_pos = [road_array_x_in[i-1],road_array_y_in[i-1]]
            new_pos = [road_array_x_in[i],road_array_y_in[i]]
            loc_env_road_s[i] = loc_env_road_s[i-1] + dist_calc.euclidean(old_pos, new_pos)
            loc_env_road_ang[i] = np.arctan((road_array_y_in[i] - road_array_y_in[i-1])/(road_array_x_in[i] - road_array_x_in[i-1]))
        self.road_ang = loc_env_road_ang
        self.road_len = loc_env_road_s
        self.object_list = self.Road_curve_def(road_array_x_in, road_array_y_in, loc_env_road_s)

    def Obj_add (self, object_in, object_param_in, object_s_location):
        """Function overview here

        Functional description

        Code example wirght follows::

            >>> [w_mot, t_mot, t_load] = Motor_control(t_mot_des)
            ...

        Args:
            * Input parameters here
            * t_mot_des:
            * w_shaft:
            * ...

        returns:
            * Return of function here
            * w_mot: motor rotational speed [rad/s]
            * t_load: load torque from body model [Nm]
        """
        loc_env_road_s = self.road_len
        tmp_s_index = np.min(np.where(loc_env_road_s >= object_s_location)) - 1
        self.object_list[tmp_s_index].add_object(object_in,object_param_in,object_s_location)

    def Road_curve_def(self, road_array_x_in, road_array_y_in, loc_env_road_s, conf_curve_val = 0.001):
        """Function overview here

        Functional description

        Code example wirght follows::

            >>> [w_mot, t_mot, t_load] = Motor_control(t_mot_des)
            ...

        Args:
            * Input parameters here
            * t_mot_des:
            * w_shaft:
            * ...

        returns:
            * Return of function here
            * w_mot: motor rotational speed [rad/s]
            * t_load: load torque from body model [Nm]
        """
        object_list = [type_objective() for _ in range(len(road_array_x_in))]
        [R_out, x_c_out, y_c_out, circle_index, mr_o, mt_o] = Calc_Radius(road_array_x_in, road_array_y_in, 3)
        tmp_Curve = 1/R_out
        tmp_Curve_Filt = Filt_MovAvg(tmp_Curve,3)
        tmp_Curve_index = np.arange(len(road_array_x_in))[tmp_Curve_Filt >= conf_curve_val]
        self.road_curve = tmp_Curve
        for i in range(len(tmp_Curve_index)):
            tmp_s_index = tmp_Curve_index[i]
            object_list[tmp_s_index].add_object('Curve',tmp_Curve[tmp_s_index],loc_env_road_s[tmp_s_index])
        return object_list

    def Vehicle_init_config(self, veh_mod, road_index = 0):
        """Function overview here

        Functional description

        Code example wirght follows::

            >>> [w_mot, t_mot, t_load] = Motor_control(t_mot_des)
            ...

        Args:
            * Input parameters here
            * t_mot_des:
            * w_shaft:
            * ...

        returns:
            * Return of function here
            * w_mot: motor rotational speed [rad/s]
            * t_load: load torque from body model [Nm]
        """
        veh_mod.pos_x_veh = self.road_x[road_index]
        veh_mod.pos_y_veh = self.road_y[road_index]
        veh_mod.psi_veh = atan((self.road_y[road_index+1] - self.road_y[road_index])/(self.road_x[road_index+1] - self.road_x[road_index])) + (1 - (self.road_x[road_index+1] - self.road_x[road_index])/abs((self.road_x[road_index+1] - self.road_x[road_index])))/2*pi

#%%  ----- test ground -----
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import numpy as np
    import scipy.io as io
    import os
    import sys
    # Set initial path
    base_dir = os.path.abspath('.')
    data_dir = os.path.abspath('.\data_roadxy')
    conf_dir = os.path.abspath('.\data_config')
    test_dir = os.path.abspath('.\sim_test')
    print('Base directory: ', base_dir)
    sys.path.append(base_dir);
    sys.path.append(data_dir);
    sys.path.append(conf_dir);
    sys.path.append(test_dir);
    
    from model_powertrain import Mod_Power
    from model_vehicle import Mod_Veh, Mod_Body    
    from model_maneuver import Mod_Behavior, Mod_Driver    
    from sub_type_def import type_DataLog
    from data_roadxy import get_roadxy
    from sub_utilities import Filt_LowPass
    #%% Import model
    drv_kyunghan = Mod_Driver()
    # Behavior model
    beh_driving = Mod_Behavior(drv_kyunghan)
    
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
    road_env_obj = env_st.object_list
    road_env_road_len = env_st.road_len
    
    road_index_len = len(road_env_obj)
    
    beh_driving.conf_cruise_speed_set = 22
    veh_road_len = []
    veh_vel_set_mod = []
    
    st_state_sim = []
    
    for road_index in range(road_index_len-200):
        veh_pos_s = road_env_road_len[road_index] + 2
        veh_road_len.append(veh_pos_s)
        cur_road_obj = road_env_obj[road_index]
        cur_object_class = cur_road_obj.object_class
        cur_object_loc_s = cur_road_obj.object_loc_s
        cur_object_param = cur_road_obj.object_param        
        stStatic = beh_driving.Static_state_recog(road_env_obj, veh_pos_s, road_env_road_len)  
        st_state_sim.append(stStatic.state)
        stDynamic = beh_driving.Dynamic_state_recog(100,200)
        veh_vel_set = beh_driving.Lon_vel_set(stStatic,stDynamic)
        veh_vel_set_mod.append(veh_vel_set)
    #%% Set lon velocity    
    os.chdir('data_vehmodel')
    veh_data = io.loadmat('amsa_speed_set.mat')
    os.chdir('..')
    veh_data_roadlen = veh_data['road_len_amsa']
    veh_data_vel = veh_data['veh_speed_amsa']
    
    stStaticList = beh_driving.stStaticList
    
    veh_set_cruise = 22
    veh_set_curve_radcoeff = 1000
    veh_set_curve_discoeff = 0.01
    
    veh_set = 0    
    veh_set_filt = 0
    vel_set_filtnum = 40
    loc_Ts = 1
    
    veh_set_array = []
    veh_set_filt_array = []
    for obj_index in range(len(stStaticList.state)):
        cur_state = stStaticList.state[obj_index]
        cur_param = stStaticList.state_param[obj_index]
        cur_reldis = stStaticList.state_reldis[obj_index]
        if cur_state == 'Curve':
            veh_set = veh_set_cruise - cur_param*veh_set_curve_radcoeff + cur_reldis*veh_set_curve_discoeff
        elif cur_state == 'Cruise': 
            veh_set = veh_set_cruise
        veh_set_filt = Filt_LowPass(veh_set,veh_set_filt,vel_set_filtnum, loc_Ts)
        veh_set_array.append(veh_set)
        veh_set_filt_array.append(veh_set_filt)
        beh_driving.Lon_vel_set
    
    fig = plt.figure()
    ax1 = fig.add_subplot(2,1,1)
    ax2 = fig.add_subplot(2,1,2)
    ax1.plot(veh_data_roadlen,veh_data_vel)
    ax1.plot(veh_road_len,veh_set_array)
    ax1.plot(veh_road_len,veh_set_filt_array)
    ax1.plot(veh_road_len,veh_vel_set_mod)
    
    pass
    



