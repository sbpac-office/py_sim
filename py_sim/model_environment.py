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
    pass
