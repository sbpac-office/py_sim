# -*- coding: utf-8 -*-
"""
Simulation model : Vehicle
==============================================================

Author
~~~~~~~~~~~~~
* kyunghan <kyunghah.min@gmail.com>

Description
~~~~~~~~~~~~~
* Body model
* Vehicle model

Update
~~~~~~~~~~~~~
* [18/05/31] - Initial release - kyunghan
* [18/06/01] - Revision 01 - Kyuhwan
  - Seperate powertrain class in new file
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


class Mod_Body:
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
    def __init__(self):
        self.w_wheel = 0
        self.vel_veh = 0
        self.the_wheel = 0
        self.t_brake = 0
        self.t_mot_des = 0
        self.t_drag = 0
        self.swtRegCtl = 0
        self.t_reg_set = 0
        self.t_load = 0
        self.Body_config()
        self.Dyn_config()
        self.Ts_loc = globals()['Ts']

    def Body_config(self, conf_rw_wheel = 0.3, conf_jw_body = 60, conf_brk_coef = 100, conf_acc_coef = 100, conf_veh_len = 2, conf_motreg_max = 40, conf_rd = 8, conf_ks = 100):
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
        self.conf_rw_wheel = conf_rw_wheel
        self.conf_jw_body = conf_jw_body
        self.conf_brk_coef = conf_brk_coef
        self.conf_acc_coef = conf_acc_coef
        self.conf_veh_len = conf_veh_len
        self.conf_motreg_max = conf_motreg_max
        self.conf_rd_gear = conf_rd
        self.conf_ks_shaft = conf_ks

    def Dyn_config(self, conf_airdrag = 4, conf_add_weight = 0, conf_drag_rol = 0.01):
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
        self.conf_drag_lon = conf_airdrag
        self.conf_weight_veh = conf_add_weight
        self.conf_drag_rol = conf_drag_rol

    def Lon_driven_in(self, u_acc, u_brake):
        """Input module to powertrain

        Determine torque set-points for motor, drag, brake

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
        t_reg_set = self.t_reg_set
        [self.t_brake, self.t_mot_reg] = self.Brake_system(u_brake, t_reg_set)
        self.t_mot_des = self.Acc_system(u_acc)
        self.t_mot_set = self.t_mot_des + self.t_mot_reg
        return self.t_mot_set, self.t_brake

    def Lon_driven_out(self, t_brake, w_mot):
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
        self.t_load = self.Drive_shaft_dynamics(self.t_load, w_mot, self.w_wheel)
        self.w_wheel = self.Tire_dynamics(self.w_wheel, self.t_load, t_brake, self.t_drag)
        self.vel_veh = self.w_wheel * self.conf_rw_wheel
        self.t_drag = self.Drag_system(self.vel_veh)
        t_mot_load = self.t_load/self.conf_rd_gear
        return [self.w_wheel, t_mot_load, self.vel_veh]

    def Lat_driven(self, u_steer):
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
        self.the_wheel = self.Lat_dynamics(self.the_wheel, u_steer)
        return self.the_wheel

    def Lat_dynamics(self, the_wheel, u_steer):
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
        the_wheel = the_wheel + self.Ts_loc/0.2*(u_steer - the_wheel)
        return the_wheel

    def Drive_shaft_dynamics(self, t_load, w_mot, w_shaft):
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
        t_load = t_load + self.Ts_loc*self.conf_ks_shaft*(w_mot/self.conf_rd_gear - w_shaft)
        return t_load

    def Tire_dynamics(self, w_wheel, t_load, t_brake = 0, t_drag = 0):
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
        w_wheel = w_wheel + self.Ts_loc/self.conf_jw_body*(t_load - t_drag - t_brake)
        if w_wheel <=0:
            w_wheel = 0
        return w_wheel

    def Brake_system(self, u_brake, t_reg_set = 0):
#        if self.w_wheel <= 0.01:
#            t_brake_set = 0
#        else:
#            t_brake_set = u_brake * self.conf_brk_coef
        t_brake_set = u_brake * self.conf_brk_coef
        # Regeneration control
        if self.swtRegCtl == 1:
            if (self.conf_motreg_max - t_brake_set) >= 0:
                t_mot_reg = - t_brake_set
                t_brake = 0
            else:
                t_brake = (t_brake_set - self.conf_motreg_max) * self.conf_rd_gear
                t_mot_reg = - self.conf_motreg_max
        elif self.swtRegCtl == 2:
            t_brake = t_brake_set * self.conf_rd_gear
            t_mot_reg = t_reg_set
        else:
            t_brake = t_brake_set * self.conf_rd_gear
            t_mot_reg = 0
        return t_brake, t_mot_reg

    def Reg_system(self, t_reg_set):
        self.t_reg_set = t_reg_set
        return t_reg_set

    def Acc_system(self, u_acc):
        t_mot_des = u_acc * self.conf_acc_coef
        return t_mot_des

    def Drag_system(self, veh_vel):
        t_drag = self.conf_drag_lon * veh_vel + self.conf_drag_rol*(1000 + self.conf_weight_veh)
        if t_drag < 0:
            t_drag = 0
        return t_drag


class Mod_Veh:
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

    Initialization:
        * Include ``powertrain``, ``body`` models

    Operation:
        Description operation here::

            !!!Make operation diagram here!!!
            # Module_name(in//out)
            Motor_control(t_mot, w_mot // des_torque)
                >> Motor_driven(t_mot, w_mot // v_mot)
                    >> Motor_elec_dynamics, Motor_mech_dynamics, Drive_shaft_dynamics
                >> Motor_torque_system(v_mot // des_torque)

    """
    def __init__(self,powertrain_model,body_model):
        self.ModPower = powertrain_model
        self.ModBody = body_model
        self.Ts_loc = globals()['Ts']
        self.Set_initState()

    def Set_initState(self, x_veh = 0, y_veh = 0, s_veh = 0, n_veh = 0, psi_veh = 0):
        self.pos_x_veh = x_veh
        self.pos_y_veh = y_veh
        self.pos_s_veh = s_veh
        self.pos_n_veh = n_veh
        self.psi_veh = psi_veh

    def Veh_driven(self, u_acc = 0, u_brake = 0, u_steer = 0):
        w_mot = self.ModPower.ModMotor.w_mot
        # Lateral motion
        the_wheel = self.ModBody.Lat_driven(u_steer)
        # Longitudinal motion
        #  Body_Lon_in --> Powertrain_Motor --> Body_Lon_out
        [t_mot_des, t_brake] = self.ModBody.Lon_driven_in(u_acc, u_brake)
        [w_wheel, t_mot_load, vel_veh] = self.ModBody.Lon_driven_out(t_brake, w_mot)
        [w_mot, t_mot] = self.ModPower.ModMotor.Motor_control(t_mot_des, t_mot_load)
        return vel_veh, the_wheel

    def Veh_position_update(self, vel_veh = 0, the_wheel = 0):
        veh_len = self.ModBody.conf_veh_len
        ang_veh = the_wheel + self.psi_veh
        x_dot = vel_veh*cos(ang_veh)
        self.pos_x_veh = self.pos_x_veh + x_dot*self.Ts_loc
        y_dot = vel_veh*sin(ang_veh)
        self.pos_y_veh = self.pos_y_veh + y_dot*self.Ts_loc
        s_dot = vel_veh*cos(the_wheel)
        self.pos_s_veh = self.pos_s_veh + s_dot*self.Ts_loc
        n_dot = vel_veh*sin(the_wheel)
        self.pos_n_veh = self.pos_n_veh + n_dot*self.Ts_loc
        psi_dot = vel_veh/veh_len*the_wheel
        self.psi_veh = self.psi_veh + psi_dot*self.Ts_loc
        return [self.pos_x_veh, self.pos_y_veh, self.pos_s_veh, self.pos_n_veh, self.psi_veh]
#%%  ----- test ground -----
if __name__ == "__main__":
    pass
