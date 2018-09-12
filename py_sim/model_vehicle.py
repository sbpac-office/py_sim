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
* [18/06/11] - Revision 02 - kyunghan
  - Modify drivetrain module to body module
  - Modify model configurations
  - Add regeneration module in body.brake_system
* [18/08/08] - Revision 03 - kyunghan
  - Modify drive train module
  - Modify tire module
  - Modify drag force module
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
        self.t_mot_load = 0
        self.t_driven = 0
        self.t_wheel_load = 0
        self.t_wheel_traction_f = 0
        self.w_wheel = 0
        self.w_shaft = 0
        self.w_motor = 0
        self.w_vehicle = 0
        self.Drivetrain_config()
        self.Ts_loc = globals()['Ts']

    def Drivetrain_config(self, conf_veh_len = 2, conf_rd_wheel = 0.301, conf_jw_wheel = 0.1431, conf_jw_diff_in = 0.015, conf_jw_diff_out = 0.015, conf_jw_trns_out = 0.015, conf_jw_trns_in = 0.01, conf_jw_mot = 0.005,
                    conf_eff_trns = 0.96, conf_eff_diff = 0.9796, conf_eff_diff_neg = 0.9587, conf_gear = 6.058, conf_mass_veh = 1200, conf_mass_add = 0):
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
        self.conf_veh_len = conf_veh_len
        self.conf_gear = conf_gear
        self.conf_mass_veh = conf_mass_veh + conf_mass_add
        self.conf_rd_wheel = conf_rd_wheel
        # rotational inertia
        self.conf_jw_wheel = conf_jw_wheel
        self.conf_jw_diff_in = conf_jw_diff_in
        self.conf_jw_diff_out = conf_jw_diff_out
        self.conf_jw_trns_in = conf_jw_trns_in
        self.conf_jw_trns_out = conf_jw_trns_out
        self.conf_jw_mot = conf_jw_mot
        # equivalence inertia
        self.conf_jw_wheel_eq_f = conf_jw_wheel + conf_jw_diff_out*2
        self.conf_jw_wheel_eq_r = conf_jw_wheel + conf_jw_diff_out
        self.conf_jw_shaft_eq = (conf_jw_mot*conf_gear + conf_jw_trns_in*conf_gear + conf_jw_trns_out + conf_jw_diff_in + conf_jw_diff_out + conf_jw_wheel)*2;
        self.conf_jw_body_eq = self.conf_mass_veh * conf_rd_wheel**2 + 2*(self.conf_jw_wheel_eq_f+self.conf_jw_wheel_eq_r)
        self.conf_jw_vehicle_eq = self.conf_jw_shaft_eq + self.conf_jw_body_eq
        self.conf_jw_vehicle = self.conf_mass_veh * conf_rd_wheel**2
        # efficiency
        self.conf_eff_trns = conf_eff_trns
        self.conf_eff_diff = conf_eff_diff
        self.conf_eff_diff_neg = conf_eff_diff_neg
        # equivalence efficiency
        self.conf_eff_eq_pos = 1 - (1-conf_eff_trns + conf_eff_trns*(1-conf_eff_diff))
        self.conf_eff_eq_neg = 1- ((1/conf_eff_trns - 1) + conf_eff_trns*(1/conf_eff_diff_neg - 1))


    def Lon_equivalence(self,t_mot, t_brk, t_drag):
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
            * w_shaft: shaft rotational speed [rad/s]
        """
        # shaft torque calculation
        t_motor_in = t_mot * self.conf_gear
        t_brk_wheel = t_brk/4
        if t_motor_in >= 0:
            t_shaft_loss = t_motor_in*(1-self.conf_eff_eq_pos)
        else:
            t_shaft_loss = -t_motor_in*(1-self.conf_eff_eq_neg)
        t_shaft_in = t_motor_in - t_shaft_loss
        # vehicle equivalence
        t_driven = t_shaft_in - t_brk - t_drag
        w_dot_wheel = t_driven/self.conf_jw_vehicle_eq
        self.w_vehicle = self.w_vehicle + w_dot_wheel*self.Ts_loc   
        if self.w_vehicle <= -0.01:
            w_dot_wheel = 0
            self.w_vehicle = 0            
        # load torque calculation - vehicle traction
        t_veh_traction = self.conf_jw_vehicle * w_dot_wheel + t_drag
        f_lon = t_veh_traction/self.conf_rd_wheel
        # load torque calculation
        t_shaft_out = t_shaft_in - self.conf_jw_shaft_eq*w_dot_wheel
        # load torque calculation
        t_wheel_in = t_shaft_out/2
        # load torque calculation - wheel load
        t_wheel_traction_r = -t_brk_wheel-w_dot_wheel*self.conf_jw_wheel_eq_r
        t_wheel_traction_f = (t_veh_traction - 2*t_wheel_traction_r)/2
        # load torque calculation - motor load
        t_mot_load = t_mot - w_dot_wheel*self.conf_gear*(self.conf_jw_mot + self.conf_jw_trns_in)
        self.t_mot_load = t_mot_load
        self.t_driven = t_driven
        self.t_shaft_out = t_shaft_in
        self.t_shaft_out = t_shaft_out
        self.t_shaft_loss = t_shaft_loss
        self.t_wheel_in = t_wheel_in
        self.t_wheel_traction_f = t_wheel_traction_f
        self.w_dot_vehicle = w_dot_wheel                    
        return t_mot_load, t_shaft_in, t_shaft_out, t_wheel_in, t_wheel_traction_f, t_driven, f_lon

    def Driveshaft_dynamics(self, t_shaft_in, t_shaft_out, w_shaft):
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
            * w_shaft: shaft rotational speed [rad/s]
        """
        w_dot_shaft = (t_shaft_in - t_shaft_out)/self.conf_jw_shaft_eq
        w_shaft = w_shaft + self.Ts_loc * w_dot_shaft
        self.w_dot_shaft = w_dot_shaft
        self.w_shaft = w_shaft
        return w_shaft

    def Tire_dynamics(self, t_wheel_load, t_wheel_traction_f, t_brk, w_wheel):
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
        t_brk_w = t_brk/4
        w_dot_wheel = (t_wheel_load - t_wheel_traction_f - t_brk_w)/self.conf_jw_wheel_eq_f
        w_wheel = w_wheel + self.Ts_loc*w_dot_wheel
        self.w_dot_wheel = w_dot_wheel
#        if w_wheel <=0:
#            w_wheel = 0
        self.w_wheel = w_wheel
        return w_wheel

    def Motor_dynamics(self, t_mot, t_mot_load, w_motor):
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
        w_dot_motor = (t_mot - t_mot_load)/(self.conf_jw_mot + self.conf_jw_trns_in)
        w_motor = w_motor + self.Ts_loc*w_dot_motor
        self.w_dot_motor = w_dot_motor
        self.w_motor = w_motor
        return w_motor


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
        * Include ``powertrain``, ``drivetrain`` models

    Operation:
        Description operation here::

            !!!Make operation diagram here!!!
            # Module_name(in//out)
            Motor_control(t_mot, w_mot // des_torque)
                >> Motor_driven(t_mot, w_mot // v_mot)
                    >> Motor_elec_dynamics, Motor_mech_dynamics, Drive_shaft_dynamics
                >> Motor_torque_system(v_mot // des_torque)

    """
    def __init__(self,powertrain_model,drivetrain_model):
        self.ModPower = powertrain_model
        self.ModDrive = drivetrain_model
        self.Ts_loc = globals()['Ts']
        self.Set_initState()
        self.Veh_config()

    def Set_initState(self, x_veh = 0, y_veh = 0, s_veh = 0, n_veh = 0, psi_veh = 0, vel_veh = 0, theta_wheel = 0):
        self.pos_x_veh = x_veh
        self.pos_y_veh = y_veh
        self.pos_s_veh = s_veh
        self.pos_n_veh = n_veh
        self.psi_veh = psi_veh
        self.vel_veh = vel_veh
        self.the_wheel = theta_wheel

    def Veh_config(self, conf_drag_air_coef = 0, conf_add_weight = 0, conf_drag_ca = 143.06, conf_drag_cc = 0.4405,
                   conf_veh_len = 2,conf_acc_trq_fac = 82.76, conf_brk_trq_fac = 501.8, conf_motreg_max = 100):
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
        self.conf_drag_air_coef = conf_drag_air_coef        
        self.conf_weight_veh = conf_add_weight
        self.conf_drag_ca = conf_drag_ca
        self.conf_drag_cc = conf_drag_cc
        self.conf_brk_trq_fac = conf_brk_trq_fac
        self.conf_acc_trq_fac = conf_acc_trq_fac
        self.conf_veh_len = conf_veh_len
        self.conf_veh_mass = self.ModDrive.conf_mass_veh
        self.conf_rd_wheel = self.ModDrive.conf_rd_wheel
        self.conf_motreg_max = conf_motreg_max
        self.swtRegCtl = 0

    def Veh_position_update(self, vel_veh = 0, the_wheel = 0):
        veh_len = self.ModDrive.conf_veh_len
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

    def Veh_driven(self, u_acc, u_brake, u_steer = 0):
        # Longitudinal driven
        vel_veh = self.Veh_lon_driven(u_acc, u_brake)
        # Lateral driven
        the_wheel = self.Veh_lat_driven(u_steer)
        return vel_veh, the_wheel

    def Veh_lon_driven(self, u_acc, u_brake):        
        w_mot = self.ModDrive.w_motor
        w_shaft = self.ModDrive.w_shaft
        w_wheel = self.ModDrive.w_wheel
        # Calculation of torque set
        t_mot_set = self.Acc_system(u_acc)
        t_brk, t_mot_reg_set = self.Brake_system(u_brake)
        t_drag, f_drag = self.Drag_system(self.vel_veh)
        # Power control
        t_mot_des = t_mot_set - t_mot_reg_set
        w_mot, t_mot = self.ModPower.ModMotor.Motor_driven(t_mot_des, w_mot)
        self.t_mot_des = t_mot_des
        # Body equivalence
        t_mot_load, t_shaft_in, t_shaft_out, t_wheel_in, t_wheel_traction_f, t_driven, f_lon = self.ModDrive.Lon_equivalence(t_mot,t_brk,t_drag)
        self.f_lon = f_lon
        # Shaft dynamics
        w_mot = self.ModDrive.Motor_dynamics(t_mot, t_mot_load, w_mot)
        w_shaft = self.ModDrive.Driveshaft_dynamics(t_shaft_in, t_shaft_out, w_shaft)
        w_wheel = self.ModDrive.Tire_dynamics(t_wheel_in, t_wheel_traction_f, t_brk, w_wheel)
        # Vehicle dynamics
        self.vel_veh, self.veh_acc = self.Veh_lon_dynamics(f_lon, f_drag, self.vel_veh)
        return self.vel_veh

    def Veh_lon_dynamics(self, f_lon, f_drag, vel_veh):
        veh_acc = (f_lon - f_drag)/self.conf_veh_mass
        vel_veh_calc = vel_veh + self.Ts_loc*veh_acc
        vel_veh = sorted((0., vel_veh_calc, 1000.))[1]
        return vel_veh, veh_acc

    def Veh_lat_driven(self, u_steer):
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
        self.the_wheel = self.Veh_lat_dynamics(self.the_wheel, u_steer)
        return self.the_wheel

    def Veh_lat_dynamics(self, the_wheel, u_steer):
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

    def Acc_system(self, u_acc):
        self.t_mot_des = u_acc * self.conf_acc_trq_fac
        return self.t_mot_des

    def Brake_system(self, u_brake, t_reg_set = 0):
#        if self.w_wheel <= 0.01:
#            t_brake_set = 0
#        else:
#            t_brake_set = u_brake * self.conf_brk_coef
        t_brake_set = u_brake * self.conf_brk_trq_fac
        t_brake_set_mot_eq = t_brake_set/self.ModDrive.conf_gear
        # Regeneration control
        if self.swtRegCtl == 1:
            if (self.conf_motreg_max - t_brake_set_mot_eq) >= 0:
                t_mot_reg = t_brake_set_mot_eq
                t_brake = 0
            else:
                t_brake = (t_brake_set - self.conf_motreg_max/self.ModDrive.conf_gear)
                t_mot_reg = self.conf_motreg_max
        elif self.swtRegCtl == 2:
            t_brake = t_brake_set
            t_mot_reg = t_reg_set
        else:
            t_brake = t_brake_set
            t_mot_reg = 0
        self.t_brake = t_brake
        self.t_mot_reg = t_mot_reg
        return t_brake, t_mot_reg

    def Drag_system(self, vel_veh):        
        f_drag_roll = self.conf_drag_ca + self.conf_drag_cc*vel_veh**2
        f_drag_air = 0.5*1.25*self.conf_drag_air_coef*vel_veh**2
        f_drag = f_drag_roll + f_drag_air
        if vel_veh < 0.1:
            f_drag = 0
        t_drag = f_drag * self.conf_rd_wheel
        self.f_drag = f_drag
        self.t_drag = t_drag
        return t_drag, f_drag
#%%  ----- test ground -----
if __name__ == "__main__":
    pass
