# -*- coding: utf-8 -*-
"""
Simulation model : Vehicle
==============================================================

Author
~~~~~~~~~~~~~
* kyunghan <kyunghah.min@gmail.com>

Description
~~~~~~~~~~~~~
* Powertrain model
* Body model
* Vehicle model

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

class Mod_PowerTrain:
    """
    Electric vhielc powertrain model

    ConfigVariables:
        * conf_rm_mot: motor registance [ohm].
        * conf_lm_mot: motor electric induction [Hennry]
        * conf_kb_mot: motor back emp coeff [-]
        * conf_kt_mot: motor torque constant [Nm/A]
        * conf_jm_mot: motor inertia [kg*m^2/rad..?]
        * conf_trq_gain: torque efficiency [-]
        * conf_rd_gear: reduction gear ratio [-]
        * conf_ks_shaft: shaft stifness [Nm/rad]

    Submodules:
        * Motor_config: set config variables
        * DriveTrain_config: set config variables
        * Motor_driven: Driven motor
        * Motor_control: Control motor torque
        * Motor_elect_dynamics: 1st order dynamics for motor electric system
        * Motor_mech_dynamics: 1st order dynamics for motor mechanical system
        * Drive_shaft_dynamics: 1st order dynamics for shaft rotational system
        * Motor_torque_system: set motor voltage

    Operation:
        Motor_control module can control the motor to desired torque::

            # Module_name(in//out)
            Motor_control(t_mot, w_mot, t_load // des_torque)
                >> Motor_driven(t_mot, w_mot, t_load // v_mot)
                    >> Motor_elec_dynamics, Motor_mech_dynamics, Drive_shaft_dynamics
                >> Motor_torque_system(v_mot // des_torque)

    """
    def __init__(self):
        self.w_mot = 0
        self.t_mot = 0
        self.t_load = 0
        self.Motor_config()
        self.DriveTrain_config()
        self.Ts_loc = globals()['Ts']

    def Motor_config(self, conf_rm = 0.1, conf_lm = 0.1, conf_kb = 6.5e-4, conf_kt = 0.1, conf_jm = 1e-3, conf_trq_gain = 1):
        """Motor parameter configuration

        Parameters not specified are declared as default values

        If you want set a specific parameter don't use this function,
        just type::

            >>> Mod_PowerTrain.conf_rm_mot = 0.2
            ...

        Args:
            Motor parameter values, default values are setted
        """
        self.conf_rm_mot = conf_rm
        self.conf_lm_mot = conf_lm
        self.conf_kb_mot = conf_kb
        self.conf_kt_mot = conf_kt
        self.conf_jm_mot = conf_jm
        self.conf_trq_gain = conf_trq_gain


    def DriveTrain_config(self, conf_rd = 8, conf_ks = 0.01):
        """Drivetrain parameter configuration

        Parameters not specified are declared as default values

        If you want set a specific parameter don't use this function,
        just type::

            >>> Mod_PowerTrain.conf_rd_gear = 7
            ...

        Args:
            * Driver shaft parameter values, default values are setted
        """
        self.conf_rd_gear = conf_rd
        self.conf_ks_shaft = conf_ks

    def Motor_driven(self, v_in = 0, w_shaft = 0):
        """Motor driven function

        Generate motor output(torque, speed) and load torque according to input voltage and wheel speed (shaft speed = wheel speed)

        Contain theree modules ``Elecic dynamics``, ``Menahicla dynamics``, ``Shaft dynamics``

        Args:
            * v_in: motor input voltage [V]
            * w_shaft: rotational speed of drive shaft from body model [rad/s]

        returns:
            * t_mot: motor torque [Nm]
            * w_mot: motor rotational speed [rad/s]
            * t_load: load torque from body model [Nm]
        """
        # Elec motor model: Motor torque --> Mech motor model: Motor speed --> Drive shaft model: Load torque
        self.t_mot = self.Motor_elec_dynamics(self.t_mot, v_in, self.w_mot)
        self.w_mot = self.Motor_mech_dynamics(self.w_mot, self.t_mot, self.t_load)
        self.t_load = self.Drive_shaft_dynamics(self.t_load, self.w_mot, w_shaft)
        return [self.w_mot, self.t_mot, self.t_load]

    def Motor_control(self, t_mot_des = 0, w_shaft = 0):
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
        v_in = self.Motor_torque_system(t_mot_des)
        self.Motor_driven(v_in, w_shaft)
        return [self.w_mot, self.t_mot, self.t_load]

    def Motor_elec_dynamics(self, t_mot, v_in, w_mot):
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
        # Motor torque calculation
        t_mot = t_mot*(1 - self.conf_rm_mot/self.conf_lm_mot * self.Ts_loc) \
        + self.Ts_loc*self.conf_kt_mot/self.conf_lm_mot * (v_in - self.conf_kb_mot * w_mot)
        return t_mot

    def Motor_mech_dynamics(self, w_mot, t_mot, t_load):
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
        # Motor speed calculation
        w_mot =  w_mot + self.Ts_loc*(t_mot - t_load/self.conf_rd_gear)/self.conf_jm_mot
        return w_mot

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

    def Motor_torque_system(self, t_mot_des):
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
        v_in = self.conf_trq_gain * t_mot_des
        return v_in
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
        self.Body_config()
        self.Dyn_config()
        self.Ts_loc = globals()['Ts']

    def Body_config(self, conf_rw_wheel = 0.3, conf_jw_body = 2, conf_brk_coef = 100, conf_acc_coef = 100, conf_veh_len = 2):
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

    def Dyn_config(self, conf_airdrag = 4, conf_add_weight = 0):
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

    def Lon_driven_in(self, u_acc, u_brake, veh_vel = 0):
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
        self.t_brake = self.Brake_system(u_brake)
        self.t_mot_des = self.Acc_system(u_acc)
        self.t_drag = self.Drag_system(veh_vel)
        return self.t_mot_des, self.t_brake, self.t_drag

    def Lon_driven_out(self,t_load, t_brake, t_drag):
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
        self.w_wheel = self.Tire_dynamics(self.w_wheel, t_load, t_brake, t_drag)
        self.vel_veh = self.w_wheel * self.conf_rw_wheel
        return [self.w_wheel, self.vel_veh]

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
        return w_wheel

    def Brake_system(self, u_brake):
        if self.w_wheel <= 0:
            t_brake = 0
        else:
            t_brake = u_brake * self.conf_brk_coef
        return t_brake

    def Acc_system(self, u_acc):
        t_mot_des = u_acc * self.conf_acc_coef
        return t_mot_des

    def Drag_system(self, veh_vel):
        t_drag = self.conf_drag_lon * veh_vel
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
        t_load = self.ModPower.t_load
        w_shaft = self.ModBody.w_wheel
        veh_vel = self.ModBody.vel_veh
        # Lateral motion
        the_wheel = self.ModBody.Lat_driven(u_steer)
        # Longitudinal motion
        # Body_Lon_in --> Powertrain_Motor --> Body_Lon_out
        [t_mot_des, t_brake, t_drag] = self.ModBody.Lon_driven_in(u_acc, u_brake, veh_vel)
        [w_mot, t_mot, t_load] = self.ModPower.Motor_control(t_mot_des, w_shaft)
        [w_wheel, vel_veh] = self.ModBody.Lon_driven_out(t_load, t_brake, t_drag)
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
