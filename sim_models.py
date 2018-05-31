# -*- coding: utf-8 -*-
"""
Vehicle simulation _models
==============================================================

Author
~~~~~~~~~~~~~
* kyunghan <kyunghah.min@gmail.com>

Description
~~~~~~~~~~~~~
* Vehicle models
    * Mod_PowerTrain
    * Mod_Body
    * Mod_Veh
* Maneuver models
    * Mod_Driver
    * Mod_Behavior
* Environement models
    * Mod_Env

Update
~~~~~~~~~~~~~
* [18/05/31] - Initial release - kyunghan
"""
# import python lib modules
from math import pi, sin, cos, atan
import numpy as np
from scipy.spatial import distance as dist_calc
# import package modules
from utilities import Calc_Radius, Filt_MovAvg, Calc_PrDis
from type_def import type_pid_controller, type_drvstate, type_objective
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
#%% 2. Maneuver
    # 1. Driver
    # 2. Behavior(Driver)     # Recognition integrated in behavior
class Mod_Driver:
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
        self.set_char('Normal')

    def set_char(self, DriverChar = 'Normal'):
        if DriverChar == 'Normal':
            self.set_driver_param(0.5, 0.1, 0,  0.001, 0.0001, 0,   0.1, 0.1, 0,    1.5, 4)
        elif DriverChar == 'Aggressive':
            self.set_driver_param(0.8, 0.15, 0,  0.001, 0.0001, 0,   0.1, 0.1, 0,    1.5, 4)
        elif DriverChar == 'Defensive':
            self.set_driver_param(0.3, 0.05, 0,  0.001, 0.0001, 0,   0.1, 0.1, 0,    1.5, 4)
        else:
            print('Set the driver only = [''Normal'', ''Aggressive'', ''Defensive'']')
            self.set_driver_param(0.5, 0.1, 0,  0.001, 0.0001, 0,   0.1, 0.1, 0,    1.5, 4)
    def set_driver_param(self, P_gain_lon = 0.5, I_gain_lon = 0.1, D_gain_lon = 0, P_gain_lat = 1, I_gain_lat = 1, D_gain_lat = 0, P_gain_yaw = 1, I_gain_yaw = 1, D_gain_yaw = 0, shift_time = 1.5, max_acc = 4):
        self.P_gain_lon = P_gain_lon; self.I_gain_lon = I_gain_lon; self.D_gain_lon = D_gain_lon
        self.P_gain_lat = P_gain_lat; self.I_gain_lat = I_gain_lat; self.D_gain_lat = D_gain_lat
        self.P_gain_yaw = P_gain_yaw; self.I_gain_yaw = I_gain_yaw; self.D_gain_yaw = D_gain_yaw
        self.shift_time = shift_time; self.max_acc = max_acc

class Mod_Behavior:
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
    def __init__(self, Driver):
        self.stStaticList = type_drvstate()
        self.stDynamicList = type_drvstate()
        self.stStatic = type_drvstate()
        self.stDynamic = type_drvstate()
        self.Maneuver_config()
        self.Drver_set(Driver)
        self.Ts_Loc = globals()['Ts']

    def Lon_control(self,veh_vel_set, veh_vel):
        # Value initialization
        if not 'timer_count' in locals():
            timer_count = 0
            shift_flag_ac = 'on'
            shift_flag_br = 'on'

        trq_set = self.Lon_Controller.Control(veh_vel_set,veh_vel)
        if (trq_set >= 5) & (shift_flag_ac == 'on'):
            stControl = 'acc'
            shift_flag_ac = 'on'
            shift_flag_br = 'off'
            timer_count = 0
        elif (trq_set <= 0) & (shift_flag_br == 'on'):
            stControl = 'brk'
            shift_flag_ac = 'off'
            shift_flag_br = 'on'
            timer_count = 0
        else:
            stControl = 'idle'
            if (shift_flag_ac == 'on') & (timer_count >= self.Driver.shift_time):
                timer_count = 0
                shift_flag_br = 'on'
            elif (shift_flag_br == 'on') & (timer_count >= self.Driver.shift_time):
                timer_count = 0
                shift_flag_ac = 'on'
            else:
                timer_count = timer_count + 1
        # Set value
        if stControl == 'acc':
            acc_out = trq_set/100
            brk_out = 0
        elif stControl == 'brk':
            acc_out = 0
            brk_out = -trq_set/100
        elif stControl == 'idle':
            acc_out = 0
            brk_out = 0
        else:
            acc_out = 0
            brk_out = 0
        self.trq_set_lon = trq_set
        self.u_acc = acc_out
        self.u_brk = brk_out
        return [acc_out, brk_out]


    def Drver_set(self, DriverSet):
        self.Driver = DriverSet
        self.Lon_Controller = type_pid_controller(DriverSet.P_gain_lon, DriverSet.I_gain_lon, DriverSet.D_gain_lon)
        self.Lat_Controller_offset = type_pid_controller(DriverSet.P_gain_lat, DriverSet.I_gain_lat, DriverSet.D_gain_lat)
        self.Lat_Controller_yaw = type_pid_controller(DriverSet.P_gain_yaw, DriverSet.I_gain_yaw, DriverSet.D_gain_yaw)

    def Maneuver_config(self, cruise_speed_set = 15, mincv_speed_set = 5, curve_coef = 1500, transition_dis = 20, forecast_dis = 100, cf_dis = 120, lat_off = 0.5):
        self.conf_cruise_speed_set = cruise_speed_set
        self.conf_mincv_speed_set = mincv_speed_set
        self.conf_curve_coef = curve_coef
        self.conf_transition_dis = transition_dis
        self.conf_forecast_dis = forecast_dis
        self.conf_cf_dis = cf_dis
        self.conf_lat_off = lat_off

    def Static_state_recog(self,static_obj_in, veh_position_s, road_len):
        # Define local state and objectives
        stStatic = type_drvstate()
        forecast_object = type_objective()
        transition_object = type_objective()
        # Determine map_index (forecasting, transition)
        if max(road_len) <= veh_position_s:
            print('========== Simulation is terminated!! ========= ')
            stStatic.set_state('Termination','None','None')
        else:
            tmp_cur_index = np.min(np.where(road_len >= veh_position_s))-1
            tmp_forecast_index = np.min(np.where(road_len >= (tmp_cur_index + self.conf_forecast_dis)))-1
            tmp_transition_index = np.min(np.where(road_len >= (tmp_cur_index + self.conf_transition_dis)))-1
            # Determine objectives from vehicle location to forecasting range
            for k in range(tmp_cur_index,tmp_forecast_index+1):
                forecast_object.merg_object(static_obj_in[k].object_class, static_obj_in[k].object_param, static_obj_in[k].object_loc_s)
            # Determine objectives from transition range to forecasting range
            for k in range(tmp_transition_index,tmp_forecast_index+1):
                transition_object.merg_object(static_obj_in[k].object_class, static_obj_in[k].object_param, static_obj_in[k].object_loc_s)
            if ('Tl' in forecast_object.object_class):
                tmp_Tl_index = forecast_object.object_class.index('Tl')
                tmp_Tl_param = forecast_object.object_param[tmp_Tl_index]
                tmp_Tl_loc = forecast_object.object_loc_s[tmp_Tl_index]
                if tmp_Tl_param == 'red':
                    stStatic.set_state('Tl_stop',tmp_Tl_param,tmp_Tl_loc - tmp_cur_index)
                else:
                    if 'Curve' in transition_object.object_class:
                        tmp_cv_index = np.where(np.array(transition_object.object_class) == 'Curve')[0]
                        tmp_cv_loc = np.mean(np.array(transition_object.object_loc_s)[tmp_cv_index])
                        tmp_cv_param = np.mean(np.array(transition_object.object_param)[tmp_cv_index])
                        stStatic.set_state('Curve',tmp_cv_param,tmp_cv_loc - tmp_cur_index)
                    else:
                        stStatic.set_state('Cruise')
            else:
                if 'Curve' in transition_object.object_class:
                    tmp_cv_index = np.where(np.array(transition_object.object_class) == 'Curve')[0]
                    tmp_cv_loc = np.mean(np.array(transition_object.object_loc_s)[tmp_cv_index])
                    tmp_cv_param = np.mean(np.array(transition_object.object_param)[tmp_cv_index])
                    stStatic.set_state('Curve',tmp_cv_param,tmp_cv_loc - tmp_cur_index)
                else:
                    stStatic.set_state('Cruise')

            self.stStaticList.add_state(stStatic.state, stStatic.state_param, stStatic.state_reldis)
        return stStatic

    def Dynamic_state_recog(self, pre_veh_speed, pre_veh_reldis = 250):
        stDynamic = type_drvstate()
        if pre_veh_reldis >= self.conf_cf_dis:
            stDynamic.set_state('Cruise')
        else:
            stDynamic.set_state('Cf', pre_veh_speed, pre_veh_reldis)
        self.stDynamicList.add_state(stDynamic.state, stDynamic.state_param, stDynamic.state_reldis)
        return stDynamic

    def Lon_vel_set(self, stStatic, stDynamic):
        # Determination of velocity set from static state
        tmp_state_step_static = self.stStatic.state
        if tmp_state_step_static == 'Cruise':
            veh_speed_set_static = self.conf_cruise_speed_set
        elif tmp_state_step_static == 'Tl_stop':
            tmp_state_reldis_step = stStatic.state_reldis
            veh_speed_set_static = self.conf_cruise_speed_set - self.conf_cruise_speed_set*(self.conf_forecast_dis - tmp_state_reldis_step)/self.conf_forecast_dis
        elif tmp_state_step_static == 'Curve':
            tmp_param_step = float(stStatic.state_param)
            # output saturation
            veh_speed_set_static = sorted((self.conf_mincv_speed_set, self.conf_cruise_speed_set - tmp_param_step*self.conf_curve_coef, self.conf_cruise_speed_set))[1]
        else:
            veh_speed_set_static = 0
        # Determination of velocity set from dynamic state
        tmp_state_step_dynamic = stDynamic.state
        if tmp_state_step_dynamic == 'Cruise':
            veh_speed_set_dynamic = self.conf_cruise_speed_set
        else:
            tmp_preveh_vel = stDynamic.state_param # have to set the cruise speed set
            veh_speed_set_dynamic = sorted((0, tmp_preveh_vel, self.conf_cruise_speed_set))[1]

        veh_speed_set = min(veh_speed_set_dynamic,veh_speed_set_static)
        return [veh_speed_set, veh_speed_set_static, veh_speed_set_dynamic]

    def Lon_behavior(self,static_obj_in, veh_position_s, road_len, veh_speed, pre_veh_speed = 'None', pre_veh_reldis = 250):
        self.stStatic = self.Static_state_recog(static_obj_in, veh_position_s, road_len)
        self.stDynamic = self.Dynamic_state_recog(pre_veh_speed, pre_veh_reldis)
        [veh_speed_set, veh_speed_set_static, veh_speed_set_dynamic] = self.Lon_vel_set(self.stStatic,self.stDynamic)
        self.veh_speed_set = veh_speed_set
        [self.acc_out, self.brk_out] = self.Lon_control(veh_speed_set, veh_speed)
        return [self.acc_out, self.brk_out]

    def Lateral_state_recog(self, veh_position_x, veh_position_y, veh_ang, road_x, road_y):
        stLateral = type_drvstate()
        [lon_offset, lat_offset, direction, min_index, veh_an, road_an] = Calc_PrDis(road_x, road_y, [veh_position_x, veh_position_y])
        if veh_ang < 0:
            veh_ang = veh_ang + 2*pi
        angle_diff = road_an - veh_ang
        if angle_diff >= pi/2:
            angle_diff = angle_diff - 2*pi
        elif angle_diff <= -pi/2:
            angle_diff = angle_diff + 2*pi
        else:
            angle_diff = angle_diff
        stLateral.set_state(direction, angle_diff, lat_offset)
        self.state_veh_an = veh_an
        self.road_an = road_an
        return stLateral

    def Lat_behavior(self, veh_position_x, veh_position_y, veh_ang, road_x, road_y):
        self.stLateral = self.Lateral_state_recog(veh_position_x, veh_position_y, veh_ang, road_x, road_y)
        angle_offset = self.stLateral.state_param
        lane_offset = self.stLateral.state_reldis
        if self.stLateral.state == 'Left':
            lane_offset = -lane_offset
        else:
            lane_offset = lane_offset
        self.lane_offset = lane_offset
        self.psi_offset = angle_offset
        self.steer_out = self.Lat_control(lane_offset, angle_offset)
        return self.steer_out

    def Lat_control(self,lane_offset, angle_dif, offset_des = 0, angle_diff_des = 0):
        steer_out_offset = self.Lat_Controller_offset.Control(offset_des,lane_offset)
        steer_out_yaw = self.Lat_Controller_yaw.Control(angle_diff_des,-angle_dif)
        steer_out = steer_out_offset + steer_out_yaw
        self.u_steer_offset = steer_out_offset
        self.u_steer_yaw = steer_out_yaw
        return steer_out
#%% 3. Environment model
    # Road, static objects
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
        loc_env_road_s = self.road_len
        tmp_s_index = np.min(np.where(loc_env_road_s >= object_s_location)) - 1
        self.object_list[tmp_s_index].add_object(object_in,object_param_in,object_s_location)

    def Road_curve_def(self, road_array_x_in, road_array_y_in, loc_env_road_s, conf_curve_val = 0.001):
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
        veh_mod.pos_x_veh = self.road_x[road_index]
        veh_mod.pos_y_veh = self.road_y[road_index]
        veh_mod.psi_veh = atan((self.road_y[road_index+1] - self.road_y[road_index])/(self.road_x[road_index+1] - self.road_x[road_index])) + (1 - (self.road_x[road_index+1] - self.road_x[road_index])/abs((self.road_x[road_index+1] - self.road_x[road_index])))/2*pi

#%%  ----- test ground -----
if __name__ == "__main__":
    pass
