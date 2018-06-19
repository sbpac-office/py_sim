# -*- coding: utf-8 -*-
"""
Simulation model : Maneuver
==============================================================

Author
~~~~~~~~~~~~~
* kyunghan <kyunghah.min@gmail.com>

Description
~~~~~~~~~~~~~
* Driver model
* Behavior model

Update
~~~~~~~~~~~~~
* [18/05/31] - Initial release - kyunghan
* [18/06/05] - Modification of lon control - kyunghan
"""
# import python lib modules
from math import pi
import numpy as np
# import package modules
from sub_utilities import Calc_PrDis
from sub_type_def import type_pid_controller, type_drvstate, type_objective, type_hyst
# import config data modules

# simulation sampling time
Ts = 0.01
"""global vairable: simulation sampling timeself.

you can declare other sampling time in application as vairable ``Ts``

"""
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
            self.set_driver_param(1, 0.05, 0,  0.001, 0.0001, 0,   0.1, 0.1, 0,    1.5, 4)
        elif DriverChar == 'Aggressive':
            self.set_driver_param(1.2, 0.08, 0,  0.001, 0.0001, 0,   0.1, 0.1, 0,    1.5, 4)
        elif DriverChar == 'Defensive':
            self.set_driver_param(0.8, 0.03, 0,  0.001, 0.0001, 0,   0.1, 0.1, 0,    1.5, 4)
        else:
            print('Set the driver only = [''Normal'', ''Aggressive'', ''Defensive'']')
            self.set_driver_param(1, 0.05, 0,  0.001, 0.0001, 0,   0.1, 0.1, 0,    1.5, 4)
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
        self.hysfLonCtl = type_hyst(1, -1)

    def Drver_set(self, DriverSet):
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
        self.Driver = DriverSet
        self.Lon_Controller = type_pid_controller(DriverSet.P_gain_lon, DriverSet.I_gain_lon, DriverSet.D_gain_lon)
        self.Lat_Controller_offset = type_pid_controller(DriverSet.P_gain_lat, DriverSet.I_gain_lat, DriverSet.D_gain_lat)
        self.Lat_Controller_yaw = type_pid_controller(DriverSet.P_gain_yaw, DriverSet.I_gain_yaw, DriverSet.D_gain_yaw)
        self.stLonControl = 'idle'

    def Maneuver_config(self, cruise_speed_set = 15, mincv_speed_set = 5, curve_coef = 1500, transition_dis = 20, forecast_dis = 200, cf_dis = 120, lat_off = 0.5):
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
        self.conf_cruise_speed_set = cruise_speed_set
        self.conf_mincv_speed_set = mincv_speed_set
        self.conf_curve_coef = curve_coef
        self.conf_transition_dis = transition_dis
        self.conf_forecast_dis = forecast_dis
        self.conf_cf_dis = cf_dis
        self.conf_lat_off = lat_off

    def Static_state_recog(self,static_obj_in, veh_position_s, road_len):
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
            tmp_forecast_index = np.min(np.where(road_len >= (veh_position_s + self.conf_forecast_dis)))-1
            tmp_transition_index = np.min(np.where(road_len >= (veh_position_s + self.conf_transition_dis)))-1
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
                    stStatic.set_state('Tl_stop',tmp_Tl_param,tmp_Tl_loc - veh_position_s)
                else:
                    if 'Curve' in transition_object.object_class:
                        tmp_cv_index = np.where(np.array(transition_object.object_class) == 'Curve')[0]
                        tmp_cv_loc = np.mean(np.array(transition_object.object_loc_s)[tmp_cv_index])
                        tmp_cv_param = np.mean(np.array(transition_object.object_param)[tmp_cv_index])
                        stStatic.set_state('Curve',tmp_cv_param,tmp_cv_loc - veh_position_s)
                    else:
                        stStatic.set_state('Cruise')
            else:
                if 'Curve' in transition_object.object_class:
                    tmp_cv_index = np.where(np.array(transition_object.object_class) == 'Curve')[0]
                    tmp_cv_loc = np.mean(np.array(transition_object.object_loc_s)[tmp_cv_index])
                    tmp_cv_param = np.mean(np.array(transition_object.object_param)[tmp_cv_index])
                    stStatic.set_state('Curve',tmp_cv_param,tmp_cv_loc - veh_position_s)
                else:
                    stStatic.set_state('Cruise')

            self.stStaticList.add_state(stStatic.state, stStatic.state_param, stStatic.state_reldis)
            self.forecast_object = forecast_object
            self.transition_object = transition_object
        return stStatic

    def Dynamic_state_recog(self, pre_veh_speed, pre_veh_reldis = 250):
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
        stDynamic = type_drvstate()
        if pre_veh_reldis >= self.conf_cf_dis:
            stDynamic.set_state('Cruise')
        else:
            stDynamic.set_state('Cf', pre_veh_speed, pre_veh_reldis)
        self.stDynamicList.add_state(stDynamic.state, stDynamic.state_param, stDynamic.state_reldis)
        return stDynamic

    def Lon_vel_set(self, stStatic, stDynamic):
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
        self.stStatic = self.Static_state_recog(static_obj_in, veh_position_s, road_len)
        self.stDynamic = self.Dynamic_state_recog(pre_veh_speed, pre_veh_reldis)
        [veh_speed_set, veh_speed_set_static, veh_speed_set_dynamic] = self.Lon_vel_set(self.stStatic,self.stDynamic)
        self.veh_speed_set = veh_speed_set
        [acc_out, brk_out] = self.Lon_control(veh_speed_set, veh_speed)

        return [self.acc_out, self.brk_out]
    
    def Lon_control(self,veh_vel_set, veh_vel):
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
        # State definition - Hysteresis filter with shift time
        vel_error = veh_vel_set - veh_vel
        stLonControl = self.hysfLonCtl.filt(vel_error)
        # Reset control values when transition
        if stLonControl == 1:
            stControl = 'acc'
            if self.stLonControl != 'acc':
                self.Lon_Controller.I_val_old = 0
        else:
            stControl = 'brk'
            if self.stLonControl != 'brk':
                self.Lon_Controller.I_val_old = 0
        # Convert torque set 
                
        # Vehicle torque control
        trq_set = self.Lon_Controller.Control(veh_vel_set,veh_vel)

        # Determine state

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
        self.stLonControl = stControl
        self.u_acc = sorted((0., acc_out, 1.))[1]
        self.u_brk = sorted((0., brk_out, 1.))[1]
        return [self.u_acc, self.u_brk]

    def Lateral_state_recog(self, veh_position_x, veh_position_y, veh_ang, road_x, road_y):
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
        steer_out_offset = self.Lat_Controller_offset.Control(offset_des,lane_offset)
        steer_out_yaw = self.Lat_Controller_yaw.Control(angle_diff_des,-angle_dif)
        steer_out = steer_out_offset + steer_out_yaw
        self.u_steer_offset = steer_out_offset
        self.u_steer_yaw = steer_out_yaw
        return steer_out
#%%  ----- test ground -----
if __name__ == "__main__":
    pass
