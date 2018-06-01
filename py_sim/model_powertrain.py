
import numpy as np

# import package modules

from config import Ts
"""
Simulation model : Vehicle
==============================================================

Author
~~~~~~~~~~~~~
* kyunghan <kyunghah.min@gmail.com>

Description
~~~~~~~~~~~~~
* Powertrain class
  - Motor class
  - Drivetrain class
  - Battery class  

Powertrain <- Motor,Drivetrain,Battery (Composition relationship)


Motor <-> Drivetrain (Aggregation relationship)
Motor <-> Battery (Aggregation relationship)



Update
~~~~~~~~~~~~~
* [18/05/31] - Initial release - kyunghan
* [18/06/01] - Add battery class - Seungeon
* [18/06/01] - Modulization - kyuhwan
- Powertrain class -> Divide into motor, drivetrain, battery classes
- Powertrain class has composition relationship with motor, drivetrain, battery

-
"""

#%% Motor class

class Motor:    
    def __init__(self, M_Drivetrain,M_Battery):
        self.w_mot = 0
        self.t_mot = 0
        self.t_load = 0
        self.p_mot = 0
        self.Motor_config()        
        self.Ts_loc = Ts    
        self.Drivetrain = M_Drivetrain
        self.Battery = M_Battery
#        self.Drivetrain()
        
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
        
    def Motor_driven(self, v_in = 0, w_shaft = 0):
        # Elec motor model: Motor torque --> Mech motor model: Motor speed --> Drive shaft model: Load torque
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

        self.t_mot = self.Motor_elec_dynamics(self.t_mot, v_in, self.w_mot)
        self.w_mot = self.Motor_mech_dynamics(self.w_mot, self.t_mot, self.t_load)
        self.t_load = self.Drivetrain.Drive_shaft_dynamics(self.t_load, self.w_mot, w_shaft)
        self.p_mot = self.Motor_Power_system(self.t_mot,self.w_mot)
        self.Battery.Calc_Current(self.p_mot)
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
        w_mot =  w_mot + self.Ts_loc*(t_mot - t_load/self.Drivetrain.conf_rd_gear)/self.conf_jm_mot
        return w_mot
        
    def Motor_torque_system(self, t_mot_des):
        v_in = self.conf_trq_gain * t_mot_des
        return v_in
    
    
    def Motor_Power_system(self, t_mot, w_mot): # Seungon
        self.p_mot = t_mot * w_mot
        return self.p_mot

#%% Drivetrain class
        
class Drivetrain:  
    def __init__(self):
        global Ts

        self.Ts_loc = Ts
#        self.conf_rd_gear = 0
#        self.conf_ks_shaft = 0    
        self.DriveTrain_config()  
    
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




#%% Battery class

class Battery:
    def __init__(self):
        # Battery Internal state
        self.conf_Voc = 0  # Open circuit voltage [V]
        self.conf_Etot = 0  # Total energy-capacitor (vehicle specification) Q : 180[Ah] => E = QV => E = 180 * 3600 * 356 [J]
        self.conf_MaxPower = 0  # Allowable output power (vehicle specification) 150[kW]
        # Variable state (need for optimization)
        self.conf_R0 = 0  # internal resistor => regulate steady-state response
        self.conf_SOC_init = 0  # 70[%]
        # regulate short time dynamic reesponse ==> Tau_short_time = R1 * C1 [s]
        self.R1_a = 0;        self.R1_b = 0;        self.R1_c = 0
        self.R1 = 0  # internal resistor1
        self.C1_a = 0;        self.C1_b = 0;        self.C1_c = 0
        self.C1 = 0  # internal capacitor1
        # regulate long time dynamic reesponse ==> Tau_long_time = R2 * C2[s]
        self.R2_a = 0;        self.R2_b = 0;        self.R2_c = 0;
        self.R2 = 0  # internal resistor2
        self.C2_a = 0;        self.C2_b = 0;        self.C2_c = 0;
        self.C2 = 0  # internal capacitor2
        # Calculation state
        self.Voc_rate = 0; # ==> Voc = Voc(norminal) * Voc_rate
        self.Voc = 0
        self.R_tot = 0
        self.T_RC1 = 0
        self.T_RC2 = 0
        self.Consume_power = 0
        self.Internal_Energy = 0
        self.Temp=0
        # Output state
        self.Current = 0
        self.V_terminal = 0
        self.SOC = 0
        self.Battery_config()

    def Battery_config(self, Voc=356.0, Etot=230688000.0, MaxPower=150000.0, R0=0.02, SOC_init=70, R1_a=160.4, R1_b=-0.3, R1_c=47.0,
                       C1_a=-376.45, C1_b=-0.13, C1_c=351.8, R2_a=330.0, R2_b=-1.5, R2_c=49, C2_a=-6056, C2_b=-0.3, C2_c=4472, Int_E=161481600.0):
        self.conf_Voc = Voc
        self.conf_Etot = Etot
        self.conf_MaxPower = MaxPower
        self.conf_SOC_init = SOC_init;        self.SOC = SOC_init
        self.conf_R0 = R0
        self.R1_a = R1_a;        self.R1_b = R1_b;        self.R1_c = R1_c
        self.C1_a = C1_a;      self.C1_b = C1_b;       self.C1_c = C1_c
        self.R2_a = R2_a;        self.R2_b = R2_b;        self.R2_c = R2_c;
        self.C2_a = C2_a;        self.C2_b = C2_b;        self.C2_c = C2_c;
        self.Internal_Energy = Int_E

    def Calc_Voc_rate(self, input):
        SOC = [0,     10,     20,     30,     40,     50,     60,     70,     80,     90,     100]
        Voc_rate = [0.812,  0.9892, 1.0054, 1.0135, 1.0216, 1.0297, 1.0378, 1.0486, 1.0649, 1.0838,  1.1081]
        self.Voc_rate=np.interp(input,SOC,Voc_rate)
        return self.Voc_rate

    def Calc_Current(self, Motor_Net_Power, AccPower=500): # AccPower : Air conditionor's consum power and so on 50[W]
        self.Voc = self.conf_Voc * self.Calc_Voc_rate(self.SOC)

        self.R1 = self.R1_a * np.exp(self.R1_b * self.SOC) + self.R1_c
        self.C1 = self.C1_a * np.exp(self.C1_b * self.SOC) + self.C1_c
        self.R2 = self.R2_a * np.exp(self.R2_b * self.SOC) + self.R2_c
        self.C2 = self.C2_a * np.exp(self.C2_b * self.SOC) + self.C2_c

        self.R_tot = self.conf_R0 + 1/np.sqrt((1/self.R1)*(1/self.R1)+(self.C1)*(self.C1)) + 1/np.sqrt((1/self.R2)*(1/self.R2)+(self.C2)*(self.C2))
        self.Consume_power = AccPower + Motor_Net_Power

        self.Temp = self.Voc - np.abs(np.sqrt(self.Voc*self.Voc-4*self.R_tot*self.Consume_power))
        self.Current = self.Temp / (2 * self.R_tot)
        self.Calc_SOC()
        return self.Current

    def Calc_SOC(self, ):
        self.Internal_Energy = self.Internal_Energy - self.Voc * self.Current
        self.SOC = self.Internal_Energy / self.conf_Etot * 100
        self.V_terminal = self.Voc - self.Current * self.R_tot # V_terminal = V_open circuit - V_internal_decrease
        return self.SOC

    def Print_States(self,):
        self.T_RC1 = self.R1 * self.C1
        self.T_RC2 = self.R2 * self.C2
        return [self.V_terminal, self.Internal_Energy, self.R_tot, self.Voc]
#%% Powertrain class


class Mod_PowerTrain():
    def __init__(self):
        
        self.Drivetrain = Drivetrain()
        self.Battery = Battery()
        self.Motor = Motor(self.Drivetrain,self.Battery)
#PT = Mod_PowerTrain()
#
#C=PT.Drivetrain.conf_ks_shaft   
#PT.Motor.Drivetrain.conf_rd_gear =1000
#PT.Drivetrain.conf_rd_gear