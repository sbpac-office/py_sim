
# import package modules
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
* Powertrain <- Motor,Battery (Composition relationship)
* Motor <-> Battery (Aggregation relationship)

Update
~~~~~~~~~~~~~
* [18/05/31] - Initial release - kyunghan
* [18/06/01] - Add battery class - seungeon
* [18/06/01] - Modulization - kyuhwan
  - Powertrain class -> Divide into motor, battery classes
  - Powertrain class has composition relationship with motor, battery
* [18/06/11] - Modification - kyunghan
  - Modify drivetrain module to body module
  - Modify model configurations
"""
# simulation sampling time
Ts = 0.01
"""global vairable: simulation sampling timeself.

you can declare other sampling time in application as vairable ``Ts``

"""
import numpy as np
from sub_type_def import type_pid_controller
#%% Motor class
class Motor:
    def __init__(self, M_Battery):
        self.v_mot = 320
        self.i_mot = 0
        self.w_mot = 0
        self.t_mot = 0
        self.p_mot_mech = 0
        self.p_mot_loss = 0
        self.p_mot_elec = 0
        self.Motor_config()
        self.Ts_loc = Ts
        self.Battery = M_Battery

    def Motor_config(self, loss_mech_C0 = 0.0206, loss_mech_C1 = -2.135e-5, loss_copper_C0 = 0.2034, loss_stray_C0 = 3.352e-6, loss_stray_C1 = -2.612e-9, loss_iron_C0 = 1e-6, loss_iron_C1 = 1.55):
        """Motor parameter configuration

        Parameters not specified are declared as default values

        If you want set a specific parameter don't use this function,
        just type::

            >>> Mod_PowerTrain.conf_rm_mot = 0.2
            ...

        Args:
            Motor parameter values, default values are setted
        """

        self.loss_mech_C0 = loss_mech_C0
        self.loss_mech_C1 = loss_mech_C1
        self.loss_copper_C0 = loss_copper_C0
        self.loss_stray_C0 = loss_stray_C0
        self.loss_stray_C1 = loss_stray_C1
        self.loss_iron_C0 = loss_iron_C0
        self.loss_iron_C1 = loss_iron_C1

    def Motor_driven(self, torque_set = 0, w_drivtrain = 0):
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
        self.t_mot = torque_set
        self.w_mot = w_drivtrain
        p_mot_elec = self.Motor_Power_system(self.t_mot,self.w_mot)
        self.Battery.Calc_Current(p_mot_elec)
        return self.w_mot, self.t_mot

    def Motor_Power_system(self, t_mot, w_mot):
        self.p_mot_mech = t_mot * w_mot
        w_mot = sorted([0,w_mot,10000])[1]
        self.p_mot_loss = (self.loss_mech_C0 + self.loss_mech_C1*w_mot)*w_mot**2 + self.loss_copper_C0*t_mot + (self.loss_stray_C0+self.loss_stray_C1*w_mot)*w_mot**2*t_mot**2 + self.loss_iron_C0*w_mot**self.loss_iron_C1*t_mot**2
        self.p_mot_elec = self.p_mot_mech + self.p_mot_loss
        self.Motor_Elec_system(self.p_mot_elec)
        return self.p_mot_elec

    def Motor_Elec_system(self, p_mot_elec):
        self.i_mot = p_mot_elec / self.v_mot
        return self.i_mot

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
        self.V1 = 0
        self.V2 = 0
        self.Consume_power = 0
        self.Internal_Energy = 0
        # Output state
        self.Current = 0
        self.V_terminal = 0
        self.SOC = 0
        self.Battery_config()

    def Battery_config(self, Voc=356.0, Etot=230400000.0, MaxPower=150000.0, R0=0.0016, SOC_init=70, R1_a=76.5218, R1_b=-7.9563, R1_c=23.8375,
                       C1_a=-649.8350, C1_b=-64.2924, C1_c=12692.1946, R2_a=5.2092, R2_b=-35.2367, R2_c=124.9467, C2_a=-78409.2788,
                       C2_b=-0.0131, C2_c=30802.62582, Int_E=161280000.0, V1 = 0, V2 = 0):
        self.conf_Voc = Voc
        self.conf_Etot = Etot
        self.conf_MaxPower = MaxPower
        self.conf_SOC_init = SOC_init;        self.SOC = SOC_init
        self.conf_R0 = R0
        self.R1_a = R1_a;        self.R1_b = R1_b;        self.R1_c = R1_c
        self.C1_a = C1_a;      self.C1_b = C1_b;       self.C1_c = C1_c
        self.R2_a = R2_a;        self.R2_b = R2_b;        self.R2_c = R2_c;
        self.C2_a = C2_a;        self.C2_b = C2_b;        self.C2_c = C2_c;
        self.V1 = V1; self.V2 = V2;
        self.Internal_Energy = Int_E

    def Calc_Voc_rate(self, input):
        SOC = [0, 6.25,  31.25,  62.5,   93.75, 100.00]
        Voc_rate = [0, 0.625, 0.8125,  0.875,  1.00,   1.0915]
        self.Voc_rate=np.interp(input,SOC,Voc_rate)
        return self.Voc_rate

    def Calc_Current(self, Motor_Net_Power, AccPower=500): # AccPower : Air conditionor's consum power and so on 50[W]
        self.Voc = self.conf_Voc * self.Calc_Voc_rate(self.SOC)

        self.R1 = self.R1_a * np.exp(self.R1_b * self.SOC) + self.R1_c
        self.C1 = self.C1_a * np.exp(self.C1_b * self.SOC) + self.C1_c
        self.R2 = self.R2_a * np.exp(self.R2_b * self.SOC) + self.R2_c
        self.C2 = self.C2_a * np.exp(self.C2_b * self.SOC) + self.C2_c

        self.Consume_power = AccPower + Motor_Net_Power

        self.Current = (self.Voc - np.sqrt(self.Voc**2-4*self.conf_R0*self.Consume_power)) / (2 * self.conf_R0)
        self.Calc_SOC()
        return self.Current

    def Calc_SOC(self, ):
        self.V_terminal = self.Voc - self.Current*self.conf_R0 - self.V1 - self.V2
        self.V1 = self.V1 + (self.Current/self.C1 - self.V1/(self.R1*self.C1))*Ts
        self.V2 = self.V2 + (self.Current/self.C2 - self.V2/(self.R2*self.C2))*Ts
        self.Internal_Energy = self.Internal_Energy - self.Voc * self.Current * Ts
        self.SOC = self.Internal_Energy / self.conf_Etot * 100

        return self.SOC

    def Print_States(self,):
        return [self.V_terminal, self.Internal_Energy,  self.Voc]

#%% Powertrain class
class Mod_Power():
    def __init__(self):
        self.ModBattery = Battery()
        self.ModMotor = Motor(self.ModBattery)
