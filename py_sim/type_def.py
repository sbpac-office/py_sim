# -*- coding: utf-8 -*-
"""
py_sim type definition
==============================================================

Author
~~~~~~~~~~~~~
* kyunghan <kyunghah.min@gmail.com>

Description
~~~~~~~~~~~~~
* Class set for type definition
    * type_pid_controller
    * type_drvstate
    * type_objective
    * type_DataLog

Update
~~~~~~~~~~~~~
* [18/05/31] - Initial release - kyunghan
"""
# Config variable
Ts = 0.01
class type_pid_controller:
    def __init__(self, P_gain = 1, I_gain = 1, D_gain = 'None', Ts = 'None', I_val_old = 0, Error_old = 0):
        self.P_gain = P_gain
        self.I_gain = I_gain
        self.D_gain = D_gain
        self.I_val_old = I_val_old
        self.Error_old = Error_old
        if Ts == 'None':
             self.Ts_loc = globals()['Ts']
        else:
             self.Ts_loc = Ts

    def Control(self,set_value,cur_value):
        self.Error = set_value - cur_value
        self.P_val = self.P_gain * self.Error
        self.I_val = self.I_gain * self.Error *self.Ts_loc + self.I_val_old
        self.I_val_old = self.I_val
        self.D_val = self.D_gain * (self.Error_old - self.Error)/self.Ts_loc
        Control_out = self.P_val + self.I_val + self.D_val
        return Control_out

class type_objective:
    def __init__(self):
        self.object_class = []
        self.object_param = []
        self.object_loc_s = []
    def add_object(self,object_class_in = 'None', object_param_in = 'None', object_loc_in = 0):
        self.object_class.append(object_class_in)
        self.object_param.append(object_param_in)
        self.object_loc_s.append(object_loc_in)
    def merg_object(self,object_class_in = 'None', object_param_in = 'None', object_loc_in = 0):
        self.object_class = self.object_class + object_class_in
        self.object_param = self.object_param + object_param_in
        self.object_loc_s = self.object_loc_s + object_loc_in

class type_drvstate:
    def __init__(self):
        self.state = []
        self.state_param = []
        self.state_reldis = []
    def add_state(self, state_in = 'None', state_param_in = 'None', state_reldis_in = 0):
        self.state.append(state_in)
        self.state_param.append(state_param_in)
        self.state_reldis.append(state_reldis_in)
    def set_state(self, state_in = 'None', state_param_in = 'None', state_reldis_in = 0):
        self.state = state_in
        self.state_param = state_param_in
        self.state_reldis = state_reldis_in

class type_DataLog:
    def __init__(self, NameSpaceList = []):
        if len(NameSpaceList) == 0:
            print('Should set name space : SetNameSpace(List)')
            self.NameSet = NameSpaceList
            self.DataProfile = {}
        else:
            self.NameSet = NameSpaceList
            self.DataProfile = {}
            for i in NameSpaceList:
                self.DataProfile.update({i:[]})

    def SetNameSpace(self, NameSpaceList):
        self.NameSet = NameSpaceList
        for i in NameSpaceList:
            self.DataProfile.update({i:[]})

    def StoreData(self, DataSet):
        if len(self.NameSet) != len(DataSet):
            print('Data length should same to Name set length')
        else:
            for i in range(len(self.NameSet)):
                self.DataProfile[self.NameSet[i]].append(DataSet[i])
    def get_profile_value(self, get_name_set = []):
        if len(get_name_set) == 0:
            get_name_set = self.NameSet
        return_profile_list = []
        for i in get_name_set:
            return_profile_list.append(self.DataProfile[i])
        return return_profile_list
    def get_profile_value_one(self, get_name_set):
        return self.DataProfile[get_name_set]

#%%  ----- test ground -----
if __name__ == "__main__":
    pass
