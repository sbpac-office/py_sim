# -*- coding: utf-8 -*-
"""
py_sim utilities
==============================================================

Author
~~~~~~~~~~~~~
* kyunghan <kyunghah.min@gmail.com>

Description
~~~~~~~~~~~~~
* Simulation utility modules
    * Calc_Radius: Calculation road radius and curvature
    * Filt_MovAvg: Moving average filter
    * Calc_PrDis: Calculation projection distance

Update
~~~~~~~~~~~~~
* [18/05/31] - Initial release - kyunghan
"""
# import python lib modules
from math import pi, sin, cos, sqrt, acos, atan
import numpy as np
from scipy.spatial import distance as dist_calc

def Calc_Radius (x_in, y_in, filt_num = 0):
    """Example function with PEP 484 type annotations.

    The return type must be duplicated in the docstring to comply
    with the NumPy docstring style.

    Parameters
    ----------
    param1
        The first parameter.
    param2
        The second parameter.

    Returns
    -------
    bool
        True if successful, False otherwise.

    """
    if len(x_in) != len(y_in):
        print('Radius calculation error: Data length must be same!')
        return
    else:
        x_c_out = np.zeros(len(x_in))
        y_c_out = np.zeros(len(x_in))
        R_out = np.zeros(len(x_in))
        circle_index = np.zeros(len(x_in))
        mr_o = np.zeros(len(x_in))
        mt_o = np.zeros(len(x_in))
        for i in range(len(x_in)-2):
            x_lst = x_in[i:i+3]; y_lst = y_in[i:i+3]
            x1 = x_lst[0]; x2 = x_lst[1]; x3 = x_lst[2]; y1 = y_lst[0]; y2 = y_lst[1]; y3 = y_lst[2]
            mr = (y2-y1)/(x2-x1);
            mt = (y3-y2)/(x3-x2);
            if np.equal(mr,mt) | np.isnan(mr) | np.isnan(mt):
                x_c = np.nan
                y_c = np.nan
                R = np.inf
                circle_on = 0
            else:
                # Radius calculation
                if mr >= abs(100):
                    x_c = (mt*(y3-y1)+(x2+x3))/2
                elif mt >= abs(100):
                    x_c = ((x1+x2) - mr*(y3-y1))/2
                else:
                    x_c = (mr*mt*(y3-y1)+mr*(x2+x3)-mt*(x1+x2))/(2*(mr-mt))
                if mr == 0:
                    y_c = -1/mt*(x_c-(x2+x3)/2)+(y2+y3)/2
                else:
                    y_c = -1/mr*(x_c-(x1+x2)/2)+(y1+y2)/2
                R = np.sqrt(np.square((x_c-x1))+np.square((y_c-y1)))
                circle_on = 1
            x_c_out[i] = x_c
            y_c_out[i] = y_c
            R_out[i] = R
            circle_index[i] = circle_on
            mr_o[i] = mr
            mt_o[i] = mt
        if filt_num !=0:
           R_out = Filt_MovAvg(R_out, filt_num)
        return [R_out, x_c_out, y_c_out, circle_index, mr_o, mt_o]

def Filt_MovAvg(Data, odd_filt_num):
    """
    #2. Moving average filt
    # Description: mean value filterring in moving window
    #              !!! moving window size should be odd number
    # Inputs -----------------------------------------------------------
    #   Data - Raw data
    #   odd_filt_num - Size of moving window
    # Outputs ----------------------------------------------------------
    #   Data_FiltOut - Filtering data
    """
    if len(Data) <= odd_filt_num:
        print('Moving average filter error: Data <= Filt Num')
        Data_FiltOut = 0
    elif odd_filt_num%2 == 0:
        print('Moving average filter error: Filt Num bust be odd number')
        Data_FiltOut = 0
    else:
        tmpFiltNum_c = np.int(odd_filt_num/2)
        Data_FiltOut = np.zeros(len(Data))
        Data_FiltOut[0:tmpFiltNum_c] = Data[0:tmpFiltNum_c]
        for i in range(tmpFiltNum_c,len(Data),1):
            Data_FiltOut[i] = np.mean(Data[i-tmpFiltNum_c:i-tmpFiltNum_c+odd_filt_num])
    return Data_FiltOut

def Calc_PrDis(Data_Array_x, Data_Array_y, Current_point):
    """
    Projection distance calculation
    Description: using for projection distance and direction between road and vehicle
    based on radial coordination
    Inputs
    -----------------------------------------------------------
    Data_Array_x - Road array X
    Data_Array_y - Road array Y
    Current_point - Vehicle current position [position x, position y]
    Outputs
    ----------------------------------------------------------
    tmp_s - Road length of projection distance
    tmp_n - Projection distance
    tmp_dir - Direction of road
    min_index - Minimum distance index of road data
    veh_an - Radian angle of vehicle position
    road_an - Radian angle of road
    """
    # Calculate distance array from current vehicle position and road data X, Y
    dis_array = np.sqrt(np.square(Data_Array_x - Current_point[0]) + np.square(Data_Array_y - Current_point[1]))
    # Find minimum distance point of road
    min_index = np.argmin(dis_array)
    if min_index >= (len(Data_Array_x)-1):
        # Road end condition
        print('========== Simulation is terminated!! ========= ')
        tmp_s = 0
        tmp_n = 0
        tmp_dir = 0
        veh_an = 0
        road_an = 0
    else:
        # tmp_a, tmp_b, tmp_c : distan
        tmp_c = dis_array[min_index]
        if dis_array[min_index-1] <= dis_array[min_index+1]:
            # Calculate distance
            tmp_b = dis_array[min_index-1]
            tmp_a = dist_calc.euclidean([Data_Array_x[min_index], Data_Array_y[min_index]], [Data_Array_x[min_index-1], Data_Array_y[min_index-1]])
            # Calculate road angle
            road_an = atan((Data_Array_y[min_index] - Data_Array_y[min_index-1])/(Data_Array_x[min_index] - Data_Array_x[min_index-1]))
            if Data_Array_x[min_index] - Data_Array_x[min_index-1] < 0:
                road_an = road_an + pi
            elif road_an < 0:
                road_an = road_an + 2*pi
            else:
                road_an = road_an
            # Calculate vehicle angle
            veh_an = atan((Current_point[1]-Data_Array_y[min_index-1])/(Current_point[0]-Data_Array_x[min_index-1]))
            if Current_point[0] - Data_Array_x[min_index-1] < 0:
                veh_an = veh_an + pi
            elif veh_an < 0:
                veh_an = veh_an + 2*pi
            else:
                veh_an = veh_an
        else:
            # Calculate distance
            tmp_b = dis_array[min_index+1]
            tmp_a = dist_calc.euclidean([Data_Array_x[min_index], Data_Array_y[min_index]], [Data_Array_x[min_index+1], Data_Array_y[min_index+1]])
            # Calculate road angle
            road_an = atan((Data_Array_y[min_index+1] - Data_Array_y[min_index])/(Data_Array_x[min_index+1] - Data_Array_x[min_index]))
            if Data_Array_x[min_index+1] - Data_Array_x[min_index] < 0:
                road_an = road_an + pi
            elif road_an < 0:
                road_an = road_an + 2*pi
            else:
                road_an = road_an
            # Calculate vehicle angle
            veh_an = atan((Current_point[1]-Data_Array_y[min_index])/(Current_point[0]-Data_Array_x[min_index]))
            if Current_point[0] - Data_Array_x[min_index] < 0:
                veh_an = veh_an + pi
            elif veh_an < 0:
                veh_an = veh_an + 2*pi
            else:
                veh_an = veh_an
        # Calculate projection distance using 2nd law of cosin
        if (tmp_a**2 + tmp_c**2 - tmp_b**2)/(2*tmp_a*tmp_c) >= 1:
            tmp_dem = 1
        elif (tmp_a**2 + tmp_c**2 - tmp_b**2)/(2*tmp_a*tmp_c) <= -1:
            tmp_dem = -1
        else:
            tmp_dem = (tmp_a**2 + tmp_c**2 - tmp_b**2)/(2*tmp_a*tmp_c)
        if tmp_c == 0:
            tmp_An = 0
        else:
            tmp_An = acos(tmp_dem)
        # Determine projection distance and road length
        tmp_n = sin(tmp_An)*tmp_c
        tmp_s = cos(tmp_An)*tmp_c
        # Determine road direction - calculation angular difference (vehicle, road)
        an_dif = veh_an - road_an
        if an_dif >= 0 and an_dif < pi:
            # Road exist at right position of vehicle
            tmp_dir = 'Right'
        else:
            tmp_dir = 'Left'
    # Return value
    return tmp_s, tmp_n, tmp_dir, min_index, veh_an, road_an
#%%  ----- test ground -----
if __name__ == "__main__":
    pass
