import numpy as np
from constants import *


def ana_fd_scenarios_IDM(uf, gamma_AV, gamma_ACC, gamma_HV):

    v_e = np.arange(0, uf, 0.01)
    lc = 5  # Vehicle length

    s_e_AV = (S_0_AV + v_e * T_AV) / np.sqrt(1 - (v_e / V_0_AV) ** DELTA_AV)
    s_e_HV = (S_0_HV + v_e * T_HV) / np.sqrt(1 - (v_e / V_0_HV) ** DELTA_HV)
    s_e_ACC = (S_0_ACC + v_e * T_ACC) / np.sqrt(1 - (v_e / V_0_ACC) ** DELTA_ACC)

    #  Mixed autonomy fundamental diagram
    s_e = gamma_HV * s_e_HV + gamma_ACC * s_e_ACC + gamma_AV * s_e_AV          

    #  Human drivern fundamental diagram
    # s_e =  s_e_HV 


    # density and flow
    k_cg = 1 / (s_e + lc) * 1000  

    # fundamental diagram
    k_min = round(np.min(k_cg), 2)
    k_ff = np.arange(k_min, 0.0 ,-0.01)
    v_ff = np.full(k_ff.shape, uf)

    k_tot = np.concatenate((k_cg, k_ff))
    v_tot = np.concatenate((v_e, v_ff))
    flow_tot = k_tot * v_tot * 3.6

    # capacity and K_cr
    Q = np.max(flow_tot)    # Vehicles per hour
    ind = np.argmax(flow_tot)
    k_ff_max = np.max(k_tot)
    k_cr = k_tot[ind]

    k_tot = k_tot #    * 1.60934  # Convert from vehicles/km to vehicles/mi
    critDen = k_cr #    * 1.60934  # Convert critical density
    jamDen = k_ff_max #    * 1.6093  # Convert jam density
    backward_speed = Q / (jamDen - critDen)

    return k_tot, flow_tot, Q, k_ff_max, k_cr, critDen, jamDen, backward_speed

# _, _, Q,_, _,critical_density,jamdensity,_=ana_fd_scenarios_IDM(26.8224,0,0,1)
# print("FullHV"," Q, k_ff_max, k_cr, critDen, jamDen, backward_speed",ana_fd_scenarios_IDM(26.8224,0,0,1)[2:])
# print("FullAV","Q, k_ff_max, k_cr, critDen, jamDen, backward_speed",ana_fd_scenarios_IDM(26.8224,1,0,0)[2:])
# print("FullACC","Q, k_ff_max, k_cr, critDen, jamDen, backward_speed",ana_fd_scenarios_IDM(26.8224,0,1,0)[2:])
# print("Equal","Q, k_ff_max, k_cr, critDen, jamDen, backward_speed",ana_fd_scenarios_IDM(26.8224,1/3,1/3,1/3)[2:6])
# print("HighAV","Q, k_ff_max, k_cr, critDen, jamDen, backward_speed",ana_fd_scenarios_IDM(26.8224,0.2,0.2,0.6)[2:])