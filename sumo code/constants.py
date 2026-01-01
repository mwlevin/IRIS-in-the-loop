# v_0_AV = params_AV[0]       #Desired Speed Guess 20
# T_AV = params_AV[1]         #Desired Time-gap  2
# s_0_AV = params_AV[2]       #Minimum Gap
# delta_AV = params_AV[3]     #Acceleration Exponent
# a_AV = params_AV[4]         #max Acceleration rate
# b_AV = params_AV[5]         #Comfortable Deceleration

# Fundamental Diagram Parameters
V_0_ACC, T_ACC, S_0_ACC, DELTA_ACC, A_ACC, B_ACC = 44.1, 2.2, 6.3, 15.5, 0.6, 5.2
V_0_AV, T_AV, S_0_AV, DELTA_AV, A_AV, B_AV = 44.1, 1, 2.5, 4, 1, 0.5
V_0_HV, T_HV, S_0_HV, DELTA_HV, A_HV, B_HV = 44.1, 1.26, 3.4, 4, 1.06, 1.11

# Cell Transmission Model Parameters
DELTA_T = 6
MIN_RATE = 5  # in percentage