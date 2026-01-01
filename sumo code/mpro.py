import traci
import sumolib
import numpy as np

from constants import DELTA_T, MIN_RATE
from fd import ana_fd_scenarios_IDM as fd_values


# def get_cell_ends(lane):
#     cell_length = round(traci.lane.getMaxSpeed(lane) * DELTA_T)
#     edge_length = round(traci.lane.getLength(lane))
#     return list(range(cell_length, edge_length, cell_length)) + [edge_length]

def get_cell_ends(edge):
    cell_length = round(edge.getSpeed() * DELTA_T)
    edge_length = round(edge.getLength())
    return list(range(cell_length, edge_length, cell_length)) + [edge_length]

def cell_number(veh, cell_length, edge) -> int:
    """
    Fix: Based on whether the edge is an upstream edge start counting cells from the start or end of the edge.
    """
    position = traci.vehicle.getLanePosition(vehID=veh)
    if len(edge.getIncoming()) >= 2:
        return int(position // cell_length)
    else:
        length = edge.getLength() 
        position = length - position
        cell = int(position // cell_length)
        maximum_cells = int(length // cell_length)
        return maximum_cells - cell

    # cell_ends = get_cell_ends(lane)
    # for i, end_pos in enumerate(cell_ends):
    #     if position < end_pos:
    #         return i
    # return len(cell_ends)

def small_w(edge, cell: int, upstream: bool = False) -> float:
    # lane = edge.getID() + "_0"
    cell_ends = get_cell_ends(edge)
    cell_length = cell_ends[0]

    p = cell_ends[cell if upstream else -cell-1]
    q = p - cell_length
    r = cell_length
    return ((p + q) * r) / (2 * cell_length)

def vehicles_per_cell(edge) -> dict:
    cell_length = round(edge.getSpeed() * DELTA_T)
    edge_length = round(edge.getLength())
    cell_vehicles = [0, ] * int(np.ceil(edge_length / cell_length))
    for vehicle in traci.edge.getLastStepVehicleIDs(edgeID=edge.getID()):
        veh_cell = cell_number(vehicle, cell_length, edge)
        cell_vehicles[veh_cell] += 1
    return cell_vehicles

def get_number_of_vehicles_in_edge_cell(edge, cell):
    return vehicles_per_cell(edge=edge)[cell]

def edge_density(a):
    vehicles_in_edge = vehicles_per_cell(a)
    cell_ends = get_cell_ends(a)
    cell_end_diffs = [cell_ends[0], ] + [cell_ends[i] - cell_ends[i-1] for i in range(len(cell_ends))]
    densities = [abs(veh / dx) for veh, dx in zip(vehicles_in_edge, cell_end_diffs)]
    return densities

def weight(a, b):
    c_ab = 1
    k_ab = edge_density(a)
    cell_ends = get_cell_ends(a)
    tk_upstream = c_ab * sum(k_ab[i] * small_w(a, i, upstream=True) for i in range(len(cell_ends)))        # total density : tk_upstream

    k_b_end = edge_density(b)
    c_b_end = 1
    cell_ends = get_cell_ends(b)
    tk_downstream = c_b_end * sum(k_b_end[i] * small_w(b, i) for i in range(len(cell_ends)))

    return abs(tk_upstream - tk_downstream)

def rampnode_in_mainline(rampnode):
    
    assert len(rampnode.getOutgoing()) == 1, "Multiple rampnodes in mainline."
    return rampnode.getOutgoing()[0].getToNode()

def get_ramp_mainup_maindown(rampnode):
    mrampnode = rampnode_in_mainline(rampnode)
    r = rampnode.getIncoming()[0]
    rEnd = rampnode.getOutgoing()
    
    for inEdge in mrampnode.getIncoming():
        if inEdge != rEnd:
            i = inEdge
    
    j = mrampnode.getOutgoing()[0]
    return r, i, j

def get_qmax(edge):
    yav, yhv, yacc = getGammas(edge=edge)
    uf = edge.getSpeed()
    _, _, Q, *_ = fd_values(uf=uf, gamma_AV=yav, gamma_ACC=yacc, gamma_HV=yhv)
    return Q / 3600 * edge.getLaneNumber()

def getGammas(edge):
    delta_x = edge.getSpeed() * DELTA_T
    av, hv, acc = 0, 0, 0
    for vehicle in traci.edge.getLastStepVehicleIDs(edge.getID()):
        vehicle_str = str(vehicle).lower()
        if "acc" in vehicle_str:
            acc += 1
        elif "av" in vehicle_str:
            av += 1
        else:
            hv += 1

    total = av + hv + acc
    if total == 0:
        return 1/3, 1/3, 1/3    # If no vehicles, return equal weights

    return av / total, hv / total, acc / total

def get_receiving_flow_mainline_downstream(j):
    yav, yhv, yacc = getGammas(edge=j)
    uf = j.getSpeed()
    _, _, Q, *_, jamDen, w = fd_values(uf=uf, gamma_AV=yav, gamma_ACC=yacc, gamma_HV=yhv)
    jamDensity_fixed_unit = jamDen / 1000 # vehicles per meter

    print("\t\torig Q", Q, "jamden ", jamDen)
    Q = Q / 3600 # vehicles per second
    N = jamDensity_fixed_unit * uf * DELTA_T
    
    # Number of vehicles in the first cell of the edge j
    n0_j = min(N, get_number_of_vehicles_in_edge_cell(edge=j, cell=0))  # Hack figure out whats wrong

    print("\t\tR calc Q=", Q, "lanes=", j.getLaneNumber(), j.getID(), "delta T=", DELTA_T, "w=", w, "uf=", uf, "N=", N)
    # Receiving flow for j
    # is this a bug? should have number of lanes in 2nd term
    return min(Q * j.getLaneNumber() * DELTA_T, w / uf * (N - n0_j))
    #return min(Q * j.getLaneNumber() * DELTA_T, w / uf * (N*j.getLaneNumber() - n0_j))


def Minimum_rate_calc(rampnode):
    incoming_edges = [edge.getID() for edge in rampnode.getIncoming()]  
    outgoing_edges = [edge.getID() for edge in rampnode.getOutgoing()] 

    # print("Incoming Edges:", incoming_edges)
    # print("Outgoing Edges:", outgoing_edges)

    arrived_veh = sum(traci.edge.getLastStepVehicleNumber(edge) for edge in incoming_edges)
    departed_veh = sum(traci.edge.getLastStepVehicleNumber(edge) for edge in outgoing_edges)

    ramp_occupancy = sum(traci.edge.getLastStepVehicleNumber(edge) for edge in incoming_edges) if incoming_edges else 1
    # MIN_RATE = 0.7 * (arrived_veh - departed_veh) / max(ramp_occupancy, 1)
    MIN_RATE = 0.7 * (arrived_veh - departed_veh) / max(arrived_veh, 1)   
    # MIN_RATE = 0.3 * (arrived_veh - departed_veh) / max(arrived_veh, 1)   
    # MIN_RATE = min(0.25, (0.3 * (arrived_veh - departed_veh) / max(arrived_veh, 1)))

    return max(MIN_RATE,0)



def optimal_control(rampnode):
    delta_ij = 0
    i, i_prime, j = get_ramp_mainup_maindown(rampnode)

    rj = get_receiving_flow_mainline_downstream(j)
    
    # Ramp link, last cell vehicles
    nC_i = get_number_of_vehicles_in_edge_cell(edge=i, cell=-1)
    qi_max = get_qmax(i)
    si = min(nC_i, qi_max * DELTA_T)
    
    # Main line link, last cell vehicles
    nC_iprime = get_number_of_vehicles_in_edge_cell(edge=i_prime, cell=-1)
    qi_prime_max = get_qmax(i_prime)
    si_prime = min(nC_iprime, qi_prime_max * DELTA_T)
    
    min_rate = int(Minimum_rate_calc(rampnode) * 100)
    
    print("\tminimum rate is:",min_rate)
    print("\tramp weight is", weight(i, j), "mainline weight is", weight(i_prime, j))
    print("\tramp S is", si, "mainline S is", si_prime, "R is", rj, "ramp Q is", qi_max)

    if si + si_prime < rj:
        #return 1    # Metering is not needed
        return -1
    # if si_prime >= rj:
    #     return 0.2

    
    
    
    
    best_objective = - float("inf")
    optimal_delta_ij = min_rate / 100

    for delta_ij in range(min_rate, 100):
        delta_ij = delta_ij / 100

        si = min(nC_i, delta_ij * qi_max * DELTA_T)
        si_prime = min(nC_iprime, qi_prime_max * DELTA_T)

        lambda_1 = (delta_ij * qi_max / (delta_ij * qi_max + qi_prime_max)) * rj
        lambda_2 = rj - si_prime
        lambda_3 = si

        y_i_j = np.median([lambda_1, lambda_2, lambda_3])
        y_iprime_j = rj - y_i_j

        objective_value = weight(i, j) * y_i_j + weight(i_prime, j) * y_iprime_j
        if objective_value >= best_objective:
            best_objective = objective_value
            optimal_delta_ij = delta_ij
    print("optimal_delta_ij:",(i,j),optimal_delta_ij)

    return optimal_delta_ij


