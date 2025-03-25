# Initialization copied from SUMO documentation
from __future__ import absolute_import
from __future__ import print_function
import os
import sys
import optparse
'''
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
    
'''


from sumolib import checkBinary
import traci
def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


# Import Functions used 
from densityFunctions_3 import computeRed, computeSwitchMeter,calcRateLimits, computeRedFlush, computeSwitchFlush, computeSwitchMeterStopped

# Define function to run script
# Control: 1 (meter on) 2 (meter off)
# critDensity and jamDensity: Critical and Jam Densities used in algorithm
# rampStorageLength: Length of ramp that can be used for queue storage (upstream of meter, depends on site)

def run(control,critDensity,jamDensity,rampStorageLength,alpha_desired,alpha_low):
    
    # Define Constants and Initialization
    stepsize = .05
    step = 0

    
    phase = 'notStarted' # Initial Phase
    phase1 = 'notStarted' # Initial Phase
    phase2 = 'notStarted' # Initial Phase


    endStep = 72000 # Time to simulate (0.05 seconds steps)
    # Define the list of update steps: Every 30 seconds we check for updates (meter phase, meter rates, etc..)
    update_time_1 = 30 # Updates for meter rate every 30 s
    update_steps_1 = list(range(int(update_time_1/stepsize),int(1000000/stepsize),int(update_time_1/stepsize)))
    
    # Define values we want to keep a running tally of - entires added every 30 seconds
  
    up_occ_noStart = [] # Upstream avg Mainline Occupancy during not started phase
    up1_occ_noStart = [] # Upstream (before the first merge) avg Mainline Occupancy during not started phase
    up2_occ_noStart = [] # Upstream (after the first merge) avg Mainline Occupancy during not started phase

    down_occ_noStart = [] # Downstream avg Mainline Occupancy during not started phase
    down1_occ_noStart = [] # Downstream (after the first merge) avg Mainline Occupancy during not started phase
    down2_occ_noStart = [] # Downstream (after the second merge) avg Mainline Occupancy during not started phase

    up_occ_meter = [] #Upstream avg Mainline Occupancy during meter phase
    up1_occ_meter = [] #Upstream (before the first merge) avg Mainline Occupancy during meter phase
    up2_occ_meter = [] #Upstream (after the first merge) avg Mainline Occupancy during meter phase

    down_occ_meter = [] # Downstream avg Mainline Occupancy during meter phase
    down1_occ_meter = [] # Downstream (after the first merge) avg Mainline Occupancy during meter phase
    down2_occ_meter = [] # Downstream (after the second merge) avg Mainline Occupancy during meter phase

    up_occ_stopped = [] #Upstream avg Mainline Occupancy during stopped phase
    up1_occ_stopped = [] #Upstream (before the first merge) avg Mainline Occupancy during stopped phase
    up2_occ_stopped = [] #Upstream (before the second merge) avg Mainline Occupancy during stopped phase


    down_occ_stopped = [] # Downstream avg Mainline Occupancy during stopped phase
    down1_occ_stopped = [] # Downstream (after the first merge) avg Mainline Occupancy during stopped phase
    down2_occ_stopped = [] # Downstream (after the second merge) avg Mainline Occupancy during stopped phase

    meterActivate = [] # Timesteps Meter Activated
    meter1Activate = [] # Timesteps Meter 1 Activated
    meter2Activate = [] # Timesteps Meter 2 Activated

    flushActivate = [] # Timesteps Flush Activated
    flush1Activate = [] # Timesteps Flush 1 Activated
    flush2Activate = [] # Timesteps Flush 2 Activated

    stoppedActivate = [] # Timesteps Stopped Phase Activated
    stopped1Activate = [] # Timesteps Stopped Phase Activated for meter 1
    stopped2Activate = [] # Timesteps Stopped Phase Activated for meter 2

    minRates = [0] # Min,max, and release rates throughout the simulation
    min1Rates = [0] # Min,max, and release rates throughout the simulation for meter 1
    min2Rates = [0] # Min,max, and release rates throughout the simulation for meter 2

    maxRates = [0]
    max1Rates = [0]
    max2Rates = [0]

    rates = [0]
    rates1 = [0]
    rates2 = [0]

    passage_vehIds = [] # Veh Ids of all passage veh
    passage1_vehIds = [] # Veh Ids of all passage veh from ramp 1
    passage2_vehIds = [] # Veh Ids of all passage veh from ramp 2


    passage_count = [0] # Accumulated vehilces entering the ramp
    passage1_count = [0] # Accumulated vehilces entering the ramp 1
    passage2_count = [0] # Accumulated vehilces entering the ramp 2

    demand_vehIds = [] # Veh Ids of all demand veh
    demand1_vehIds = [] # Veh Ids of all demand veh ramp 1
    demand2_vehIds = [] # Veh Ids of all demand veh ramp 2

    demand_count = [0] # Accumulated vehicles exiting the ramp
    demand1_count = [0] # Accumulated vehicles exiting the ramp 1
    demand2_count = [0] # Accumulated vehicles exiting the ramp 2

    
    # START SIMULATION, RUN FOR SET TIME 
    while step <= endStep:
        # 1. NOT STARTED PHASE - All green, Switch to meter when mainline has high enough density
        if phase == 'notStarted':
            print('Not Started Phase Activated')
        while phase == 'notStarted' and step <= endStep:
            traci.lane.setDisallowed('rampStart_0',['passenger']) # Only 1 lane used when meter off
            traci.lane.setDisallowed('ramp2Start_0',['passenger']) 

            traci.trafficlight.setPhase('J3',0) # Set phase ramp 1 to all green
            traci.trafficlight.setPhase('J9',0) # Set phase ramp 2 to all green

            traci.simulationStep() # Progress Sim by 1 time step
            step += 1
            
            # Turn ramp HVs to ACCs
            for vehicle_id in traci.vehicle.getIDList():
                if traci.vehicle.getTypeID(vehicle_id) == "HV_R_ACC":
                    lane_id = traci.vehicle.getLaneID(vehicle_id)
                    if lane_id == "switchACC_0" or lane_id == 'switchACC_1':
                        traci.vehicle.setType(vehicle_id, "ACC_AMax_R")

                        # --------------------------
                    elif lane_id == "endSeg_0" or lane_id == 'endSeg_1':
                        traci.vehicle.setType(vehicle_id, "ACC_AMax_R")
            
            if step in update_steps_1: #Check if meter should be turned on (check every 30s with 2 min averages)
                
                #Store Rate info - When off just store as 0 (ramp 1)
                min1Rates.append(0)
                max1Rates.append(0)
                rates1.append(0)
                
                # ramp 2
                min2Rates.append(0)
                max2Rates.append(0)
                rates2.append(0)
                
                
                # Get Queue info and store (ramp 1)
                new_vehIds = traci.inductionloop.getLastIntervalVehicleIDs('detPass2')
                passage1_vehIds += [string for string in new_vehIds if string not in passage1_vehIds]
                passage1_count.append(len(passage1_vehIds))
                new_vehIdsA = traci.inductionloop.getLastIntervalVehicleIDs('detDemandA')
                new_vehIdsB = traci.inductionloop.getLastIntervalVehicleIDs('detDemandB')
                demand1_vehIds += [string for string in new_vehIdsA if string not in demand1_vehIds]
                demand1_vehIds += [string for string in new_vehIdsB if string not in demand1_vehIds]
                demand1_count.append(len(demand1_vehIds))

                # Get Queue info and store (ramp 2)
                new_vehIds2 = traci.inductionloop.getLastIntervalVehicleIDs('detPass4')
                passage2_vehIds += [string for string in new_vehIds2 if string not in passage2_vehIds]
                passage2_count.append(len(passage2_vehIds))
                new_vehIdsC = traci.inductionloop.getLastIntervalVehicleIDs('detDemandC')
                new_vehIdsD = traci.inductionloop.getLastIntervalVehicleIDs('detDemandD')
                demand2_vehIds += [string for string in new_vehIdsC if string not in demand2_vehIds]
                demand2_vehIds += [string for string in new_vehIdsD if string not in demand2_vehIds]
                demand2_count.append(len(demand2_vehIds))
                
                # Get Occupancy Info and Store
                upOcc = (traci.inductionloop.getLastIntervalOccupancy('detUp1')+traci.inductionloop.getLastIntervalOccupancy('detUp2'))/2
                downOcc = (traci.inductionloop.getLastIntervalOccupancy('detDown1')+traci.inductionloop.getLastIntervalOccupancy('detDown2'))/2
                up_occ_noStart.append(upOcc)
                down_occ_noStart.append(downOcc)
               
                # Compute Switch Condition
                switch = computeSwitchMeter(up_occ_noStart, down_occ_noStart,critDensity,alpha_desired,alpha_low)
                if switch == 'True':
                    
                    # Get initial rate for meter 1 phase [veh/hr]
                    lastRate = traci.inductionloop.getLastIntervalVehicleNumber('detPass1') # 90 s veh count
                    lastRate = lastRate * 40 # veh/hr
                    print('Initial Rate [veh/hr]:', lastRate)
                    
                    # Define initial red and green times for meter phase
                    alternate = 0 # Alternate between left and right lane for meter (0 = left 1 = right)
                    g = 3 # Green time (constant, allows 1 veh to pass)
                    r = round((3600 / lastRate) - g) # Calc from rate value
                    print('Initial Red Time:', r)

                    rstart = 0 # These start/end times are used to make meter switch between red and green (hard to explain, see presentation for detatils)
                    rend = rstart + r # While it seems the current step wont fall within these bounds. The bounds will continously update in the next while loop until they are within range of the step. (add print statments to prove)
                    gstart = rend 
                    gend = gstart + g

                                        
                    # Get initial rate for meter 2 phase [veh/hr]
                    lastRate2 = traci.inductionloop.getLastIntervalVehicleNumber('detPass3') # 90 s veh count
                    lastRate2 = lastRate2 * 40 # veh/hr
                    print('Initial Rate [veh/hr]:', lastRate)
                    
                    # Define initial red and green times for meter phase
                    alternate2 = 0 # Alternate between left and right lane for meter (0 = left 1 = right)
                    g2 = 3 # Green time (constant, allows 1 veh to pass)
                    r2 = round((3600 / lastRate2) - g) # Calc from rate value
                    print('Initial Red Time:', r2)

                    rstart2 = 0 # These start/end times are used to make meter switch between red and green (hard to explain, see presentation for detatils)
                    rend2 = rstart2 + r2 # While it seems the current step wont fall within these bounds. The bounds will continously update in the next while loop until they are within range of the step. (add print statments to prove)
                    gstart2 = rend2 
                    gend2 = gstart2 + g2
                               
                
                    # Switch the phase to meter
                    if control == 1: # If we test scenario with no meter (control=2) just keep meter off
                        phase = 'meter'
                        print('Meter Phase Activated!', 'Time[s]:', step*stepsize) # Print time meter turns on. Important so we can know when to start "counting" travel time
                        meter1Activate.append(step)
                        
                        meter2Activate.append(step)

        
        # 2. METER PHASE - Switch to flush if mainline density is low enough
        while phase == 'meter' and step <= endStep:
            
            traci.lane.setDisallowed('rampStart_0',[]) # Now 2 lines form for meter 1
            
            traci.lane.setDisallowed('ramp2Start_0',[]) # Now 2 lines form for meter 2

                        
            # RED
            while step <= rend/stepsize and step >= rstart/stepsize: # If time step in red range, set to red
                traci.trafficlight.setPhase('J3',1) # Set phase to all red on ramp 1
                traci.trafficlight.setPhase('J9',1) # Set phase to all red on ramp 2
                traci.simulationStep() 
                step += 1
                
                # Turn ramp HVs to ACCs
                for vehicle_id in traci.vehicle.getIDList():
                    if traci.vehicle.getTypeID(vehicle_id) == "HV_R_ACC":
                        lane_id = traci.vehicle.getLaneID(vehicle_id)
                        if lane_id == "switchACC_0" or lane_id == 'switchACC_1':
                            traci.vehicle.setType(vehicle_id, "ACC_AMax_R")
                        elif lane_id == "endSeg_0" or lane_id == 'endSeg_1':
                            traci.vehicle.setType(vehicle_id, "ACC_AMax_R")
                
                if step in update_steps_1: # If its a 30s interval, update all detector counts and calcuate redtime
                    
                    # Get Queue info and store for ramp 1
                    new_vehIds = traci.inductionloop.getLastIntervalVehicleIDs('detPass2')
                    passage1_vehIds += [string for string in new_vehIds if string not in passage1_vehIds]
                    passage1_count.append(len(passage1_vehIds))
                    new_vehIdsA = traci.inductionloop.getLastIntervalVehicleIDs('detDemandA')
                    new_vehIdsB = traci.inductionloop.getLastIntervalVehicleIDs('detDemandB')
                    demand1_vehIds += [string for string in new_vehIdsA if string not in demand1_vehIds]
                    demand1_vehIds += [string for string in new_vehIdsB if string not in demand1_vehIds]
                    demand1_count.append(len(demand1_vehIds))
                    
                    # Get Queue info and store for ramp 2
                    new_vehIds2 = traci.inductionloop.getLastIntervalVehicleIDs('detPass4')
                    passage2_vehIds += [string for string in new_vehIds2 if string not in passage2_vehIds]
                    passage2_count.append(len(passage2_vehIds))
                    new_vehIdsC = traci.inductionloop.getLastIntervalVehicleIDs('detDemandC')
                    new_vehIdsD = traci.inductionloop.getLastIntervalVehicleIDs('detDemandD')
                    demand2_vehIds += [string for string in new_vehIdsC if string not in demand2_vehIds]
                    demand2_vehIds += [string for string in new_vehIdsD if string not in demand2_vehIds]
                    demand2_count.append(len(demand2_vehIds))

                    # Get Occupancy Info and Store
                    upOcc = (traci.inductionloop.getLastIntervalOccupancy('detUp1')+traci.inductionloop.getLastIntervalOccupancy('detUp2'))/2
                    downOcc = (traci.inductionloop.getLastIntervalOccupancy('detDown1')+traci.inductionloop.getLastIntervalOccupancy('detDown2'))/2
                    up_occ_meter.append(upOcc)
                    down_occ_meter.append(downOcc)
                                                                     
                    # Calcuate Rate limits - updates every 30 s based on 5 min averages for ramp 1 
                    maxRate, minRate = calcRateLimits(demand1_count,passage1_count,rampStorageLength)

                    # Calcuate Rate limits - updates every 30 s based on 5 min averages for ramp 1 
                    max2Rate, min2Rate = calcRateLimits(demand2_count,passage2_count,rampStorageLength)
                    
                    # Check if we need to switch to Flush phase
                    switch = computeSwitchFlush(up_occ_meter,down_occ_meter,critDensity,alpha_desired,alpha_low)
                    if switch == 'True':
                        
                        # Switch the phase to Flush
                        phase = 'flush'
                        print('Flush Phase Activated','Time[s]:', step*stepsize)
                        flush1Activate.append(step)
                        flush2Activate.append(step)

                        
                    # Compute new red time for ramp 1
                    r,lastRate = computeRed(lastRate,upOcc,downOcc,minRate,maxRate,critDensity,jamDensity,alpha_desired,alpha_low)
                    r = round(r)
                    
                    #Store Rate info for ramp 1
                    min1Rates.append(minRate)
                    max1Rates.append(maxRate)
                    rates1.append(lastRate)


                    # Compute new red time for ramp 2
                    r2,lastRate2 = computeRed(lastRate2,upOcc,downOcc,min2Rate,max2Rate,critDensity,jamDensity,alpha_desired,alpha_low)
                    r2 = round(r2)
                    
                    #Store Rate info for ramp 2
                    min2Rates.append(min2Rate)
                    max2Rates.append(max2Rate)
                    rates2.append(lastRate2)
                    
            
            # Get the new start and end times for red for ramp 1
            rstart = gend # Red time always starts once the green ends
            rend = rstart + r

            # Get the new start and end times for red for ramp 1
            rstart2 = gend2 # Red time always starts once the green ends
            rend2 = rstart2 + r2
            
            # IF we are switching to flush phase (condition above) get the intitail values to use
            if phase == 'flush':
                r = computeRedFlush(maxRate)
                rstart = 0 # These start/end times are used to make meter switch between red and green (hard to explain)
                rend = rstart + r
                gstart = rend 
                gend = gstart + g
                passageSinceFlush = 0
                greenCount = 0

                r2 = computeRedFlush(max2Rate)
                rstart2 = 0 # These start/end times are used to make meter switch between red and green (hard to explain)
                rend2 = rstart2 + r2
                gstart2 = rend2 
                gend2 = gstart2 + g2
                passageSinceFlush2 = 0
                greenCount2 = 0
                
            
            # GREEN
            while step <= gend/stepsize and step >= gstart/stepsize: # If time step in green range, set to green
                
                if alternate == 0:
                    traci.trafficlight.setPhase('J3',2) # Set Phase to green left on ramp 1
                    traci.trafficlight.setPhase('J9',2) # Set Phase to green left on ramp 2

                    traci.simulationStep()
                    step +=1
                    
                    # Turn ramp HVs to ACCs
                    for vehicle_id in traci.vehicle.getIDList():
                        if traci.vehicle.getTypeID(vehicle_id) == "HV_R_ACC":
                            lane_id = traci.vehicle.getLaneID(vehicle_id)
                            if lane_id == "switchACC_0" or lane_id == 'switchACC_1':
                                traci.vehicle.setType(vehicle_id, "ACC_AMax_R")
                            elif lane_id == "endSeg_0" or lane_id == 'endSeg_1':
                                traci.vehicle.setType(vehicle_id, "ACC_AMax_R")
                
                    
                elif alternate == 1:
                    traci.trafficlight.setPhase('J3',3) # Set Phase to green right on ramp 1
                    traci.trafficlight.setPhase('J9',3) # Set Phase to green right on ramp 2

                    traci.simulationStep()
                    step +=1
                    
                    # Turn ramp HVs to ACCs
                    for vehicle_id in traci.vehicle.getIDList():
                        if traci.vehicle.getTypeID(vehicle_id) == "HV_R_ACC":
                            lane_id = traci.vehicle.getLaneID(vehicle_id)
                            if lane_id == "switchACC_0" or lane_id == 'switchACC_1':
                                traci.vehicle.setType(vehicle_id, "ACC_AMax_R")
                            elif lane_id == "endSeg_0" or lane_id == 'endSeg_1':
                                traci.vehicle.setType(vehicle_id, "ACC_AMax_R")
                
                if step in update_steps_1: # If its a 30s interval, update all detector counts and calcuate redtime
                    
                    
                    # Get Queue info and store on ramp 1
                    new_vehIds = traci.inductionloop.getLastIntervalVehicleIDs('detPass2')
                    passage1_vehIds += [string for string in new_vehIds if string not in passage1_vehIds]
                    passage1_count.append(len(passage1_vehIds))
                    new_vehIdsA = traci.inductionloop.getLastIntervalVehicleIDs('detDemandA')
                    new_vehIdsB = traci.inductionloop.getLastIntervalVehicleIDs('detDemandB')
                    demand1_vehIds += [string for string in new_vehIdsA if string not in demand1_vehIds]
                    demand1_vehIds += [string for string in new_vehIdsB if string not in demand1_vehIds]
                    demand1_count.append(len(demand1_vehIds))


                    # Get Queue info and store for ramp 2
                    new_vehIds2 = traci.inductionloop.getLastIntervalVehicleIDs('detPass4')
                    passage2_vehIds += [string for string in new_vehIds2 if string not in passage2_vehIds]
                    passage2_count.append(len(passage2_vehIds))
                    new_vehIdsC = traci.inductionloop.getLastIntervalVehicleIDs('detDemandC')
                    new_vehIdsD = traci.inductionloop.getLastIntervalVehicleIDs('detDemandD')
                    demand2_vehIds += [string for string in new_vehIdsC if string not in demand2_vehIds]
                    demand2_vehIds += [string for string in new_vehIdsD if string not in demand2_vehIds]
                    demand2_count.append(len(demand2_vehIds))
                    
                    # Get Occupancy Info and Store
                    upOcc = (traci.inductionloop.getLastIntervalOccupancy('detUp1')+traci.inductionloop.getLastIntervalOccupancy('detUp2'))/2
                    downOcc = (traci.inductionloop.getLastIntervalOccupancy('detDown1')+traci.inductionloop.getLastIntervalOccupancy('detDown2'))/2
                    up_occ_meter.append(upOcc)
                    down_occ_meter.append(downOcc)
                    
                                   
                    # Calcuate Rate limits - updates every 30 s based on 5 min averages for ramp 1 
                    maxRate, minRate = calcRateLimits(demand1_count,passage1_count,rampStorageLength)

                    # Calcuate Rate limits - updates every 30 s based on 5 min averages for ramp 1 
                    max2Rate, min2Rate = calcRateLimits(demand2_count,passage2_count,rampStorageLength)


                    # Check if we need to switch to Flush phase
                    switch = computeSwitchFlush(up_occ_meter,down_occ_meter,critDensity,alpha_desired,alpha_low)
                    if switch == 'True':
                       
                        # Switch the phase to Flush
                        phase = 'flush'
                        print('Flush Phase Activated', 'Time[s]:', step*stepsize)
                        flush1Activate.append(step)
                        flush2Activate.append(step)
                        
                    # Compute new red time for ramp 1
                    r,lastRate = computeRed(lastRate,upOcc,downOcc,minRate,maxRate,critDensity,jamDensity,alpha_desired,alpha_low)
                    r = round(r)
                    
                    #Store Rate info
                    min1Rates.append(minRate)
                    max1Rates.append(maxRate)
                    rates1.append(lastRate)

                    # Compute new red time for ramp 2
                    r2,lastRate2 = computeRed(lastRate2,upOcc,downOcc,min2Rate,max2Rate,critDensity,jamDensity,alpha_desired,alpha_low)
                    r2 = round(r2)
                    
                    #Store Rate info
                    min2Rates.append(min2Rate)
                    max2Rates.append(max2Rate)
                    rates2.append(lastRate2)
             
            # Get the new start and end times for green ramp 1
            gstart = rend # Green time always starts once the red ends
            gend = gstart + g # Green time is always a constant g
            
            # Get the new start and end times for green ramp 2
            gstart2 = rend2 # Green time always starts once the red ends
            gend2 = gstart2 + g2 # Green time is always a constant g
            

            # Switch the alternate green
            if alternate == 0:
                alternate = 1
            elif alternate == 1:
                alternate = 0
# ---------------------------------------------------
# ---------------------------------------------------
# ---------------------------------------------------

            # IF we are switching to flush phase (condition above) get the intitail values to use
            if phase == 'flush':
                r = computeRedFlush(maxRate)
                rstart = 0 # These start/end times are used to make meter switch between red and green (hard to explain)
                rend = rstart + r
                gstart = rend 
                gend = gstart + g
                passageSinceFlush = 0
                greenCount = 0

                r2 = computeRedFlush(max2Rate)
                rstart2 = 0 # These start/end times are used to make meter switch between red and green (hard to explain)
                rend2 = rstart2 + r2
                gstart2 = rend2 
                gend2 = gstart2 + g2
                passageSinceFlush2 = 0
                greenCount2 = 0
                
        # 3. FLUSH PHASE - Cycles at the max rate, switch to Stopped Phase if queue is empty
        while phase == 'flush' and step <= endStep:
            traci.lane.setDisallowed('rampEnd_0',[]) # Now 2 lines form for meter
            greenTallyActivated = False # Use to keep tally of green counts
            
            # RED
            while step <= rend/stepsize and step >= rstart/stepsize: # If time step in red range, set to red
                traci.trafficlight.setPhase('J3',1) # Set phase to all red on ramp 1
                traci.trafficlight.setPhase('J9',1) # Set phase to all red on ramp 2

                traci.simulationStep() 
                step += 1
                
                # Turn ramp HVs to ACCs
                for vehicle_id in traci.vehicle.getIDList():
                    if traci.vehicle.getTypeID(vehicle_id) == "HV_R_ACC":
                        lane_id = traci.vehicle.getLaneID(vehicle_id)
                        if lane_id == "switchACC_0" or lane_id == 'switchACC_1':
                            traci.vehicle.setType(vehicle_id, "ACC_AMax_R")
                        elif lane_id == "endSeg_0" or lane_id == 'endSeg_1':
                            traci.vehicle.setType(vehicle_id, "ACC_AMax_R")
                
                if step in update_steps_1: # If its a 30s interval, update all detector counts and calcuate redtime
                    
                    # Get Queue info and store
                    new_vehIds = traci.inductionloop.getLastIntervalVehicleIDs('detPass2')
                    passage1_vehIds += [string for string in new_vehIds if string not in passage1_vehIds]
                    passage1_count.append(len(passage1_vehIds))
                    new_vehIdsA = traci.inductionloop.getLastIntervalVehicleIDs('detDemandA')
                    new_vehIdsB = traci.inductionloop.getLastIntervalVehicleIDs('detDemandB')
                    demand1_vehIds += [string for string in new_vehIdsA if string not in demand1_vehIds]
                    demand1_vehIds += [string for string in new_vehIdsB if string not in demand1_vehIds]
                    demand1_count.append(len(demand1_vehIds))


                    # Get Queue info and store for ramp 2
                    new_vehIds2 = traci.inductionloop.getLastIntervalVehicleIDs('detPass4')
                    passage2_vehIds += [string for string in new_vehIds2 if string not in passage2_vehIds]
                    passage2_count.append(len(passage2_vehIds))
                    new_vehIdsC = traci.inductionloop.getLastIntervalVehicleIDs('detDemandC')
                    new_vehIdsD = traci.inductionloop.getLastIntervalVehicleIDs('detDemandD')
                    demand2_vehIds += [string for string in new_vehIdsC if string not in demand2_vehIds]
                    demand2_vehIds += [string for string in new_vehIdsD if string not in demand2_vehIds]
                    demand2_count.append(len(demand2_vehIds))
                    
                    # Keep track of passage since flush started to see if queue empty on ramp 1
                    passageGain = passage1_count[-1] - passage1_count[-2]
                    passageSinceFlush += passageGain
                    
                    # Calcuate Rate limits - updates every 30 s based on 5 min averages 
                    maxRate, minRate = calcRateLimits(demand1_count,passage1_count,rampStorageLength)
            
                    # Compute new red time
                    r = computeRedFlush(maxRate)
                    r = round(r)
                    
                    #Store Rate info - During flush rate is always max rate and max rate now is 150% of track demand
                    min1Rates.append(minRate)
                    max1Rates.append(min(720,maxRate*1.2))
                    rates1.append(min(720,maxRate*1.2))
                    
                    # Check if we need to switch to Stopped Phase - USNURE ABOUT THIS CONDITION
                    print('greenCount:',greenCount)
                    print('passageSinceFlush:',passageSinceFlush)

                    # Keep track of passage since flush started to see if queue empty on ramp 2
                    passageGain2 = passage2_count[-1] - passage2_count[-2]
                    passageSinceFlush2 += passageGain2
                    
                    # Calcuate Rate limits - updates every 30 s based on 5 min averages 
                    max2Rate, min2Rate = calcRateLimits(demand2_count,passage2_count,rampStorageLength)
            
                    # Compute new red time
                    r2 = computeRedFlush(max2Rate)
                    r2 = round(r2)
                    
                    #Store Rate info - During flush rate is always max rate and max rate now is 150% of track demand
                    min2Rates.append(min2Rate)
                    max2Rates.append(min(720,max2Rate*1.2))
                    rates2.append(min(720,max2Rate*1.2))
                    
                    # Check if we need to switch to Stopped Phase - USNURE ABOUT THIS CONDITION
                    print('greenCount on ramp 2:',greenCount2)
                    print('passageSinceFlush on ramp 2:',passageSinceFlush2)



                    if passageSinceFlush < greenCount:
                        print('Stopped Phase Activated','Time[s]:', step*stepsize)
                        phase = 'stopped'
                        up_occ_stopped.clear() # Clear lists for next phase
                        down_occ_stopped.clear()
                        stopped1Activate.append(step)
                        stopped2Activate.append(step)

                        
                                     
            # Get the new start and end times for red on ramp 1
            rstart = gend # Red time always starts once the green ends
            rend = rstart + r
            
            # Get the new start and end times for red on ramp 2
            rstart2 = gend2 # Red time always starts once the green ends
            rend2 = rstart2 + r2
            
            # GREEN
            while step <= gend/stepsize and step >= gstart/stepsize: # If time step in green range, set to green
                
                # Add green count only when loop starts
                if not greenTallyActivated:
                    greenCount += 1
                    greenTallyActivated = True
                    
                if alternate == 0:
                    traci.trafficlight.setPhase('J3',2) # Set Phase to green left

                    traci.trafficlight.setPhase('J9',2) # Set Phase to green left

                    traci.simulationStep()
                    step +=1
                    
                    # Turn ramp HVs to ACCs
                    for vehicle_id in traci.vehicle.getIDList():
                        if traci.vehicle.getTypeID(vehicle_id) == "HV_R_ACC":
                            lane_id = traci.vehicle.getLaneID(vehicle_id)
                            if lane_id == "switchACC_0" or lane_id == 'switchACC_1':
                                traci.vehicle.setType(vehicle_id, "ACC_AMax_R")
                            elif lane_id == "endSeg_0" or lane_id == 'endSeg_1':
                                traci.vehicle.setType(vehicle_id, "ACC_AMax_R")
                elif alternate == 1:
                    traci.trafficlight.setPhase('J3',3) # Set Phase to green right on ramp 1


                    traci.trafficlight.setPhase('J9',3) # Set Phase to green right on ramp 2

                    traci.simulationStep()
                    step +=1
                    
                    # Turn ramp HVs to ACCs
                    for vehicle_id in traci.vehicle.getIDList():
                        if traci.vehicle.getTypeID(vehicle_id) == "HV_R_ACC":
                            lane_id = traci.vehicle.getLaneID(vehicle_id)
                            if lane_id == "switchACC_0" or lane_id == 'switchACC_1':
                                traci.vehicle.setType(vehicle_id, "ACC_AMax_R")
                            elif lane_id == "endSeg_0" or lane_id == 'endSeg_1':
                                traci.vehicle.setType(vehicle_id, "ACC_AMax_R")
                
                if step in update_steps_1: # If its a 30s interval, update all detector counts and calcuate redtime
                    
                    # Get Queue info and store
                    new_vehIds = traci.inductionloop.getLastIntervalVehicleIDs('detPass2')
                    passage1_vehIds += [string for string in new_vehIds if string not in passage1_vehIds]
                    passage1_count.append(len(passage1_vehIds))
                    new_vehIdsA = traci.inductionloop.getLastIntervalVehicleIDs('detDemandA')
                    new_vehIdsB = traci.inductionloop.getLastIntervalVehicleIDs('detDemandB')
                    demand1_vehIds += [string for string in new_vehIdsA if string not in demand1_vehIds]
                    demand1_vehIds += [string for string in new_vehIdsB if string not in demand1_vehIds]
                    demand1_count.append(len(demand1_vehIds))


                    # Get Queue info and store for ramp 2
                    new_vehIds2 = traci.inductionloop.getLastIntervalVehicleIDs('detPass4')
                    passage2_vehIds += [string for string in new_vehIds2 if string not in passage2_vehIds]
                    passage2_count.append(len(passage2_vehIds))
                    new_vehIdsC = traci.inductionloop.getLastIntervalVehicleIDs('detDemandC')
                    new_vehIdsD = traci.inductionloop.getLastIntervalVehicleIDs('detDemandD')
                    demand2_vehIds += [string for string in new_vehIdsC if string not in demand2_vehIds]
                    demand2_vehIds += [string for string in new_vehIdsD if string not in demand2_vehIds]
                    demand2_count.append(len(demand2_vehIds))




                    
                    # Keep track of passage since flush started to see if queue empty
                    passageGain = passage1_count[-1] - passage1_count[-2]
                    passageSinceFlush += passageGain

                    # Calcuate Rate limits - updates every 30 s based on 5 min averages 
                    maxRate, minRate = calcRateLimits(demand1_count,passage1_count,rampStorageLength)
            
                    # Compute new red time
                    r = computeRedFlush(maxRate)
                    r = round(r)
                    
                    #Store Rate info - During flush rate is always max rate which is now 150% track demand
                    min1Rates.append(minRate)
                    max1Rates.append(min(720,maxRate*1.2))
                    rates1.append(min(720,maxRate*1.2))
                    


                    # Keep track of passage since flush started to see if queue empty on ramp 2
                    passageGain2 = passage2_count[-1] - passage2_count[-2]
                    passageSinceFlush2 += passageGain2
                    
                    # Calcuate Rate limits - updates every 30 s based on 5 min averages 
                    max2Rate, min2Rate = calcRateLimits(demand2_count,passage2_count,rampStorageLength)
            
                    # Compute new red time
                    r2 = computeRedFlush(max2Rate)
                    r2 = round(r2)
                    
                    #Store Rate info - During flush rate is always max rate and max rate now is 150% of track demand
                    min2Rates.append(min2Rate)
                    max2Rates.append(min(720,max2Rate*1.2))
                    rates2.append(min(720,max2Rate*1.2))
                    
                    # Check if we need to switch to Stopped Phase - USNURE ABOUT THIS CONDITION
                    print('greenCount on ramp 2:',greenCount2)
                    print('passageSinceFlush on ramp 2:',passageSinceFlush2)




                    # Check if we need to switch to Stopped Phase
                    print('greenCount:',greenCount)
                    print('passageSinceFlush:',passageSinceFlush)
                    if passageSinceFlush < greenCount:
                        print('Stopped Phase Activated','Time[s]:', step*stepsize)
                        phase = 'stopped'
                        up_occ_stopped.clear() # Clear lists for next phase
                        down_occ_stopped.clear()
                        stopped1Activate.append(step)
                        stopped2Activate.append(step)
                        
                                
            # Get the new start and end times for green on ramp 1
            gstart = rend # Green time always starts once the red ends
            gend = gstart + g # Green time is always a constant g
            
            # Get the new start and end times for green on ramp 2
            gstart2 = rend2 # Green time always starts once the red ends
            gend2 = gstart2 + g2 # Green time is always a constant g

            # Switch the alternate green
            if alternate == 0:
                alternate = 1
            elif alternate == 1:
                alternate = 0
                
            # --------------------------------------------------
            # --------------------------------------------------
            # --------------------------------------------------
            # --------------------------------------------------


        # 4. STOPPED PHASE - All green, Switch to meter when mainline has high enough density      
        while phase == 'stopped' and step <= endStep:
            traci.lane.setDisallowed('rampStart_0',['passenger']) # Only 1 lane used when meter off
            traci.trafficlight.setPhase('J3',0) # Set phase to all green

            traci.lane.setDisallowed('ramp2Start_0',['passenger']) # Only 1 lane used when meter off on ramp 2
            traci.trafficlight.setPhase('J9',0) # Set phase to all green


            traci.simulationStep() # Progress Sim by 1 time step
            step += 1
            
            # Turn ramp HVs to ACCs
            for vehicle_id in traci.vehicle.getIDList():
                if traci.vehicle.getTypeID(vehicle_id) == "HV_R_ACC":
                    lane_id = traci.vehicle.getLaneID(vehicle_id)
                    if lane_id == "switchACC_0" or lane_id == 'switchACC_1':
                        traci.vehicle.setType(vehicle_id, "ACC_AMax_R")
                    elif lane_id == "endSeg_0" or lane_id == 'endSeg_1':
                        traci.vehicle.setType(vehicle_id, "ACC_AMax_R")
            
            if step in update_steps_1: #Check if meter should be turned on (check every 30s with 5 min averages)
                
                #Store Rate info - When meter off store as 0 ramp 1
                min1Rates.append(0)
                max1Rates.append(0)
                rates1.append(0)
                
                #Store Rate info - When meter off store as 0 ramp 2
                min2Rates.append(0)
                max2Rates.append(0)
                rates2.append(0)
                
                # Get Queue info and store
                new_vehIds = traci.inductionloop.getLastIntervalVehicleIDs('detPass2')
                passage1_vehIds += [string for string in new_vehIds if string not in passage1_vehIds]
                passage1_count.append(len(passage1_vehIds))
                new_vehIdsA = traci.inductionloop.getLastIntervalVehicleIDs('detDemandA')
                new_vehIdsB = traci.inductionloop.getLastIntervalVehicleIDs('detDemandB')
                demand1_vehIds += [string for string in new_vehIdsA if string not in demand1_vehIds]
                demand1_vehIds += [string for string in new_vehIdsB if string not in demand1_vehIds]
                demand1_count.append(len(demand1_vehIds))


                # Get Queue info and store for ramp 2
                new_vehIds2 = traci.inductionloop.getLastIntervalVehicleIDs('detPass4')
                passage2_vehIds += [string for string in new_vehIds2 if string not in passage2_vehIds]
                passage2_count.append(len(passage2_vehIds))
                new_vehIdsC = traci.inductionloop.getLastIntervalVehicleIDs('detDemandC')
                new_vehIdsD = traci.inductionloop.getLastIntervalVehicleIDs('detDemandD')
                demand2_vehIds += [string for string in new_vehIdsC if string not in demand2_vehIds]
                demand2_vehIds += [string for string in new_vehIdsD if string not in demand2_vehIds]
                demand2_count.append(len(demand2_vehIds))

                
                # Get Occupancy Info and Store
                upOcc = (traci.inductionloop.getLastIntervalOccupancy('detUp1')+traci.inductionloop.getLastIntervalOccupancy('detUp2'))/2
                downOcc = (traci.inductionloop.getLastIntervalOccupancy('detDown1')+traci.inductionloop.getLastIntervalOccupancy('detDown2'))/2
                up_occ_stopped.append(upOcc)
                down_occ_stopped.append(downOcc)
                                
                # Compute Switch Condition
                switch = computeSwitchMeterStopped(up_occ_stopped, down_occ_stopped,critDensity,alpha_desired,alpha_low)
                if switch == 'True':
                    
                    # Get initial rate for meter phase [veh/hr] ramp 1
                    lastRate = traci.inductionloop.getLastIntervalVehicleNumber('detPass1') # 90 s veh count
                    lastRate = lastRate * 40 # veh/hr
                    print('Initial Rate [veh/hr]:', lastRate)
                    
                    # Define initial red and green times for meter phase
                    alternate = 0 # Alternate between left and right lane for meter (0 = left 1 = right)
                    g = 3 # Green time (constant, allows 1 veh to pass)
                    r = round((3600 / lastRate) - g) # Calc from rate value
                    print('Initial Red Time:', r)

                    rstart = 0 # These start/end times are used to make meter switch between red and green (hard to explain, see presentation for detatils)
                    rend = rstart + r # While it seems the current step wont fall within these bounds. The bounds will continously update in the next while loop until they are within range of the step. (add print statments to prove)
                    gstart = rend 
                    gend = gstart + g
                    
                    
                    
                    
                    # Get initial rate for meter phase [veh/hr] ramp 1
                    lastRate2 = traci.inductionloop.getLastIntervalVehicleNumber('detPass3') # 90 s veh count
                    lastRate2 = lastRate2 * 40 # veh/hr
                    print('Initial Rate [veh/hr]:', lastRate2)
                    
                    # Define initial red and green times for meter phase
                    alternate2 = 0 # Alternate between left and right lane for meter (0 = left 1 = right)
                    g2 = 3 # Green time (constant, allows 1 veh to pass)
                    r2 = round((3600 / lastRate2) - g2) # Calc from rate value
                    print('Initial Red Time:', r)

                    rstart2 = 0 # These start/end times are used to make meter switch between red and green (hard to explain, see presentation for detatils)
                    rend2 = rstart2 + r2 # While it seems the current step wont fall within these bounds. The bounds will continously update in the next while loop until they are within range of the step. (add print statments to prove)
                    gstart2 = rend2 
                    gend2 = gstart2 + g2



                    # Switch the phase to meter
                    if control == 1: # If we test scenario with no meter (control=2) just keep meter off
                        phase = 'meter'
                        print('Meter Phase Activated!', 'Time[s]:', step*stepsize) # Print timestep meter turns on. Important so we can know when to start "counting" travel time
                        down_occ_meter.clear() # Clear meter lists so we dont start flush right away!
                        up_occ_meter.clear()
                        meter1Activate.append(step)
                        meter2Activate.append(step)
            
      
    traci.close()
    sys.stdout.flush()
    
    return(passage1_count,demand1_count,meter1Activate,flush1Activate,stopped1Activate,min1Rates,max1Rates,rates1,passage2_count,demand2_count,meter2Activate,flush2Activate,stopped2Activate,min2Rates,max2Rates,rates2)
    


# ## RUN THE ABOVE SCRIPT ##
# if __name__ == "__main__":
#     options = get_options()

#     # SHOW SIM GUI OR NOT: IF want GUI set first line to 'sumo' and second to 'sumo-gui' and vice versa
#     if options.nogui:
#         sumoBinary = checkBinary('sumo')
#     else:
#         sumoBinary = checkBinary('sumo-gui')
    
#     ## PARAMETERS: ENTER VALUES HERE##
#     control = [1] # 1 is ramp meter, 2 is no meter (all green): Can run both one after another by putting both numbers in list
#     stepsize = '.05' # Step length (prob keep at 0.1s)
#     site = "site5.sumocfg" # Enter name of site config file here
#     # critDensity = 40.04 #[veh/mi-ln] # 0 MPR: DEFAULT
#     # jamDensity = 191.58
#     # critDensity = 34.46 # 25 MPR
#     # jamDensity = 176.36 
#     # critDensity = 30.22 # 50 MPR
#     # jamDensity = 163.38
#     # critDensity = 26.92 # 75 MPR
#     # jamDensity = 152.18 
#     critDensity = 24.27 # 100 MPR
#     jamDensity = 142.42 #
#     rampStorageLength = 180 # [m] DEPENDS ON SITE
#     rampStorageDensity = 20/180 # Always same (always HV on ramp)
    
    
#     # Runs the program
#     for i in range(len(control)):
#         traci.start([sumoBinary,"--step-length",stepsize, "-c", site, "--statistic-output","tripinfo.xml", "--duration-log.statistics"]) 
#         passage_count,demand_count,meterActivate,flushActivate,stoppedActivate,minRates,maxRates,rates = run(control[i],critDensity,jamDensity,rampStorageLength)


# ## EXTRACT IMPORTANT DATA FROM .XML FILES
# import xml.etree.ElementTree as ET
# import numpy as np

# # Mainline Density [veh/mi] : From .xml File not Traci retrieval
# fieldL = 16.4042
# tree = ET.parse('detUp1.xml')
# root = tree.getroot()
# interval_OccsUp1 = [0]
# interval_time_steps2 = [0]
# for interval in root.iter('interval'):
#     interval_Occ = float(interval.attrib['occupancy'])
#     time_step = float(interval.attrib['end'])
#     interval_OccsUp1.append(interval_Occ)
#     interval_time_steps2.append(time_step)
# interval_OccsUp1 = np.array(interval_OccsUp1)

# tree = ET.parse('detUp2.xml')
# root = tree.getroot()
# interval_OccsUp2 = [0]
# interval_time_steps2 = [0]
# for interval in root.iter('interval'):
#     interval_Occ = float(interval.attrib['occupancy'])
#     time_step = float(interval.attrib['end'])
#     interval_OccsUp2.append(interval_Occ)
#     interval_time_steps2.append(time_step)
# interval_OccsUp2 = np.array(interval_OccsUp2)

# interval_OccsUp = (interval_OccsUp1 + interval_OccsUp2)/2


# tree = ET.parse('detDown1.xml')
# root = tree.getroot()
# interval_OccsDown1 = [0]
# interval_time_steps2 = [0]
# for interval in root.iter('interval'):
#     interval_Occ = float(interval.attrib['occupancy'])
#     time_step = float(interval.attrib['end'])
#     interval_OccsDown1.append(interval_Occ)
#     interval_time_steps2.append(time_step)
# interval_OccsDown1 = np.array(interval_OccsDown1)

# tree = ET.parse('detDown2.xml')
# root = tree.getroot()
# interval_OccsDown2 = [0]
# interval_time_steps2 = [0]
# for interval in root.iter('interval'):
#     interval_Occ = float(interval.attrib['occupancy'])
#     time_step = float(interval.attrib['end'])
#     interval_OccsDown2.append(interval_Occ)
#     interval_time_steps2.append(time_step)
# interval_OccsDown2 = np.array(interval_OccsDown2)

# interval_OccsDown = (interval_OccsDown1 + interval_OccsDown2)/2

# upDen = ((interval_OccsUp/100) * 5280) / fieldL #[veh/mile]
# downDen = ((interval_OccsDown/100) * 5280) / fieldL #[veh/mile]

# segDen = (upDen + downDen) / 2# [veh/mi]

# # 20 min Interval Travel Times [s]
# tree = ET.parse('mainTravelTimeLong.xml')
# root = tree.getroot()
# interval_travel_times = [0]
# interval_vehicle_sums = [0]
# interval_time_steps = [0]
# for interval in root.iter('interval'):
#     mean_travel_time = float(interval.attrib['meanTravelTime'])
#     vehicle_sum = int(interval.attrib['vehicleSum'])
#     time_step = float(interval.attrib['end'])
#     interval_travel_times.append(mean_travel_time)
#     interval_vehicle_sums.append(vehicle_sum)
#     interval_time_steps.append(time_step)

# #Cummulative Travel Times [s]
# cumAvgTT = [0]
# cumVeh = 0
# cumSec = 0
# for i in range(1,len(interval_time_steps)):
#     cumVeh += interval_vehicle_sums[i]
#     cumSec += interval_travel_times[i] * interval_vehicle_sums[i]
#     cumAvgTT.append(cumSec/cumVeh)

# # Total Travel Times [s]
# tree = ET.parse('mainTravelTimeTotalLong.xml')
# root = tree.getroot()
# totalTravelTime = []
# for interval in root.iter('interval'):
#     totalTravelTime.append(float(interval.attrib['meanTravelTime']))
    
# ## XML Queue info (just to make sure Traci values ok)
# tree1 = ET.parse('detDemandA.xml')
# root1 = tree1.getroot()
# tree2 = ET.parse('detDemandB.xml')
# root2 = tree2.getroot()
# tree3 = ET.parse('detPass2.xml')
# root3 = tree3.getroot()
# demandA = [0]
# demandB = [0]
# passage = [0]
# for interval in root1.iter('interval'):
#     interval_demandA = float(interval.attrib['nVehContrib'])
#     demandA.append(interval_demandA)
# demandA = np.array(demandA)
# for interval in root2.iter('interval'):
#     interval_demandB = float(interval.attrib['nVehContrib'])
#     demandB.append(interval_demandB)  
# demandB = np.array(demandB)
# for interval in root3.iter('interval'):
#     interval_passage = float(interval.attrib['nVehContrib'])
#     passage.append(interval_passage)  
# passage = np.array(passage)
# timeSteps = np.arange(30, len(passage) * 30, 30)
# cumDemand = np.delete(np.cumsum(demandA) + np.cumsum(demandB),-1)
# cumPassage = np.delete(np.cumsum(passage),-1)
# QLength = cumDemand-cumPassage
# combined_cum = np.column_stack((timeSteps, cumDemand, cumPassage,QLength))

# ## CALC INTERESTING VALUES TO STORE
# #QueueInfo
# queueLength = np.array(demand_count)-np.array(passage_count)
# avgQueueLength = np.average(queueLength)
# maxQueueLength = np.max(queueLength)
# maxAllowQueueLength = rampStorageDensity*rampStorageLength*2

# #Mainline
# avgDensity = np.average(segDen)


# ## SAVE IMPORTANT DATA##
# import pandas as pd
# # Define Data for all excel tabs
# simInfo = {'critDensity[veh/mi]':[critDensity],'jamDensity[veh/mi]':[jamDensity],
#            'meterActivate':np.array(meterActivate)*float(stepsize),
#            'flushActivate':np.array(flushActivate)*float(stepsize),
#            'stoppedActivate':np.array(stoppedActivate)*float(stepsize),
#            'mainLineTT[s]':totalTravelTime,
#            'Time[s]':interval_time_steps,
#            '20minTT[s]':interval_travel_times,
#            'cumTT[s]':cumAvgTT
#              }
# rampInfo = {'Time[s]':interval_time_steps2,'cumDemand':demand_count,
#             'cumPassage':passage_count,'queueLength':queueLength,
#             'avgLength':[avgQueueLength],'maxLength':[maxQueueLength],
#             'maxAllowLenth:':[maxAllowQueueLength]}

# rateInfo = {'Time[s]':interval_time_steps2,'minRate[veh/hr]':minRates,'maxRate[veh/hr]':maxRates,
#              'ReleaseRate[veh/hr]':rates,'MainlineDensity[veh/mi]':segDen,
#              'avgDensity':[avgDensity]}
# # Create an Excel writer object
# writer = pd.ExcelWriter('output.xlsx', engine='xlsxwriter')
# # Create a DataFrame for each tab's data
# simInfo = pd.DataFrame.from_dict(simInfo, orient='index').transpose()
# rampInfo = pd.DataFrame.from_dict(rampInfo, orient='index').transpose()
# rateInfo= pd.DataFrame.from_dict(rateInfo, orient='index').transpose()
# # Write dataframes to their respective tabs
# simInfo.to_excel(writer, sheet_name='simInfo', index=False)
# rampInfo.to_excel(writer, sheet_name='rampInfo', index=False)
# rateInfo.to_excel(writer, sheet_name='ratesInfo', index=False)
# # Save the Excel file
# writer.save()

# ## PLOT IMPORTANT DATA##
# import matplotlib.pyplot as plt

# #Cummulative Ramp passage and demand
# fig, ax = plt.subplots()
# t = [30 * i for i in range(len(demand_count))]
# ax.plot(t, passage_count, label='Output') # Demand vs Passage
# ax.plot(t, demand_count, label='Input')
# for t in meterActivate:
#     ax.axvline(x=t*float(stepsize), linestyle='dashed', color='Black') # Vertical lines of phases

# for t in flushActivate:
#     ax.axvline(x=t*float(stepsize), linestyle='dashed', color='Orange')
    
# for t in stoppedActivate:
#     ax.axvline(x=t*float(stepsize), linestyle='dashed', color='Blue')
# ax.set_xlabel('Time Since Start')
# ax.set_ylabel('Vehicle Count')
# ax.legend()
# plt.savefig('Queue.png')
# plt.show()

# # 20 min Interval Travel Time
# fig, ax = plt.subplots()
# ax.plot(interval_time_steps,interval_travel_times)
# for t in meterActivate:
#     ax.axvline(x=t*float(stepsize), linestyle='dashed', color='Black') # Vertical lines of phases
# for t in flushActivate:
#     ax.axvline(x=t*float(stepsize), linestyle='dashed', color='Orange')  
# for t in stoppedActivate:
#     ax.axvline(x=t*float(stepsize), linestyle='dashed', color='Blue')
# ax.set_xlabel('Time Since Start')
# ax.set_ylabel('Interval Avg Mainline TT [s]')
# plt.savefig('MainIntAvgTT.png')
# plt.show()

# # Cummulative Travel Time
# fig, ax = plt.subplots()
# ax.plot(interval_time_steps,cumAvgTT)
# for t in meterActivate:
#     ax.axvline(x=t*float(stepsize), linestyle='dashed', color='Black') # Vertical lines of phases
# for t in flushActivate:
#     ax.axvline(x=t*float(stepsize), linestyle='dashed', color='Orange')  
# for t in stoppedActivate:
#     ax.axvline(x=t*float(stepsize), linestyle='dashed', color='Blue')
# ax.set_xlabel('Time Since Start')
# ax.set_ylabel('Cummulative Avg Mainline TT [s]')
# plt.savefig('MainCumAvgTT.png')
# plt.show()

# #Min,Max, and release rates
# fig, ax = plt.subplots()
# t = [30 * i for i in range(len(rates))]
# ax.plot(t, minRates, label='minRate') 
# ax.plot(t, maxRates, label='maxRate')
# ax.plot(t, rates, label='ReleaseRate')
# for t in meterActivate:
#     ax.axvline(x=t*float(stepsize), linestyle='dashed', color='Black') # Vertical lines of phases
# for t in flushActivate:
#     ax.axvline(x=t*float(stepsize), linestyle='dashed', color='Orange')
# for t in stoppedActivate:
#     ax.axvline(x=t*float(stepsize), linestyle='dashed', color='Blue')
# ax.set_xlabel('Time Since Start')
# ax.set_ylabel('Ramp Rates [veh/hr]')
# ax.legend()
# plt.savefig('Rates.png')
# plt.show()

# # Mainline Density
# fig, ax = plt.subplots()
# ax.plot(interval_time_steps2,segDen)
# for t in meterActivate:
#     ax.axvline(x=t*float(stepsize), linestyle='dashed', color='Black') # Vertical lines of phases
# for t in flushActivate:
#     ax.axvline(x=t*float(stepsize), linestyle='dashed', color='Orange')  
# for t in stoppedActivate:
#     ax.axvline(x=t*float(stepsize), linestyle='dashed', color='Blue')
    
# ax.axhline(y=avgDensity, linestyle='dotted', color='Green', label='Average Density')
# ax.axhline(y=critDensity, linestyle='dotted', color='Red', label='Critical Density')
# ax.set_xlabel('Time Since Start')
# ax.set_ylabel('Avg Mainline Density [veh/mi]')
# ax.legend() 
# plt.savefig('Density.png')
# plt.show()

