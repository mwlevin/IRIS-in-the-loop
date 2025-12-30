# Initialization copied from SUMO documentation
from __future__ import absolute_import
from __future__ import print_function
import os
import sys
import optparse
import time

import socket





'''
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
    
'''


from sumolib import checkBinary
import traci
import threading

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

def run(directory, control,critDensity,jamDensity,rampStorageLength,alpha_desired,alpha_low):
    
    if control:
        print("meter active")
    else:
        print("meter disabled")
        
        
    # Define Constants and Initialization
    steps_per_sec = 20
    stepsize = 1/steps_per_sec
    step = 0
    time_period = 30
    
    
    test = False
    
    min_red_time = 2 # constant minimum red time for meter
    green_time = 3 # constant green time for meter
    
    phase = 'notStarted' # Initial Phase
    phase1 = 'notStarted' # Initial Phase
    phase2 = 'notStarted' # Initial Phase
    
    
    
    
    meters = []
    lanes = dict()
    
    with open(directory+"/meters.txt") as file:
        lines = [line.rstrip() for line in file]
        for line in lines:
            data = line.split()
            meters.append(data[0])
            lanes[data[0]] = data[1]
    
    #lanes["J3"] = "rampStart_0"
    #lanes["J9"] = "ramp2Start_0"
    
    print("meters: ", meters)
    print("lanes: ", lanes)    
    
    
    # detPass1 and detPass3 hvae the same location as detPass2 and detPass 4 but a different period?
    detectors = []
    with open(directory+"/detectors.txt") as file:
        detectors = [line.rstrip() for line in file]
    
    print("detectors: ", detectors)
    
    #detectors = ["detDemandA", "detDemandB", "detDemandC", "detDemandD", "detMerge1", "detMerge2", "detGreen1", "detGreen2", "detDown1", "detDown2", "detPass2", "detPass4", "detUp1", "detUp2", "detDSA1", "detDSA2", "detDSB1", "detDSB2", "detUSB1", "detUSB2"]
    
    connection = None
    
    if control and not test:
        
        host = "localhost"
        port = 5452
    
        print("connecting to IRIS server ", host, port);
        
        while(True):
            try:
                connection = socket.socket()
                connection.connect((host, port))
                break
            except ConnectionRefusedError:
                print("Waiting for IRIS server")
                time.sleep(1)
                
        print("connected")
    else:
        print("test only, no server conection")
    
    
    
    endStep = 3600* steps_per_sec *1.2 # Time to simulate (0.05 seconds steps)


    
    
    
    cc = {d : 0 for d in detectors}
    passed = {d : 0 for d in detectors}
    
    detectors_abbrv = dict()
    vehIds = dict()
    #counts = dict()
    #occupancies = dict()
    
    for det in detectors:
        vehIds[det] = set()
        #counts[det] = 0
        #occupancies[det] = 0
        # necessary due to limitations of PSQL database
        detectors_abbrv[det] = det.replace("Demand", "D").replace("Down", "Do").replace("Pass", "P").replace("Green", "G").replace("Merge", "M").replace("det", "d")
        
    
    metering_rates = dict()
    
    for m in meters:
        metering_rates[m] = dict()
        metering_rates[m]["on"] = False
        metering_rates[m]["last-green"] = 0
        metering_rates[m]["rate"] = -1
        metering_rates[m]["lane"] = 0
        
        traci.lane.setDisallowed(lanes[m], ['passenger'])
        
    listen_thread = None
        
    
         
    speeds = dict()
    waitingTime = dict()
    dataCount = 0
    
    for id in traci.lane.getIDList():
        speeds[id] = 0
        waitingTime[id] = 0
            
    # START SIMULATION, RUN FOR SET TIME 
    while step <= endStep:
        
        
        
        current_ms = time.time() * 1000
        
        print("\nSUMO time ", step*stepsize, "sec / ", endStep*stepsize, "sec\n")
        
        
        
        
        
        # get detector data
        if step > 0:
            
            # testing occupancy
            if "610EB" in directory:
                upstream_occ = traci.lane.getLastStepVehicleNumber('EB610-17_0') + traci.lane.getLastStepVehicleNumber('EB610-17_1')
                downstream_occ = traci.lane.getLastStepVehicleNumber('EB610-18_0') + traci.lane.getLastStepVehicleNumber('EB610-18_1') + traci.lane.getLastStepVehicleNumber('EB610-18_2')
                ramp_occ = traci.lane.getLastStepVehicleNumber('rmp-EB610/NBNoble_0') + traci.lane.getLastStepVehicleNumber('rmp-EB610/NBNoble_1')
                
                print(step*stepsize, "J89 occ check", upstream_occ, downstream_occ, ramp_occ)
            elif "test_network" in directory:
                upstream_occ = traci.lane.getLastStepVehicleNumber('switchACC_0') + traci.lane.getLastStepVehicleNumber('switchACC_1')
                downstream_occ = traci.lane.getLastStepVehicleNumber('endSeg_0') + traci.lane.getLastStepVehicleNumber('endSeg_1')
                ramp_occ = traci.lane.getLastStepVehicleNumber('ramp2End_0') #+ traci.lane.getLastStepVehicleNumber('ramp2End_1')
                ramp_queue = traci.lane.getLastStepVehicleNumber('ramp2Start_0')+traci.lane.getLastStepVehicleNumber('ramp2Start_1')
                
                print(step*stepsize, "J9 occ check", upstream_occ, downstream_occ, ramp_occ)
                print("\t", "ramp queue", ramp_queue)
            
            
            for det in detectors:
                
                # update detector counts
                detcount = readCount(det, vehIds[det])
                
                passed[det] = detcount
                cc[det] += detcount
                #counts[det] += count
                det_occ = traci.inductionloop.getLastIntervalOccupancy(det)
                
                occ = det_occ * 30.0/100 # this is a percentage of the time step, so multiply by period (30sec) to get time
                #occupancies[det] += occ 
                
                
                #print("detector ", det, "count=", count, "cc=", cc[det], "occupancy=", det_occ, "as % of 30sec interval")
                    
                msg = "det,"+detectors_abbrv[det]+","+str(detcount)+","+str(occ)
                #counts[det] = 0
                #occupancies[det] = 0
                
                if control and not test:
                    sendMessage(connection, msg)
        
        if test:
            for meter in meters:
                passCount = dict()
                queueCount = 0
                
                ramp = lanes[meter][4:].replace("_0", "")
                mergelane = "merge-"+ramp
                queuelane = "rmp-"+ramp
                
                
                for det in detectors:
                    detlane = traci.inductionloop.getLaneID(det)
                    if "G" in det and (detlane == mergelane+"_0" or detlane == mergelane+"_1"):
                        passCount[detlane] = passed[det]
                    
                
                rate = metering_rates[meter]["rate"]
                
                if rate > 0:
                    print("compare rate", meter, "expected is ", rate * 30.0/3600, "actual is ", passCount, "lane use is ", traci.lane.getLastStepVehicleNumber(queuelane+"_0"), traci.lane.getLastStepVehicleNumber(queuelane+"_1"),
                    "ds lane use is ", traci.lane.getLastStepVehicleNumber(mergelane+"_0"), traci.lane.getLastStepVehicleNumber(mergelane+"_1"))
                    
        if control:
            if test:
                for meter in meters:
                    if step*stepsize < 300:
                        rate = 0
                    elif step*stepsize < 3600:
                        rate = 1800
                    else:
                        rate = -1
                    
                    updateRate(meter, rate, metering_rates)
            else:
                # wait for meter rates
                for i in range(0, len(meters)):
                    meter = ""
                    rate = -1
            
                    message = readLine(connection)
                    meter, rate = processMessage(message)
                    updateRate(meter, rate, metering_rates)
                  
                
      
        sim_start_ms = time.time()* 1000
        
        # progress sim by time_period (probably 30sec)
        for i in range(0, round(time_period/stepsize)):
            
            dataCount += 1
            for id in traci.lane.getIDList():
                speeds[id] += traci.lane.getLastStepMeanSpeed(id)
                waitingTime[id] += traci.lane.getWaitingTime(id)

            # meter timing
            for m in meters:

                if rate == 0:
                    traci.lane.setDisallowed(lanes[m], []) # Now 2 lines form for meter
                    traci.trafficlight.setPhase(m, 1)
                    continue
                    
                rate = metering_rates[m]["rate"]
                headway = 3600.0/rate * steps_per_sec
                
                # set traffic signal
                if rate < 0:
                    traci.trafficlight.setPhase(m, 0)
                    #traci.lane.setDisallowed(lanes[m], ['passenger']) # disable lane 1
                    traci.trafficlight.setPhase(m, 0) # this is set to gg phase
                else:
                    traci.lane.setDisallowed(lanes[m], []) # Now 2 lines form for meter
                    
                    if(rate <= 1200):
                        # switch to rr
                        if step - metering_rates[m]["last-green"] == green_time * steps_per_sec:
                            traci.trafficlight.setPhase(m, 1)
                            metering_rates[m]["lane"] = 1 - metering_rates[m]["lane"] # switch active lane
                            
                            #print(m, "transition red", step, metering_rates[m]["last-green"])
                            #print("\tphase check ", m, traci.trafficlight.getPhase(m))
                        elif step - metering_rates[m]["last-green"] >= headway:
                            
                            
                            
                            if metering_rates[m]["lane"] == 0:
                                traci.trafficlight.setPhase(m, 2) # rg
                                
                            else:
                                traci.trafficlight.setPhase(m, 3) # gr
                        
                            #print(m, "transition green", metering_rates[m]["lane"], step, metering_rates[m]["last-green"], headway, "phase", traci.trafficlight.getRedYellowGreenState(m), )
                            
                            metering_rates[m]["last-green"] = step
                    else:
                        # switch to a block green time
                        # assume capacity is 2100 veh/hr/lane
                        green_time = rate / (2.0*2100.0) * 30 + 2 # startup lost time
                        
                        red_time = 30 - green_time
                        
                        if step - metering_rates[m]["last-green"] == green_time * steps_per_sec:
                            traci.trafficlight.setPhase(m, 1)
                            
                            print(m, "transition red", metering_rates[m]["lane"], step, metering_rates[m]["last-green"], green_time, "phase", traci.trafficlight.getRedYellowGreenState(m), )
                        elif step - metering_rates[m]["last-green"] >= red_time * steps_per_sec:
                            
                            print(m, "transition green", metering_rates[m]["lane"], step, metering_rates[m]["last-green"], green_time, "phase", traci.trafficlight.getRedYellowGreenState(m), )
                            metering_rates[m]["last-green"] = step
                            traci.trafficlight.setPhase(m, 0)
                            
                         
                
            traci.simulationStep() # Progress Sim by 1 time step       
            step += 1
            
        # end if no vehicles remaining
        if traci.simulation.getMinExpectedNumber() == 0:
            break
            
        used_ms = time.time() * 1000 - sim_start_ms
        
        print("simulation cpu time ", used_ms/1000.0)
                    
            
    for id in traci.lane.getIDList():
        speeds[id] = speeds[id] / dataCount
        waitingTime[id] = waitingTime[id] / dataCount
    
    with open("output.txt", "w") as f:
        for lid in traci.lane.getIDList():
            if 'ent' not in lid and 'rmp' not in lid and '/' not in lid:
                f.write(str(lid)+ "\t"+ str(speeds[lid])+ "\n")
        f.write("\n")
        
        for lid in traci.lane.getIDList():
            if 'rmp' in lid:
                ent_name = "ent-"+lid[4:0]
                incCount = 0
                
                for det in detectors:
                    if ent_name+"_0" == traci.inductionloop.getLaneID(det) or ent_name+"_1" == traci.inductionloop.getLaneID(det):
                        incCount += cc[det]
                    
                f.write(str(lid)+ "\t"+ str(waitingTime[lid])+ "\t"+str(incCount) + "\n") 
                      
 
    traci.close()
    sys.stdout.flush()
    
    

    
    if control and not test:
            
        connection.shutdown(0)
    
    #return(passage1_count,demand1_count,meter1Activate,flush1Activate,stopped1Activate,min1Rates,max1Rates,rates1,passage2_count,demand2_count,meter2Activate,flush2Activate,stopped2Activate,min2Rates,max2Rates,rates2)
    
def sendMessage(connection, msg):
    #print("sending \""+msg+"\"")
    connection.sendall((msg+"\n").encode())

def readCount(detector, vehIds):
    new_vehIds = traci.inductionloop.getLastIntervalVehicleIDs(detector)
    
    oldcount = len(vehIds)
    for id in new_vehIds:
        vehIds.add(id)
    
    newcount = len(vehIds)
    
    return newcount - oldcount
    

def readLine(sock):
    data = ""
    while True:
        chunk = sock.recv(1)
        if not chunk:
            break  # Connection closed or error
        
        #print(str(chunk), data)
        if chunk == b'\n':
            return data  # Return the line without the newline character
        else:
            data += str(chunk)[2]
    return data # Return any remaining data, even if it's not a complete line


def updateRate(meter, rate, metering_rates):
    if rate != metering_rates[meter]["rate"]:
        metering_rates[meter]["rate"] = rate
        metering_rates[meter]["changed"] = True   
        
        print("rate changed", meter, metering_rates[meter]["rate"], metering_rates[meter]["changed"])
        
        

def listen(metering_rates):
    while True:
        message = readLine(connection)
        meter, rate = processMessage(message)
        updateRate(meter, rate, metering_rates)
           
def processMessage(message):
    print("received \""+message+"\"");
    
    s = message.split(",")
    
    if s[0] == "meter-rate":
        meter = s[1][s[1].index("-")+1:]
        rate = s[2]
        if rate == "null":
            rate = -1 # change this, I'm not sure what "null" rate is
        else:
            rate = int(rate)
            
        return meter, rate

    

