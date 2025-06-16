# Initialization copied from SUMO documentation
from __future__ import absolute_import
from __future__ import print_function
import os
import sys
import optparse

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
    time_period = 30
    
    
    test = True
    
    min_red_time = 2 # constant minimum red time for meter
    green_time = 3 # constant green time for meter
    
    phase = 'notStarted' # Initial Phase
    phase1 = 'notStarted' # Initial Phase
    phase2 = 'notStarted' # Initial Phase
    
    
    if not test:
        connection = socket.socket()
        host = "localhost"
        port = 5452
    
        print("connecting to IRIS server ", host, port);
        connection.connect((host, port))
    
    lanes = dict()

    endStep = 72000 # Time to simulate (0.05 seconds steps)

    # this is hardcoded for now
    meters = ["J3", "J9"]
    lanes["J3"] = "rampStart_0"
    lanes["J9"] = "ramp2Start_0"
    
    
    
    
    # detPass1 and detPass3 hvae the same location as detPass2 and detPass 4 but a different period?
    detectors = ["detDemandA", "detDemandB", "detDemandC", "detDemandD", "detMerge1", "detMerge2", "detGreen1", "detGreen2", "detDown1", "detDown2", "detPass2", "detPass4", "detUp1", "detUp2"]
    
    
    
    
    
    detectors_abbrv = dict()
    vehIds = dict()
    #counts = dict()
    #occupancies = dict()
    
    for det in detectors:
        vehIds[det] = set()
        #counts[det] = 0
        #occupancies[det] = 0
        # necessary due to limitations of PSQL database
        detectors_abbrv[det] = det.replace("Demand", "Dem").replace("Down", "Do").replace("Pass", "Pa").replace("Green", "Gr").replace("Merge", "Me")
        
    
    metering_rates = dict()
    
    for m in meters:
        metering_rates[m]  = dict()
        metering_rates[m]["on"] = False
        metering_rates[m]["next-green-1"] = -1
        metering_rates[m]["next-red-1"] = 0
        metering_rates[m]["next-green-2"] = -1
        metering_rates[m]["next-red-2"] = 0
        metering_rates[m]["rate"] = -1
        metering_rates[m]["green-active-1"] = True
        metering_rates[m]["green-active-2"] = True
        metering_rates[m]["phase"] = 0
        metering_rates[m]["changed"] = False
        
    # START SIMULATION, RUN FOR SET TIME 
    while step <= endStep:
        
        print("\nSUMO step ", step, step*stepsize, "sec")
        # get detector data
        if step > 0:
            for det in detectors:
                
                # update detector counts
                count = readCount(det, vehIds[det])
                #counts[det] += count
                det_occ = traci.inductionloop.getLastIntervalOccupancy(det)
                
                occ = det_occ * 30.0/100 # this is a percentage of the time step, so multiply by period (30sec) to get time
                #occupancies[det] += occ 
                '''
                if det_occ > 0:
                    print(det, det_occ, occ)
                '''
                
                print("detector ", det, "count=", count, "occupancy=", det_occ, "as % of 30sec interval")
                    
                msg = "det,"+detectors_abbrv[det]+","+str(count)+","+str(occ)
                #counts[det] = 0
                #occupancies[det] = 0
                
                if not test:
                    sendMessage(connection, msg)
        
        # wait for meter rates
        for i in range(0, len(meters)):
            meter = ""
            rate = -1
            
            if not test:
                message = readLine(connection)
                meter, rate = processMessage(message)
            else:
                meter = meters[i]
                
                if step < time_period / stepsize * 1:
                    rate = -1
                elif step < time_period / stepsize * 2:
                    rate = 650
                elif step < time_period / stepsize * 4:
                    rate = 300
                elif step < time_period / stepsize * 6:
                    rate = 900
                elif step < time_period / stepsize * 8:
                    rate = -1
                elif step < time_period / stepsize * 10:
                    endStep = step
            
            if rate != metering_rates[meter]["rate"]:
                metering_rates[meter]["rate"] = rate
                metering_rates[meter]["changed"] = True
            
            if test:
                print("rate changed", meter, metering_rates[meter]["rate"], metering_rates[meter]["changed"])
        
        
                
                
            
        # progress sim by time_period (probably 30sec)
        for i in range(0, round(time_period/stepsize)):
            

            # meter timing
            for m in meters:

                # first step: calculate active phase
                # second step: update phase
                
                
                
                # check red times
                # corner case where red time lane 1 overlaps green start time lane 2
                
                for idx in range(1, 3):   
                    lane = str(idx)
                          
                    if step == metering_rates[m]["next-green-"+lane]:
                        metering_rates[m]["green-active-"+lane] = True
                        
                    elif step == metering_rates[m]["next-red-"+lane]:
                        metering_rates[m]["green-active-"+lane] = False
                        
                        # get rate to recalc next times
                        rate = metering_rates[m]["rate"]
                            
                        if rate == -1:
                            # meter off
                            # mode supersedes red/green phase status
                            metering_rates[m]["on"] = False
                            metering_rates[m]["changed"] = False
                            
                        
                            # recheck meter status at start of next interval
                            metering_rates[m]["next-red-1"] = step + round(time_period/stepsize) - i
                            metering_rates[m]["next-red-2"] = step + round(time_period/stepsize) - i
                            
                            if test:
                                print("time ", step * stepsize, "meter off")
                        else:
                            # lines so total time is 7200s but next green and red are based on single lane 
                            headway = 7200/rate
                            # for example, 3600 vph becomes 2 sec headway. Every 2 seconds, 1 vehicle leaves per lane, equivalent to 1 vehicle every 1 second
                                
                            # if rate changes, the reset both meters. The second meter red time needs to be changed so that the 2 meters are equally spaced
                            # if rate changes from -1 to active, changed = True
                            if metering_rates[m]["changed"] == True:
                                metering_rates[m]["changed"] = False
                                
                                
                                # first lane will reset both lanes
                                
                                # red interval followed by green
                                red = max(min_red_time, headway - green_time)
                                    
                                metering_rates[m]["next-red-"+lane] = step + round( (red + green_time) / stepsize)
                                metering_rates[m]["next-green-"+lane] = step + round( (red) / stepsize)
                                
                                # turn off green light
                                metering_rates[m]["green-active-"+lane] = False
                                
                                
                                
                                otherlane = str(1-idx + 2)
                                
                                
                                # offset second meter by 50%
                                # take into account existing red status of meter
                                if metering_rates[m]["on"] == True:
                                    red = headway * 0.5 - green_time
                                else:
                                    red = headway * 1.5 - green_time
                                
                                metering_rates[m]["next-red-"+otherlane] = step + round( (red + green_time) / stepsize)
                                metering_rates[m]["next-green-"+otherlane] = step + round( (red) / stepsize)
                                
                                if red < 0:
                                    # turn on green light
                                    metering_rates[m]["green-active-"+otherlane] = True
                                
                                metering_rates[m]["on"] = True
                                
                                if test:
                                    print("recalc-all", m, round(step * stepsize, 3), "new meter times", rate)
                                    print("\t lane 1", round(metering_rates[m]["next-green-1"]*stepsize, 3), round(metering_rates[m]["next-red-1"]*stepsize, 3))
                                    print("\t lane 2", round(metering_rates[m]["next-green-2"]*stepsize, 3), round(metering_rates[m]["next-red-2"]*stepsize, 3))
                                    
                            else:
                                # only recalculate this meter next red and green time
                                # red interval followed by green
                                red = max(min_red_time, headway - green_time)
                                    
                                metering_rates[m]["next-red-"+lane] = step + round( (red + green_time) / stepsize)
                                metering_rates[m]["next-green-"+lane] = step + round( (red) / stepsize)
                                
                                # turn off green light
                                metering_rates[m]["green-active-"+lane] = False
                                
                                # still in red phase; green is not active
                                
                                if test:
                                    print("recalc", m, "lane", lane, round(step * stepsize, 3), "new meter times", rate)
                                    print("\t lane 1", round(metering_rates[m]["next-green-1"]*stepsize, 3), round(metering_rates[m]["next-red-1"]*stepsize, 3))
                                    print("\t lane 2", round(metering_rates[m]["next-green-2"]*stepsize, 3), round(metering_rates[m]["next-red-2"]*stepsize, 3))
                        
                    # set traffic signal
                    phase = 0
                    
                    if metering_rates[m]["on"] == False:
                        traci.lane.setDisallowed(lanes[m], ['passenger']) # Only 1 lane used when meter off
                        traci.trafficlight.setPhase(m, 0)
                    else:
                        traci.lane.setDisallowed(lanes[m], []) # Now 2 lines form for meter
                        
                        if metering_rates[m]["green-active-1"] == True:
                            if metering_rates[m]["green-active-2"] == True:
                                traci.trafficlight.setPhase(m, 0)
                            else:
                                traci.trafficlight.setPhase(m, 2)
                                phase = 2
                        else:
                            if metering_rates[m]["green-active-2"] == True:
                                traci.trafficlight.setPhase(m, 3)
                                phase = 3
                            else:
                                traci.trafficlight.setPhase(m, 1)
                                phase = 1
                                
                    
                    
                    if test and metering_rates[m]["phase"] != phase:
                        print("time", round(step * stepsize, 3), m, "active phase", phase, "meter on", metering_rates[m]["on"])
                        
                    metering_rates[m]["phase"] = phase    
                
            traci.simulationStep() # Progress Sim by 1 time step       
            step += 1
            
 
    traci.close()
    sys.stdout.flush()
    
    if not test:
        socket.close()
    
    return(passage1_count,demand1_count,meter1Activate,flush1Activate,stopped1Activate,min1Rates,max1Rates,rates1,passage2_count,demand2_count,meter2Activate,flush2Activate,stopped2Activate,min2Rates,max2Rates,rates2)
    
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

    
def processMessage(message):
    print("received \""+message+"\"");
    
    s = message.split(",")
    
    if s[0] == "meter-rate":
        meter = s[1][s[1].index("-")+1]
        rate = s[2]
        if rate == "null":
            rate = -1 # change this, I'm not sure what "null" rate is
        else:
            rate = int(rate)
            
        return meter, rate

    
    # this contains the metering rate!
    
def computeRed(newRate):
    
    # CONVERT RELEASE RATE TO RED TIME
    C = 3600 / newRate # Convert Release rate to a cycle time
    redTime = round(C - greenTime)
    redTime = max(redTime,2) # Dont have red less than 2 seconds
    
    #print('lastRate:',lastRate,'minRate',minRate,'maxRate',maxRate,'segDen',segDen)
    print('NewRate',newRate, 'redTime',redTime)
    
    return redTime, newRate
    
def computeRedFlush(maxRate):
    # Computes rate during flushing phase, just use the max Rate
    if maxRate <=0: # Make sure max rate is not negative
        maxRate = 200
    
    maxRate = maxRate * 1.2 # During flush phase max rate is 150% not 125% of track demand
    
    C = 3600 / maxRate
    greenTime = 3
    redTime = round(C - greenTime)
    redTime = max(redTime, 2) # Dont have red less than 2 seconds
    
    print('Flush Rate Update!')
    print('FlushRate:', maxRate, 'FlushRedTime:', redTime)
    
    return redTime

