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
    
    phase = 'notStarted' # Initial Phase
    phase1 = 'notStarted' # Initial Phase
    phase2 = 'notStarted' # Initial Phase
    
    
    connection = socket.socket()
    host = "localhost"
    port = 5452
    
    print("connecting to IRIS server ", host, port);
    connection.connect((host, port))
    


    endStep = 72000 # Time to simulate (0.05 seconds steps)

    # this is hardcoded for now
    meters = ["J3", "J9"]
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
                sendMessage(connection, msg)
        
        # wait for meter rates
        for i in range(0, len(meters)):
            message = readLine(connection)
            processMessage(message)
            
        
        # progress sim by time_period (probably 30sec)
        for i in range(0, round(time_period/stepsize)):
            traci.simulationStep() # Progress Sim by 1 time step       
            step += 1
            
 
    traci.close()
    sys.stdout.flush()
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
            rate = 3000 # change this, I'm not sure what "null" rate is
        else:
            rate = int(rate)
    
    # this contains the metering rate!

