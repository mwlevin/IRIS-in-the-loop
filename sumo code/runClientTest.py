# Initialization Stuff
from __future__ import absolute_import
from __future__ import print_function
import os
import sys
import optparse
import traci
import xml.etree.ElementTree as ET
import numpy as np
import pandas as pd


# ################################################## give me an alarm when the code finish running
# from IPython.display import Audio, display
# #################################################


'''
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
'''

from sumolib import checkBinary
def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options

# SHOW SIM GUI OR NOT: IF want GUI set first line to 'sumo' and second to 'sumo-gui' and vice versa
options = get_options()

options.nogui = True

if options.nogui:
    sumoBinary = checkBinary('sumo')
else:
    sumoBinary = checkBinary('sumo-gui')
 
# Import density_main function and wait time function
from clientTest import run
from waitTime import waitTimes

# Fixed parameters
control = True
adjust = 0 # If 0, will run with default settings
rampStorageLength = 180
rampStorageDensity = 20/180
stepsize = '.05'
# site = ["site3_fullHV.sumocfg","site3_fullACC.sumocfg","site3_fullAV.sumocfg","site3_highHV.sumocfg","site3_highACC.sumocfg","site3_highAV.sumocfg"]
# site = ['highACC.sumocfg','highAV.sumocfg']

#directory = "test_network"
#site = [directory+'/new_network.sumocfg']
directory = "610EB"
site = [directory+'/'+directory+'.sumocfg']

if adjust == 0:
    # output_prefix = ['fullHV','fullACC','fullAV','highHV','highACC','highAV']
    output_prefix = ['fullHV_A2']

elif adjust == 1: 
    output_prefix = ['fullHV_A1','fullACC_A1','fullAV_A1','highHV_A1','highACC_A1','highAV_A1']
else:
    output_prefix = ['fullHV_A2','fullACC_A2','fullAV_A2','highHV_A2','highACC_A2','highAV_A2']



for i in range(len(site)):
    
    if adjust == 0: # Default Settings
        critDensity = 51.67
        jamDensity = 192.58
        alpha_desired = 0.9
        alpha_low = 0.75
       
    else: # Adjusted Settings - 
        if output_prefix[i] == 'fullHV_A2': #(no adjustment)
            critDensity = 51.67
            jamDensity = 192.58
            alpha_desired = 0.9
            alpha_low = 0.75            

        elif output_prefix[i] == 'fullACC_A2':
            critDensity = 22.88 
            jamDensity = 142.42
            alpha_desired = 0.65
            alpha_low = 0.54            

        elif output_prefix[i] == 'fullAV_A2':
            critDensity =  46.17
            jamDensity = 214.57
            alpha_desired = 1.25
            alpha_low = 1.04            

        elif output_prefix[i] == 'highHV_A2':
            critDensity = 41.53 
            jamDensity = 173.63 
            alpha_desired = 0.78
            alpha_low = 0.65         

        elif output_prefix[i] == 'highACC_A2':
            critDensity = 33.91 
            jamDensity = 160.13 
            alpha_desired = 0.73
            alpha_low = 0.61            
        elif output_prefix[i] == 'highAV_A2':  #highAV_A1
            critDensity = 44.02 #
            jamDensity = 190.67 
            alpha_desired = 0.95
            alpha_low = 0.79 
        
    print("starting TraCI")
    traci.start([sumoBinary,"--step-length",stepsize, "-c", site[i], "--tripinfo-output","tripinfo.xml", "--statistic-output","statistics.xml", "--duration-log.statistics","--output-prefix",output_prefix[i]]) 
    
    
    print("running simulation")
    # passage_count,demand_count,meterActivate,flushActivate,stoppedActivate,minRates,maxRates,rates = run(control,critDensity,jamDensity,rampStorageLength,alpha_desired,alpha_low)
    #passage1_count,demand1_count,meter1Activate,flush1Activate,stopped1Activate,min1Rates,max1Rates,rates1,passage2_count,demand2_count,meter2Activate,flush2Activate,stopped2Activate,min2Rates,max2Rates,rates2 = run(control,critDensity,jamDensity,rampStorageLength,alpha_desired,alpha_low)
    run(directory, control,critDensity,jamDensity,rampStorageLength,alpha_desired,alpha_low)

    # Mainline Density [veh/mi] : From .xml File not Traci retrieval
    fieldL = 16.4042
    tree = ET.parse(output_prefix[i]+'detUp1.xml')
    root = tree.getroot()
    interval_OccsUp1 = [0]
    interval_time_steps2 = [0]
    for interval in root.iter('interval'):
        interval_Occ = float(interval.attrib['occupancy'])
        time_step = float(interval.attrib['end'])
        interval_OccsUp1.append(interval_Occ)
        interval_time_steps2.append(time_step)
    interval_OccsUp1 = np.array(interval_OccsUp1)

    tree = ET.parse(output_prefix[i]+'detUp2.xml')
    root = tree.getroot()
    interval_OccsUp2 = [0]
    interval_time_steps2 = [0]
    for interval in root.iter('interval'):
        interval_Occ = float(interval.attrib['occupancy'])
        time_step = float(interval.attrib['end'])
        interval_OccsUp2.append(interval_Occ)
        interval_time_steps2.append(time_step)
    interval_OccsUp2 = np.array(interval_OccsUp2)

    interval_OccsUp = (interval_OccsUp1 + interval_OccsUp2)/2


    tree = ET.parse(output_prefix[i]+'detDown1.xml')
    root = tree.getroot()
    interval_OccsDown1 = [0]
    interval_time_steps2 = [0]
    for interval in root.iter('interval'):
        interval_Occ = float(interval.attrib['occupancy'])
        time_step = float(interval.attrib['end'])
        interval_OccsDown1.append(interval_Occ)
        interval_time_steps2.append(time_step)
    interval_OccsDown1 = np.array(interval_OccsDown1)

    tree = ET.parse(output_prefix[i]+'detDown2.xml')
    root = tree.getroot()
    interval_OccsDown2 = [0]
    interval_time_steps2 = [0]
    for interval in root.iter('interval'):
        interval_Occ = float(interval.attrib['occupancy'])
        time_step = float(interval.attrib['end'])
        interval_OccsDown2.append(interval_Occ)
        interval_time_steps2.append(time_step)
    interval_OccsDown2 = np.array(interval_OccsDown2)

    interval_OccsDown = (interval_OccsDown1 + interval_OccsDown2)/2

    upDen = ((interval_OccsUp/100) * 5280) / fieldL #[veh/mile]
    downDen = ((interval_OccsDown/100) * 5280) / fieldL #[veh/mile]

    segDen = (upDen + downDen) / 2# [veh/mi]

    # Total Travel Times [s]
    tree = ET.parse(output_prefix[i]+'mainTravelTimeTotalLong.xml')
    root = tree.getroot()
    totalTravelTime = []
    for interval in root.iter('interval'):
        totalTravelTime.append(float(interval.attrib['meanTravelTime']))
        
    # Throughput [veh]
    tree = ET.parse(output_prefix[i]+'mainTravelTimeTotalLong.xml')
    root = tree.getroot()
    throughput = []
    for interval in root.iter('interval'):
        throughput.append(float(interval.attrib['vehicleSum']))


    ## CALC INTERESTING VALUES TO STORE
    # Queue Length on ramp 1
    queueLength1 = np.array(demand1_count)-np.array(passage1_count)
    avgQueueLength1 = np.average(queueLength1)
    maxQueueLength1 = np.max(queueLength1)

    # Queue Length on ramp 2
    queueLength2 = np.array(demand2_count)-np.array(passage2_count)
    avgQueueLength2 = np.average(queueLength2)
    maxQueueLength2 = np.max(queueLength2)

    # Same for both ramps:
    maxAllowQueueLength = rampStorageDensity*rampStorageLength*2

    #Mainline Density
    avgDensity = np.average(segDen)
    
    # Queue Wait Times on ramp 1
    avg_wait1,max_wait1 = waitTimes(demand1_count,passage1_count)
    max_allow_wait1 = 240
    
    # Queue Wait Times on ramp 2
    avg_wait2,max_wait2 = waitTimes(demand2_count,passage2_count)
    max_allow_wait2 = 240
    
    ## SAVE IMPORTANT DATA##
    # Define Data for all excel tabs
    simInfo = {'critDensity[veh/mi]':[critDensity],'jamDensity[veh/mi]':[jamDensity],
                'meter1Activate':np.array(meter1Activate)*float(stepsize),
                'flush1Activate':np.array(flush1Activate)*float(stepsize),
                'stopped1Activate':np.array(stopped1Activate)*float(stepsize),
                'meter2Activate':np.array(meter2Activate)*float(stepsize),
                'flush2Activate':np.array(flush2Activate)*float(stepsize),
                'stopped2Activate':np.array(stopped2Activate)*float(stepsize),
                'mainLineTT[s]':totalTravelTime,'throughput[veh]':throughput}
    rampInfo = {'Time[s]':interval_time_steps2,'cumDemand1':demand1_count,
                'cumPassage1':passage1_count,'queueLength1':queueLength1,
                'avgLength1':[avgQueueLength1],'maxLength1':[maxQueueLength1],
                'maxAllowLenth:':[maxAllowQueueLength],'avgWait1':[avg_wait1],
                'maxWait1':[max_wait1],'maxAllowWait1':[max_allow_wait1]
                ,'cumDemand2':demand2_count,
                'cumPassage2':passage2_count,'queueLength2':queueLength2,
                'avgLength2':[avgQueueLength2],'maxLength2':[maxQueueLength2],'avgWait2':[avg_wait2],
                'maxWait2':[max_wait2],'maxAllowWait2':[max_allow_wait2]}

    rateInfo = {'Time[s]':interval_time_steps2,'minRate1[veh/hr]':min1Rates,'maxRate1[veh/hr]':max1Rates,
                  'ReleaseRate1[veh/hr]':rates1,'minRate2[veh/hr]':min2Rates,'maxRate2[veh/hr]':max2Rates,
                  'ReleaseRate2[veh/hr]':rates2,'MainlineDensity[veh/mi]':segDen,
                  'avgDensity':[avgDensity]}
    # Create an Excel writer object
    writer = pd.ExcelWriter(output_prefix[i]+'output.xlsx', engine='xlsxwriter')
    # Create a DataFrame for each tab's data
    simInfo = pd.DataFrame.from_dict(simInfo, orient='index').transpose()
    rampInfo = pd.DataFrame.from_dict(rampInfo, orient='index').transpose()
    rateInfo= pd.DataFrame.from_dict(rateInfo, orient='index').transpose()
    # Write dataframes to their respective tabs
    simInfo.to_excel(writer, sheet_name='simInfo', index=False)
    rampInfo.to_excel(writer, sheet_name='rampInfo', index=False)
    rateInfo.to_excel(writer, sheet_name='ratesInfo', index=False)
    # Save the Excel file
    writer.close()

