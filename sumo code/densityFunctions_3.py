def computeSwitchMeter(upOcc,downOcc,critDensity,alpha_desired,alpha_low):
    # Computes conditon (T or F) if we need to switch to metering
    
    # CONSTANTS
    fieldL = 16.4042 # Field length (vehicleL + detL) [ft]

    # desiredDen = 0.9 * critDensity # 90% of critDensity [veh/mi/ln] (original)

    # # --------############# for adjustment 2 (comment out in adjustment 1)--------#########
    desiredDen = alpha_desired*critDensity # [veh/mi/ln] (original)

    # # --------############# for adjustment 2 (comment out in adjustment 1)--------#########
    
    #GET VALUES FROM THE LAST 2 MINUTES
    twoMinUpOcc = upOcc[-5:] # All entires from past 2 minutes
    twoMindownOcc = downOcc[-5:] 
    
    # GET TWO MINUTE AVERAGE VALUES
    upOcc = sum(twoMinUpOcc) / len(twoMinUpOcc)
    downOcc = sum(twoMindownOcc) / len(twoMindownOcc)
    
    # CONVERT OCCUPANCY TO DENSITY
    upDen = ((upOcc/100) * 5280) / fieldL #[veh/mile]
    downDen = ((downOcc/100) * 5280) / fieldL #[veh/mile]
    
    # COMPUTE SEGMENT DENSITY
    segDen = (upDen + downDen) / 2# [veh/mi]
    
    # CALC IF WE SWITCH METER ON
    if segDen >= desiredDen and (len(twoMinUpOcc) >= 5): # Must consider at least 2 minuts
        switch = 'True'
    else:
        switch = 'False'
        
    return switch

def computeSwitchFlush(upOcc, downOcc,critDensity,alpha_desired,alpha_low):
    # Computes conditon (T or F) if we need to switch to flush phase
    
    # CONSTANTS
    fieldL = 16.4042 # Field length (vehicleL + detL) [ft]

    # lowDen = 0.75 * critDensity # 75% of critDensity [veh/mi/ln] (original)
    
    # # --------############# for adjustment 2 (comment out in adjustment 1)--------#########
    lowDen = alpha_low*critDensity # [veh/mi/ln] (original)
    # # --------############# for adjustment 2 (comment out in adjustment 1)--------#########
    
    #GET VALUES FROM THE LAST 10 MINUTES
    tenMinUpOcc = upOcc[-21:] # All entires from past 10 minutes
    tenMindownOcc = downOcc[-21:] 
    
    # GET 10 MINUTE AVERAGE VALUES
    upOcc = sum(tenMinUpOcc) / len(tenMinUpOcc)
    downOcc = sum(tenMindownOcc) / len(tenMindownOcc)
    
    
    # CONVERT OCCUPANCY TO DENSITY
    upDen = ((upOcc/100) * 5280) / fieldL #[veh/mile]
    downDen = ((downOcc/100) * 5280) / fieldL #[veh/mile]
    
    # COMPUTE SEGMENT DENSITY
    segDen = (upDen + downDen) / 2# [veh/mi]
    
    # CALC IF WE SWITCH FLUSH
    if segDen <= lowDen and (len(tenMinUpOcc) >= 21): # Must consider at least 10 minuts
        switch = 'True'
    else:
        switch = 'False'
        
    return switch
    
    
def computeSwitchMeterStopped(upOcc,downOcc,critDensity,alpha_desired,alpha_low):
    # Computes conditon (T or F) if we need to switch to metering
    
    # CONSTANTS
    fieldL = 16.4042 # Field length (vehicleL + detL) [ft]
    
    # desiredDen = 0.9 * critDensity # 90% of critDensity [veh/mi/ln] (original)
    
    # # --------############# for adjustment 2 (comment out in adjustment 1)--------#########
    desiredDen = alpha_desired*critDensity # [veh/mi/ln] (original)

    # # --------############# for adjustment 2 (comment out in adjustment 1)--------#########

    #GET VALUES FROM THE LAST 5 MINUTES
    fiveMinUpOcc = upOcc[-11:] # All entires from past 5 minutes
    fiveMindownOcc = downOcc[-11:] 
    
    # GET TWO MINUTE AVERAGE VALUES
    upOcc = sum(fiveMinUpOcc) / len(fiveMinUpOcc)
    downOcc = sum(fiveMindownOcc) / len(fiveMindownOcc)
    
    
    # CONVERT OCCUPANCY TO DENSITY
    upDen = ((upOcc/100) * 5280) / fieldL #[veh/mile]
    downDen = ((downOcc/100) * 5280) / fieldL #[veh/mile]
    
    # COMPUTE SEGMENT DENSITY
    segDen = (upDen + downDen) / 2# [veh/mi]
    
    # CALC IF WE SWITCH METER ON
    if segDen >= desiredDen and (len(fiveMinUpOcc) >= 11): # Must consider at least 5 minutes
        switch = 'True'
    else:
        switch = 'False'
        
    return switch


def calcRateLimits(demandCount,passageCount,rampLength):
    # Computes the rate limits : NOTE rampLength is length of storage [m] for vehicles in line at meter. Depends on site

    
    ## TRACKING DEMAND - avg demand over the past 5 minutes ##
    fiveMinDemandValues = demandCount[-11:] # All counts over the last 5 minutes
    time_period = (len(fiveMinDemandValues) - 1) * 30 # [s] Usually 5 minutes, but less at the start when we dont have 5 min of data
    AvgVeh = fiveMinDemandValues[-1] - fiveMinDemandValues[0]# [veh] number of vehicles over past 5 minutes
    track_demand = (AvgVeh * 3600) / time_period # [veh/hr]
    print('Tracking Demand:', track_demand)
    
    ## MAX RATE - 125% of tracking demand ##
    maxRate = track_demand * 1.25
    
    ## MIN RATE (3 differnt opitons, use the max) ##
    
    # DEMAND MIN RATE (75% of tracking demand) #
    minRateDemand = track_demand * 0.75
    
    # WAIT TIME MIN RATE #
    maxWait = 240 #[s] Can change this, max time a vehicle should wait in line
    samplePeriod = 30 #[s] How often we sample
    targetWait = maxWait - (2*samplePeriod) #[s] account for measurment uncertainity (for input and output)
    numIntervals = int(targetWait / samplePeriod) # Number of intervals within the target wait time
    
    # Check all time periods where queue hasnt cleared yet
    currentPassage = passageCount[-1] # Current cummulative vehicles released
    periodIndices = [i for i, num in enumerate(demandCount) if num > currentPassage] # Get indices where demand is higher than current passage count (check these for wait time)
    periodIndices = periodIndices[::-1] # Reverse order of list (check most recent entries first)
    periodIndices = periodIndices[:numIntervals]# Only check intervals within the target wait time
    
    maxSlope = 0 # Initialize the slope (rate) to 0
    backstep = 0 # How many intervals back from present. Every interval back we get 30 seconds less time to clear the queue!
    for index in periodIndices:
        rise = demandCount[index] - currentPassage # How many vehicles we must clear
        run = targetWait-(backstep*30) # How long we have to clear those vehicles
        slope = rise/run # The rate associated with the values above
        backstep += 1
        if slope > maxSlope: # Update maxSlope (check all relevant time intervals)
            maxSlope = slope
    
    minRateWait = maxSlope*3600 # [veh/s] to [veh/hr]
    
    
    # STORAGE MIN RATE # 
    
    storageDensity = 20 / 180 # [veh/m] for HVs. Must change if diff veh type on ramp
    numStorageLanes = 2 #2 lines form on ramp
    totalStorage = storageDensity * rampLength * numStorageLanes # [veh]
    targetStorage = 0.75 * totalStorage # target is 75% of total
    
    # Subtract target storage from accumulated demand
    extra = max(0,demandCount[-1] - targetStorage)
    
    # Project to target wait time using tracking demand
    projStorage = extra + (targetWait/3600) * track_demand #[veh]
    
    # Calc storage limit by finding rate from current passage to proejct storage
    rise = projStorage - passageCount[-1]
    run = targetWait
    
    minRateStorage = (rise/run) *3600 # [veh/s] to [veh/hr]

    ## ENSURE MAX RATE DOES NOT LEAD TO LESS THAN 5s cycle time
    absMax = 720 # veh/s, abs max rate for 5s cycle time
    maxRate = min(maxRate,absMax)

    ## CHOOSE LARGEST MINIMUM RATE TO USE##
    minRate = max(minRateDemand,minRateStorage,minRateWait)
    
    ## ENSURE MIN RATE IS LESS THAN MAX RATE
    minRate = min(maxRate-10,minRate) 
    
    
    print('minRateDemand:', minRateDemand)
    print('minRateWait:', minRateWait)
    print('minRateStorage:', minRateStorage)
    print('Max Rate:',maxRate)
    
    return maxRate, minRate


def computeRed(lastRate,upOcc,downOcc,minRate,maxRate,critDensity,jamDen,alpha_desired,alpha_low):
    # Computes the updated red time
    print('RATE UPDATE!')
    
    # CONSTANTS
    fieldL = 16.4042 # Field length (vehicleL + detL) [ft]
    # desiredDen = 0.9*critDensity # [veh/mi/ln] (original)
# # --------############# for adjustment 2 (comment out in adjustment 1)--------#########
    desiredDen = alpha_desired*critDensity # [veh/mi/ln] (original)

# # --------############# for adjustment 2 (comment out in adjustment 1)--------#########

    greenTime = 3 # [s] constant, allows 1 veh to pass 
    

    # CONSTRAIN LAST RATE WITHIN LIMITS!
    if lastRate <= minRate:
        lastRate = minRate
        
    if lastRate >= maxRate:
        lastRate = maxRate
        

    # CONVERT OCCUPANCY TO DENSITY
    upDen = ((upOcc/100) * 5280) / fieldL #[veh/mile]
    downDen = ((downOcc/100) * 5280) / fieldL #[veh/mile]
    
    # COMPUTE SEGMENT DENSITY
    segDen = (upDen + downDen) / 2# [veh/mi]
    
    
    # CONSTRAIN SEGMENT DENSITY WITHIN LIMITS
    if segDen > jamDen:
        segDen = jamDen
    
    # CALUCATE NEW RELEASE RATE (Linear interpolation)
    
    if segDen <= desiredDen:
        x1 = 0
        y1 = maxRate
        x2 = desiredDen
        y2 = lastRate
        x = segDen
        
        newRate = y1 + ((x-x1)* ((y2-y1)/(x2-x1)))
        
    else:
        x1 = desiredDen
        y1 = lastRate
        x2 = jamDen
        y2 = minRate
        x = segDen
        
        newRate = y1 + ((x-x1)* ((y2-y1)/(x2-x1)))
        
    # Ensure Rate is positive
    if newRate <= 0:
        newRate = 200
    
    
    # CONVERT RELEASE RATE TO RED TIME
    C = 3600 / newRate # Convert Release rate to a cycle time
    redTime = round(C - greenTime)
    redTime = max(redTime,2) # Dont have red less than 2 seconds
    
    print('lastRate:',lastRate,'minRate',minRate,'maxRate',maxRate,'segDen',segDen)
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

