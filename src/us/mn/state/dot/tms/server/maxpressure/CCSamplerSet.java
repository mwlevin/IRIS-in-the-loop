/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package us.mn.state.dot.tms.server.maxpressure;

import java.util.TreeMap;
import us.mn.state.dot.tms.server.SamplerSet;
import us.mn.state.dot.tms.server.VehicleSampler;

/**
 * I need to track cumulative counts. This is a wrapper around a SamplerSet to avoid changing SamplerSet code directly.
 * @author michael
 */
public class CCSamplerSet implements VehicleSampler {
    
    private SamplerSet sampler;

    // this depends on length and could be different for each location
    private long max_lookback;
    
    private TreeMap<Long, Integer> counts;
    
    /**
     * I need to reset this periodically otherwise the counts will go to infinity.
     * Ideally we want to reset it at a time where no one is around (e.g. 3am)
     * reset interval is in ms
     * offset is to determine time-of-day
     * timestamp is in GMT, we are GMT-5 or GMT-6, so 3am is 8 or 9 hours after start-of-day
     */
    private static final int RESET_INTERVAL = 1000*3600*24; // every 24 hours
    private static final int RESET_OFFSET = 1000*3600*8; 
    
    public CCSamplerSet(SamplerSet s, long max_lookback, long stamp){
        sampler = s;
        
        counts = new TreeMap<>();
        counts.put(stamp, 0);
    }
    
    
    
    
    
    // this may be asked for historical counts, and those counts need to be stored and retrieved
    // this may be non-integer due to interpolation
    public double getCumulativeCount(long stamp){
        
        // nothing stored: return 0
        // this case should not occur
        if(counts.isEmpty()){
            counts.put(stamp, 0);
            return 0;
        }
        
        long last_update = counts.lastKey();
        
        // if we passed the reset interval, then reset
        if(stamp % RESET_INTERVAL >= RESET_OFFSET && last_update % RESET_INTERVAL < RESET_OFFSET)
        {
            counts.clear();
            // last_update becomes reset time
            last_update = stamp - (stamp % RESET_INTERVAL - RESET_OFFSET);
            counts.put(last_update, 0);
            
            // now continue with remainder of cc (the first case, stamp > last_update, should happen)
        }
        
        if(stamp > last_update){
            int passed = sampler.getVehCount(stamp, (int)(stamp - last_update));
            
            int cc = passed;
           
            int last = counts.get(last_update);
            cc += last;
            
            
            // store CC
            counts.put(stamp, cc);
            
            // check if I can remove the front to avoid the storage of counts from growing infinitely
            long first = counts.firstKey();
            if(last_update - first > max_lookback){
                counts.remove(first);
            }
            
            return cc;
        }
        // if exactly stored, then return its value
        else if(counts.containsKey(stamp)){
            return counts.get(stamp);
        }
        else{
            // linear interpolation between stored points
            // upper should never be null
            // lower could be null after a reset
            Long lower = counts.lowerKey(stamp);
            Long upper = counts.higherKey(stamp);
            
            if(lower == null){
                return 0;
            }
            
            int lowerN = counts.get(lower);
            int upperN = counts.get(upper);
            
            
            return ((double)(stamp-lower))/(upper-lower) * (upperN - lowerN) + lowerN;
        }
    }
    
    public int getVehCount(long stamp, int per_ms) {
        return sampler.getVehCount(stamp, per_ms);
    }
    
    public float getSpeed(long stamp, int per_ms) {
        return sampler.getSpeed(stamp, per_ms);
    }
    
    public float getMaxOccupancy(long stamp, int per_ms) {
        return sampler.getMaxOccupancy(stamp, per_ms);
    }
    
    public float getDensity(long stamp, int per_ms) {
        return sampler.getDensity(stamp, per_ms);
    }
    
    public int getFlow(long stamp, int per_ms) {
        return sampler.getFlow(stamp, per_ms);
    }
    
    public boolean isPerfect() {
        return sampler.isPerfect();
    }
}
