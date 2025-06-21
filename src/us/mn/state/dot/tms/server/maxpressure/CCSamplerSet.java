/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package us.mn.state.dot.tms.server.maxpressure;

import java.util.TreeMap;
import us.mn.state.dot.tms.server.SamplerSet;
import us.mn.state.dot.tms.server.VehicleSampler;

/**
 * I need to track cumulative counts.
 * @author michael
 */
public class CCSamplerSet implements VehicleSampler {
    
    private SamplerSet sampler;

    // this depends on length and could be different for each location
    private long max_lookback;
    
    private TreeMap<Long, Integer> counts;
    
    public CCSamplerSet(SamplerSet s, long max_lookback, long stamp){
        sampler = s;
        
        counts = new TreeMap<>();
        resetCC(stamp);
    }
    
    
    
    // it may be helpful to reset the cumulative count periodically to 0. Otherwise, it could grow to infinity.
    public void resetCC(long stamp){
        counts.clear();
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
            // these should always be non-null: there should be 0 stored as a baseline, and by definition this is less than the highest key and not exactly equal to it
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
