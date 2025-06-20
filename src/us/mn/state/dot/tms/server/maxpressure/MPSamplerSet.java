/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package us.mn.state.dot.tms.server.maxpressure;

import us.mn.state.dot.tms.server.SamplerSet;
import us.mn.state.dot.tms.server.VehicleSampler;

/**
 * I need to track cumulative counts.
 * @author michael
 */
public class MPSamplerSet implements VehicleSampler {
    
    private SamplerSet sampler;
    
    
    public MPSamplerSet(SamplerSet s){
        sampler = s;
    }
    
    
    public void updateCumulativeCount(){
        
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
