/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package us.mn.state.dot.tms.server.maxpressure;

import static us.mn.state.dot.tms.server.MaxPressureAlgorithm.CTM_DT;
import static us.mn.state.dot.tms.server.MaxPressureAlgorithm.STEP_SECONDS;
import us.mn.state.dot.tms.server.R_NodeImpl;
import us.mn.state.dot.tms.server.SamplerSet;

/**
 *
 * @author michael
 */
public class ExitLink extends SimLink {
    protected SamplerSet det; 
    
    private double cc;
    public double exited;
    public double actual_exited;
    
    private double R_base;
    protected double queue; // vehicles to remove that have not been removed
    
    public ExitLink(R_NodeImpl det_rnode, SamplerSet det){
        end = new DummyNode(det_rnode);
        this.det = det;
        
        
        Q = 10000; // ramp capacity
        // this should be ramp capacity * number of lanes
        // I'm going to ignore this part of the ramp and instead model it as unlimited exit, except limited by sensor count
    }
    
    public String toString(){
        return end.getName();
    }
    
    public void prepare(long stamp, int PERIOD_MS){
        exited = det.getVehCount(stamp, PERIOD_MS);
        actual_exited = 0;
        
        
        if(exited >= 0){
            
            cc += exited;
        }
        else{
            exited = 0;
            System.out.println("exit missing det data "+end.getName());
        }
        
        R_base = exited * CTM_DT / STEP_SECONDS;
    }
    
    public double getOccupancyChange(){
        return -R_base;
    }
    
    public String getName(){
        return end.getName();
    }
    public double getDetCumulativeCount(){
        return cc;
    }
    
    public void update(){
    }
    
    public void step(){
        // nothing to do here
    }
    
    public double getReceivingFlow(){
        queue += R_base;
        // queue may not exit uniformly over time
        return queue;
    }
    
    public double getSendingFlow(){
        return 0;
    }
    
    
    
    public double getCleanupMaxAdd(){
        return queue;
    }
    
    public double getCleanupMaxRemove(){
        return 0;
    }

    
    public void propagateExcessRemovedFlow(double y){
        // ignore y. This will never be called by downstream link (none exists by definition)
        // Propagate queue
        
        if(queue > 0){
            //System.out.println("exit-cleanup "+queue);
        
            start.propagateExcessRemovedFlow(queue);
        }
        queue = 0;
    }
    
    public double getOccupancy(){
        return -queue; // should not have occupancy. 
        // After every STEP_SECONDS all exiting flow (as recorded by sensor) is removed.
    }
    
    public void addFlow(double y){
        // add everything. Excess is added to remove_carry
        actual_exited += y;
        queue -= y;
    }
    
    public void removeFlow(double y){
        // do nothing
    }


}
