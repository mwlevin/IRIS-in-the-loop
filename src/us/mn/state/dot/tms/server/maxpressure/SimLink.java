/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package us.mn.state.dot.tms.server.maxpressure;

/**
 *
 * @author michael
 */
public abstract class SimLink {
    protected SimNode start, end;

    
    public SimLink(){
        Q = 0;
        L = 0;
    }
    
    public String toString(){
        return start.getName()+"-"+end.getName();
    }
    
    protected double Q; // capacity
    protected double L; // length
    
    public abstract void step();
    public abstract void update();
    
    public abstract double getOccupancy();
    
    public abstract double getReceivingFlow();
    public abstract double getSendingFlow();
    
    public abstract void addFlow(double y);
    public abstract void removeFlow(double y);
    

    public abstract void propagateExcessRemovedFlow(double y);
    
    
    public abstract void prepare(long stamp, int PERIOD_MS); // used to obtain sensor data for next time period (usually multiple CTM time steps)
}
