/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package us.mn.state.dot.tms.server.maxpressure;

import java.util.List;
import us.mn.state.dot.tms.server.R_NodeImpl;


/**
 * I need to track cumulative counts. This is a wrapper around a SamplerSet to avoid changing SamplerSet code directly.
 * @author michael
 */
public abstract class SimNode {
    private R_NodeImpl rnode;
    
    public SimNode(R_NodeImpl rnode){
        this.rnode = rnode;
    }
    
    public abstract void setMainlineOut(CTMLink out);
    
    public R_NodeImpl getRnode(){
        return rnode;
    }
    
    public String getName(){
        return rnode.getName();
    }
    
    public String toString(){
        return rnode.getName();
    }
    public abstract void step();
    

    public abstract void propagateExcessRemovedFlow(double y);
    
    // attempt to find the CTMLink for the mainline
    public abstract CTMLink getMainlineIn(); 
    public abstract CTMLink getMainlineOut(); 
}
