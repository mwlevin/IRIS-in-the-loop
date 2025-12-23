/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package us.mn.state.dot.tms.server.maxpressure;

import us.mn.state.dot.tms.server.R_NodeImpl;

/**
 *
 * @author michael
 */
public class DummyNode extends SimNode {

    public DummyNode(R_NodeImpl rnode) {
        super(rnode);
    }
    
    public void propagateExcessRemovedFlow(double y){
        // do nothing
    }
 
    public void setMainlineOut(CTMLink out){
        // do nothing
    }
    public CTMLink getMainlineOut(){
        return null;
    }
    
    public CTMLink getMainlineIn(){
        return null;
    }
    
    public double removeFlowCleanup(double y){
        return 0;
    }
    
    public double addFlowCleanup(double y){
        return 0;
    }
    
    public void cleanup(){}
    
    public void step(){}
}
