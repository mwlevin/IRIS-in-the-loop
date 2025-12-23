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
public class DivergeNode extends SimNode {
    private CTMLink inc;
    private CTMLink out_mainline;
    private ExitLink out_ramp;
    
    public DivergeNode(R_NodeImpl rnode){
        super(rnode);
    }
    public DivergeNode(R_NodeImpl rnode, CTMLink inc, ExitLink out_ramp){
        super(rnode);
        this.inc = inc;
        this.out_ramp = out_ramp;
        
        inc.end = this;
        out_ramp.start = this;
    }
    
    public void setMainlineOut(CTMLink out){
        out_mainline = out;
        out.start = this;
    }
    
    public String toString(){
        return getName()+" [->"+out_ramp.end.getName()+"] L="+inc.L;
    }
    
    public CTMLink getMainlineIn(){
        return inc;
    }
    
    public CTMLink getMainlineOut(){
        return out_mainline;
    }
    
    public void propagateExcessRemovedFlow(double y){
        y += out_ramp.queue;
        out_ramp.queue = 0;
        inc.propagateExcessRemovedFlow(y);
    }
    
    public void step(){
        // no diverge model here: out_ramp tells us how much flow exits
        double y_exit = out_ramp.getReceivingFlow();
        
        double S = Math.max(0, inc.getSendingFlow()-y_exit);
        double R = out_mainline.getReceivingFlow();
        
        double y = Math.min(S, R);
        inc.removeFlow(y + y_exit);
        out_mainline.addFlow(y);
        
        /*
        if(y_exit > 0){
            System.out.println("\t\tdiverge "+out_ramp.getName()+" remove "+y_exit+" against "+y_exit);
        }
        */
    }
    
}
