/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package us.mn.state.dot.tms.server.maxpressure;

import java.util.List;
import us.mn.state.dot.tms.server.MaxPressureAlgorithm;
import us.mn.state.dot.tms.server.R_NodeImpl;

/**
 *
 * @author michael
 */
public class SeriesNode extends SimNode {
    protected SimLink inc, out;
    
    public SeriesNode(R_NodeImpl rnode){
        super(rnode);
    }
    
    public SeriesNode(R_NodeImpl rnode, CTMLink inc, ExitLink out){
        super(rnode);
        this.inc = inc;
        this.out = out;
        inc.end = this;
        out.start = this;
    }
    public SeriesNode(R_NodeImpl rnode, EntranceLink inc){
        super(rnode);
        this.inc = inc;
        this.out = out;
        inc.end = this;
        
    }
    
    public void propagateExcessRemovedFlow(double y){
        inc.propagateExcessRemovedFlow(y);
    }
    
    public String toString(){
        if(inc instanceof EntranceLink){
            return ((EntranceLink)inc).getName()+" [entrance]\n\t"+super.toString()+" [series]";
        }
        else if(out instanceof ExitLink){
            return super.toString()+" [series] L="+inc.L+"\n\t"+
                    ((ExitLink)out).getName()+" [exit]";
        }
        else{
            return super.toString();
        }
    }
    
    public void setMainlineOut(CTMLink out){
        this.out = out;
        out.start = this;
    }
    
    
    
    
    public CTMLink getMainlineIn(){
        if(inc instanceof CTMLink){
            return (CTMLink)inc;
        }
        else{
            // this may be null at the start of the network where out is CTMLink and inc is 
            return null;
        }
    }
    
    public CTMLink getMainlineOut(){
        if(out instanceof CTMLink){
            return (CTMLink)out;
        }
        else{
            // this may be null at the start of the network where inc is CTMLink and out is ExitLink
            return null;
        }
    }
    
    public void step(){
        double S = inc.getSendingFlow();
        double R = out.getReceivingFlow();
        
        double y = 0;
        
        if(out instanceof ExitLink){
            y = R;
        }
        else{
            y = Math.min(S, R);
        }
        inc.removeFlow(y);
        out.addFlow(y);
        
        /*
        if(y > 0){
            if(inc instanceof EntranceLink){
                System.out.println("\t\tseries "+((EntranceLink)inc).getName()+" is adding "+(y)+" flow against "+S);
            }
            else if(out instanceof ExitLink){
                System.out.println("\t\tseries "+((ExitLink)out).getName()+" remove "+(y)+" flow against "+R);
            }
        }
        */
        
    }
    
    
}
