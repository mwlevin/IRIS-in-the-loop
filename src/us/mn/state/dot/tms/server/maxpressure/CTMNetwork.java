/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package us.mn.state.dot.tms.server.maxpressure;

import java.util.List;
import us.mn.state.dot.tms.server.MaxPressureAlgorithm;
import static us.mn.state.dot.tms.server.MaxPressureAlgorithm.CTM_DT;
import static us.mn.state.dot.tms.server.MaxPressureAlgorithm.STEP_SECONDS;


/**
 * I need to track cumulative counts. This is a wrapper around a SamplerSet to avoid changing SamplerSet code directly.
 * @author michael
 */
public class CTMNetwork {
    
    protected static final double EPSILON = 1e-6; // catch numerical errors where something is close to 0 but not quite
    
    public List<SimNode> nodes;
    public List<SimLink> links;
    
    public MergeNode center_merge; // merge that I am modeling with this sim
    
    public CTMNetwork(MergeNode center_merge, List<SimNode> nodes, List<SimLink> links){
        this.nodes = nodes;
        this.links = links;
        
        this.center_merge = center_merge;
    }
    
    public boolean isDownstreamCongested(){
        return center_merge.out.getDensity() > center_merge.out.getCriticalDensity();
    }
    
    
    
    
    public double getTotalOccupancy(){
        
        double total = 0;
        
        for(SimLink l : links){
            total += l.getOccupancy();
        }
        
        return total;
    }

    public double getDetOccupancy(long stamp, int PER_MS){
        
        double inc = 0;
        double out = 0;
        for(SimLink l : links){
            if(l instanceof EntranceLink){
                inc += ((EntranceLink)l).getDetCumulativeCount();
            }
            else if(l instanceof ExitLink){
                out += ((ExitLink)l).getDetCumulativeCount();
            }
        }
        
        return inc - out;
    }
    
    
    public double getUpstreamSendingFlow(){
        double total = 0;

        CTMLink inc = center_merge.inc_mainline;
        
        int ncells = (int)Math.round(STEP_SECONDS / CTM_DT);
        
        do{
            int n = Math.min(ncells, inc.cells.length);
            
            for(int i = 0; i < n; i++){
                total += inc.cells[inc.cells.length-1-i].n;
            }
            
            ncells -= n;
            
            if(ncells == 0){
                break;
            }
            else{
                // continue looking down the mainline
                SimNode next = inc.start;
                
                inc = next.getMainlineIn();
            }
        }
        while(true);
        
        return total;
    }
    
    public double getUpstreamOccupancy(){
        double total = 0;
        
        CTMLink l = center_merge.inc_mainline;
        
        while(l != null){
            total += l.getOccupancy();
            
            l = l.start.getMainlineIn();
        }
        return total;
    }
    
    public double getDownstreamOccupancy(){
        double total = 0;
        
        CTMLink l = center_merge.out;
        
        while(l != null){
            total += l.getOccupancy();
            
            l = l.end.getMainlineOut();
        }
        
        return total;
    }
    
    public double getDownstreamReceivingFlow(){
        // estimated value of receiving flow * number of CTM time steps
        return center_merge.out.getReceivingFlow() * MaxPressureAlgorithm.STEP_SECONDS / MaxPressureAlgorithm.CTM_DT;
    }
    
    
    public double getDownstreamAvgDensity(){
        return center_merge.out.getAvgDensity();
    }
    
    public double getUpstreamAvgDensity(){
        return center_merge.inc_mainline.getAvgDensity();
    }
    
    // calculate max-pressure weight for incoming link
    public double getUpstreamWeight(boolean print){

        double look_len = center_merge.inc_mainline.v * MaxPressureAlgorithm.STEP_SECONDS / 3600.0; // this could be (should be) less than link length
        // remaining length to evaluate
        double rem_len = look_len;

        double output = 0;

        CTMLink link = center_merge.inc_mainline;
        
        outer: do{

            
            for(int i = link.cells.length-1; i >= 0; i--){
                double end = rem_len;
                double start = Math.max(0, rem_len - link.cell_len); // in case look_len is not divisible by cell_len

                // integrate x/L * k dx from start to end
                double k = link.cells[i].getDensity();
                
                double integral = (end*end / 2 - start*start / 2) / look_len * k;
                
                if(print)
                System.out.println("check upstream weight "+String.format("%.2f", start)+" "+String.format("%.2f", end)+" "+String.format("%.2f", k)+" "+look_len+" "+String.format("%.2f", integral));
                
                output += integral;

                rem_len -= link.cell_len;

                // stop when we have evaluated the entire length
                if(rem_len <= EPSILON){
                    break outer;
                }
                
            }
            
            if(rem_len <= EPSILON){
                break;
            }
            else{
                link = link.start.getMainlineIn();
            }
        }
        while(link != null);

        return output;
    }
    
    
    // calculate max-pressure weight for downstream link
    public double getDownstreamWeight(boolean print){

        double look_len = center_merge.out.v * MaxPressureAlgorithm.STEP_SECONDS / 3600.0; // this could be (should be) less than link length

        double carry_len = 0;
        
        double output = 0;

        CTMLink link = center_merge.out;
        
        outer: do{
            for(int i = 0; i < link.cells.length; i++){
                double start = link.cell_len * i + carry_len;
                double end = Math.min(link.cell_len * (i+1) + carry_len, look_len);

                // integrate (L-x)/L * k dx from start to end
                // = [Lx - x^2/2] / L * k = x * k - x^2/2/L * k
                double k = link.cells[i].getDensity();
                double integral = (end - start) * k - (end*end/2 - start*start/2) / look_len * k;
                
                if(print)
                    System.out.println("check downstream weight "+String.format("%.2f", start)+" "+String.format("%.2f", end)+" "+String.format("%.2f", k)+" "+look_len+" "+String.format("%.2f", integral));
                output += integral;;

                // stop after reaching look_len
                if(look_len - (link.cell_len * (i+1) + carry_len) <= CTMNetwork.EPSILON){
                    break outer;
                }
                
            }
            
            // if we have to go to the next link, then the starting point is farther
            carry_len += link.cells.length * link.cell_len;
            
            if(look_len - carry_len > EPSILON){
                link = link.end.getMainlineOut();
            }
        }
        while(link != null);

        return output;
    }

    public String toString(){
        // print network for testing purposes
        String after_merge = "";
        String before_merge = "";
        
        SimNode curr = center_merge.getMainlineIn().start;
        
        while(curr != null){
            SimLink prev = curr.getMainlineIn();
            
            before_merge = "\t"+curr.toString()+"\n"+before_merge;
            
            if(prev != null){
                curr = prev.start;
            }
            else{
                break;
            }
        }
        

        
        after_merge = "\t center "+center_merge.toString();
        
        curr = center_merge.getMainlineOut().end;
        
        while(curr != null){
            SimLink next = curr.getMainlineOut();
            
            after_merge = after_merge+"\n"+"\t"+curr.toString();
            
            if(next != null){
                curr = next.end;
            }
            else{
                break;
            }
        }
        
        
        
        return before_merge+after_merge;
    }
     /**
    * This updates the CTM model for the last time step (defined by STEP_SECONDS)
    * This may involve multiple CTM time steps of simulation, defined by CTM_DT
    */
   public void simulateLastTimestep(long stamp, int PERIOD_MS){
        int num_steps = (int)Math.round(STEP_SECONDS / CTM_DT);

        //String before_occ = "before occ "+getTotalOccupancy()+" "+getDetOccupancy(stamp-PERIOD_MS, PERIOD_MS);

        for(SimLink l : links){
            l.prepare(stamp, PERIOD_MS);
        }

        double ent = 0;
        double exit = 0;
       
        
       
        for(int i = 0; i < num_steps; i++){
            for(SimLink l : links){
                l.step();
            }

            for(SimNode n : nodes){
                n.step();
            }

            for(SimLink l : links){
                l.update();
            }
             
            
            //System.out.println("after occ "+i+" "+String.format("%.2f", getTotalOccupancy()));
            /*
            for(SimLink l : links){
                System.out.println("\t"+l.getOccupancy()+" "+l+" "+l.getClass().getName());
            }
            */

        }
       
        // clean up process
        // 1) propagate excess removed flow backwards
        // 2) move excess entering flow to mainline if possible
        
        // 0 link should be downstream link
        // ExitLink propagateExcessRemovedFlow ignores passed number
        links.get(0).propagateExcessRemovedFlow(0);
        
        for(SimLink l : links){
            if(l instanceof EntranceLink){
                ((EntranceLink)l).propagateExcessAddedFlow();
            }
        }
        

        /*
        if(Math.abs(getTotalOccupancy() - getDetOccupancy(stamp, PERIOD_MS)) > 0.001 ){
            System.out.println(before_occ);
            System.out.println("change = +"+ent+" -"+exit);
            
            for(SimLink l : links){
                if(l instanceof EntranceLink){
                    ent += ((EntranceLink)l).entered;
                }
                else if(l instanceof ExitLink){
                    exit += ((ExitLink)l).exited;
                }
            }

            
            for(SimLink l : links){
                if(l instanceof EntranceLink){
                    EntranceLink en = (EntranceLink)l;
                    System.out.println("\t"+en.getName()+" "+en.getDetCumulativeCount()+" delta="+en.entered);
                }
            }
            
            System.out.println("\t---");
            
            for(SimLink l : links){
                if(l instanceof ExitLink){
                    ExitLink ex = (ExitLink)l;
                    System.out.println("\t"+ex.getName()+" "+ex.getDetCumulativeCount()+" delta="+ex.exited);
                }
            }


            System.out.println("after occ "+String.format("%.2f", getTotalOccupancy())+" "+getDetOccupancy(stamp, PERIOD_MS));
            System.out.println("\n");
            System.out.println(toString());
            
            
            for(SimLink l : links){
                System.out.println("\t"+l.getOccupancy()+" "+l+" "+l.getClass().getName());
            }
            
            System.exit(1);
            
        }
        */
       


   }

}
