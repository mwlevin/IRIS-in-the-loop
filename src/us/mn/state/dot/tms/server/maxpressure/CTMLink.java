/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package us.mn.state.dot.tms.server.maxpressure;

import us.mn.state.dot.tms.server.MaxPressureAlgorithm;
import static us.mn.state.dot.tms.server.maxpressure.CTMNetwork.EPSILON;

public class CTMLink extends SimLink {
    protected double K; // jam density
    protected double w; // congested wave speed
    
    protected double v; // free flow speed
    protected int lanes;

    protected double cell_len;

    protected Cell[] cells;


    public CTMLink(double L, int lanes, double v, double Q, double w, double K){
        this.L = L;
        this.v = v;
        this.Q = Q;
        this.lanes = lanes;
        this.w = w;
        this.K = K;

        // want cell length to be approximately v * dt
        cell_len = v * MaxPressureAlgorithm.CTM_DT / 3600;
        int ncells = Math.max(1, (int)Math.ceil(L / (v * MaxPressureAlgorithm.CTM_DT / 3600.0)));
        // minimum of 1 cell

        System.out.println("length check "+ncells+" "+cell_len+" "+L);

        cells = new Cell[ncells];

        for(int i = 0; i < cells.length; i++){
            cells[i] = new Cell(this);
        }
    }
    
    public void propagateExcessRemovedFlow(double y){
        for(int i = cells.length-1; i >= 0; i--){
            double removed = Math.min(y, cells[i].n);
            cells[i].n -= removed;
            
            y -= removed;
            
            if(y < EPSILON){
                break;
            }
        }
        
        // even if y=0, keep going because the start node may be a diverge with an exit link
        start.propagateExcessRemovedFlow(y);
    }
    
    public double getOccupancyChange(){
        return 0;
    }
    
    public double getDensity(){
        return getOccupancy() / (cell_len * cells.length);
    }
    
    public double getAvgDensity(){
        return getDensity()/lanes;
    }
    
    public double cleanupAddFlow(double y){
        double total_added = 0;
        
        /*
        double attempt = y;
        double before_occ = getOccupancy();
        */
        
        for(int i = 0; i < cells.length; i++){
            double added = Math.min(y, cells[i].getMaxOccupancy() - cells[i].n);
            cells[i].n += added;
            
            total_added += added;
            y -= added;
            
            if(y < EPSILON){
                break;
            }
        }
        
        /*
        double after_occ = getOccupancy();
        System.out.println("cleanup added "+total_added+" of "+attempt+" "+before_occ+" "+after_occ);
        */
        
        
        return total_added;
    }
    
    public int getNumCells(){
        return cells.length;
    }

    public double getOccupancy(){
        double total_n = 0;

        for(Cell c : cells){
            total_n += c.n;
        }

        return total_n;
    }
    
    public void prepare(long stamp, int PERIOD_MS){
        // nothing to do here
    }

    

    public void addFlow(double y){
        cells[0].n += y;
    }


    public void removeFlow(double y){
        cells[cells.length-1].n -= y;
    }

    
    
    public double getCleanupMaxAdd(){
        double total = 0;
        
        for(Cell c : cells){
            total += c.getMaxOccupancy() - c.n;
        }
        return total;
    }
    
    public double getCleanupMaxRemove(){
        double total = 0;
        
        for(Cell c : cells){
            total += c.n;
        }
        return total;
    }
    

    

    // sending flow for next CTM timestep
    // units of veh
    public double getSendingFlow(){
        return cells[cells.length-1].getSendingFlow();
    }



    // receiving flow for next CTM timestep
    // units of veh
    public double getReceivingFlow(){
        return cells[0].getReceivingFlow();
    }
    
    public double getCriticalDensity(){
        return Q / v;
    }

    // calculate state at next CTM time step
    public void step(){
        //System.out.println("CTM step");
        for(int i = 1; i < cells.length; i++){
            double S = cells[i-1].getSendingFlow();
            double R = cells[i].getReceivingFlow();
            double y = Math.max(0, Math.min(S, R)); // in case it becomes negative due to sensor fault

            //System.out.println("\t cell check "+i+" "+S+" "+R+" "+cells[i-1].n+" "+(Q*CTM_DT/3600));
            cells[i].addFlow(y);
            cells[i-1].removeFlow(y);
        }
    }

    // set state to state at next time step
    public void update(){
        for(int i = 0; i < cells.length; i++){
            cells[i].update();
        }
    }
    
   
}
