/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package us.mn.state.dot.tms.server.maxpressure;

import us.mn.state.dot.tms.server.MaxPressureAlgorithm;



public class Cell {
    protected double n, y;
    private CTMLink link;
    
    protected double cc;

    public Cell(CTMLink link){
        this.link = link;
        this.n = 0;
        this.y = 0;
    }

    public double getSendingFlow()
    {
        return Math.min(n, link.Q * MaxPressureAlgorithm.CTM_DT/3600.0);
    }

    public double getReceivingFlow()
    {
        double actual_Q = link.Q;
        
        
        // capacity drop
        
        if(getDensity() > link.getCriticalDensity()){
            //System.out.println("activating capacity drop "+link.start.getName()+" "+getDensity()+" "+link.getCriticalDensity());
            actual_Q = link.Q * 0.70;
        }
        
        
        //System.out.println("\t\t"+n+ " "+link.w+" "+link.v+" "+link.K +" "+link.cell_len+" "+(link.cell_len * link.K)
        //       +" "+ (link.w / link.v * (link.K * link.cell_len - n)));
        double term1 = actual_Q * MaxPressureAlgorithm.CTM_DT/3600.0;
        double term2 = link.w / link.v * (getMaxOccupancy() - n);
        //System.out.println("R check "+term1+" "+term2);
        return Math.min(term1, term2);
    }
    
    public double getMaxOccupancy(){
        return link.K * link.cell_len;
    }

    public void addFlow(double y)
    {
        this.y += y;
    }

    public void removeFlow(double y)
    {
        this.y -= y;
    }


    public void update()
    {
        n += y;
        y = 0;
        
        cc += y;
    }

    public double getDensity(){
        return n / link.cell_len;
    }
}
