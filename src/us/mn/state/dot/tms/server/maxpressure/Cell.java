/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package us.mn.state.dot.tms.server.maxpressure;

import us.mn.state.dot.tms.server.MaxPressureAlgorithm;



public class Cell {
    protected double n, y;
    private CTMLink link;

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
        //System.out.println("\t\t"+n+ " "+link.w+" "+link.v+" "+link.K +" "+link.cell_len+" "+(link.cell_len * link.K)
        //       +" "+ (link.w / link.v * (link.K * link.cell_len - n)));
        return Math.min(link.Q * MaxPressureAlgorithm.CTM_DT/3600.0, link.w / link.v * (getMaxOccupancy() - n));
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
    }

    public double getDensity(){
        return n / link.cell_len;
    }
}
