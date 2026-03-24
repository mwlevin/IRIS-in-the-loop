
import java.io.File;
import java.io.IOException;
import java.io.PrintStream;
import java.util.HashMap;
import java.util.Scanner;

/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */

/**
 *
 * @author michael
 */
public class CreateSumoDemand {
    public static void main(String network) throws IOException{
        PrintStream fileout = new PrintStream(new File("sumo code/"+network+"/demand.rou.xml"));
        
        
        printHeader(fileout);
        
        
        
        
        double scale = 1;
        
        processFile(new File("sumo code/"+network+"/counts1.txt"), fileout, 0, 300, scale);
        processFile(new File("sumo code/"+network+"/counts2.txt"), fileout, 300, 600, scale);
        processFile(new File("sumo code/"+network+"/counts3.txt"), fileout, 600, 900, scale);
        processFile(new File("sumo code/"+network+"/counts4.txt"), fileout, 900, 1200, scale);
        processFile(new File("sumo code/"+network+"/counts5.txt"), fileout, 1200, 1500, scale);
        processFile(new File("sumo code/"+network+"/counts6.txt"), fileout, 1500, 1800, scale);
        processFile(new File("sumo code/"+network+"/counts7.txt"), fileout, 1800, 2100, scale);
        processFile(new File("sumo code/"+network+"/counts8.txt"), fileout, 2100, 2400, scale);
        processFile(new File("sumo code/"+network+"/counts9.txt"), fileout, 2400, 2700, scale);
        processFile(new File("sumo code/"+network+"/counts10.txt"), fileout, 2700, 3000, scale);
        processFile(new File("sumo code/"+network+"/counts11.txt"), fileout, 3000, 3300, scale);
        processFile(new File("sumo code/"+network+"/counts12.txt"), fileout, 3300, 3600, scale);
        //processFile(new File("sumo code/"+network+"/EB.txt"), fileout, 0, 3600, scale);
        
        
        
        
        printFooter(fileout);
        fileout.close();
        
        
    }
    
    public static int flow_id = 1;
    
    public static void processFile(File input, PrintStream fileout, int begin, int end, double scale) throws IOException {
        Scanner filein = new Scanner(input);
        
        HashMap<String, Double> entrances = new HashMap<>();
        double total = 0;
        double carry = 0;
     
        
        filein.nextLine(); // header line
        
        while(filein.hasNext()){
            String name = filein.next();
            String type = filein.next();
            
            double count = 0;
            
            
            
            if(filein.hasNextDouble()){
                count = filein.nextDouble()*scale;
            }
            else{
                count = carry * 0.2; // if no data, assume 20% vehicles enter or exit, could be changed later
            }
            
            filein.nextLine();
            
            if(type.equalsIgnoreCase("entrance")){
                carry += count;
                total += count;
                entrances.put(name, count);
            }
            else if(type.equalsIgnoreCase("exit")){
                
                double exitcount = Math.min(count, carry);
                
                for(String r : entrances.keySet()){
                    double actual_count = (exitcount * entrances.get(r)/total);
                    double vph = actual_count / ( (end-begin) /3600.0);
                    
                    //System.out.println(r+" "+name+" "+vph);

                    
                    if(vph > 0){
                        fileout.println("<flow id=\"HV_"+flow_id+"_0\" type=\"HV_M\" begin=\""+String.format("%.2f", (double)begin)+"\" departLane=\"0\" departSpeed=\"avg\" from=\""+r+"\" to=\""+name+"\" end=\""+String.format("%.2f", (double)end)+"\" vehsPerHour=\""+String.format("%.2f", (double)vph/2.0)+"\"/>");
                        fileout.println("<flow id=\"HV_"+flow_id+"_1\" type=\"HV_M\" begin=\""+String.format("%.2f", (double)begin)+"\" departLane=\"1\" departSpeed=\"avg\" from=\""+r+"\" to=\""+name+"\" end=\""+String.format("%.2f", (double)end)+"\" vehsPerHour=\""+String.format("%.2f", (double)vph/2.0)+"\"/>");
                        flow_id ++;
                    }
                    
                    
                    entrances.put(r, entrances.get(r) - actual_count);
                    
                }
                
                carry -= exitcount;
                
            }
        }
        
        filein.close();
    }
    
    public static void printHeader(PrintStream fileout){
        fileout.println("<?xml version=\"1.0\" encoding=\"UTF-8\"?>");


        fileout.println("<routes xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:noNamespaceSchemaLocation=\"http://sumo.dlr.de/xsd/routes_file.xsd\">");
        fileout.println("<vType id=\"HV_M\" length=\"5.00\" minGap=\"3.40\" maxSpeed=\"44.10\" speedFactor=\"normc(1.00,0.10,0.20,2.00)\" color=\"yellow\" lcAssertive=\"5\" carFollowModel=\"IDM\" accel=\"1.06\" decel=\"1.11\" tau=\"1.26\" delta=\"4\"/>");
    }
    
    public static void printFooter(PrintStream fileout){
        fileout.println("</routes>");
    }
}
