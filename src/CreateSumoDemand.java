
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
    public static void main(String[] args) throws IOException{
        PrintStream fileout = new PrintStream(new File("sumo code/610/demand.rou.xml"));
        
        
        printHeader(fileout);
        
        
        
        
        
        
        processFile(new File("sumo code/610/test.txt"), fileout, 0, 3600);
        
        
        
        
        
        printFooter(fileout);
        fileout.close();
        
        
    }
    
    public static void processFile(File input, PrintStream fileout, int begin, int end) throws IOException {
        Scanner filein = new Scanner(input);
        
        HashMap<String, Double> entrances = new HashMap<>();
        double total = 0;
        double carry = 0;
     
        int flow_id = 1;
        
        filein.nextLine(); // header line
        
        while(filein.hasNext()){
            String name = filein.next();
            String type = filein.next();
            
            double count = 0;
            
            if(filein.hasNextDouble()){
                count = filein.nextDouble();
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
                    int vph = (int)Math.round(exitcount * entrances.get(r)/total);
                    
                    if(vph > 0){
                        fileout.println("<flow id=\"HV_"+flow_id+"\" type=\"HV_M\" begin=\""+String.format("%.2f", (double)begin)+"\" departLane=\"free\" departSpeed=\"avg\" from=\""+r+"\" to=\""+name+"\" end=\""+String.format("%.2f", (double)end)+"\" vehsPerHour=\""+String.format("%.2f", (double)vph)+"\"/>");
                    }
                    
                    entrances.put(r, entrances.get(r) - vph);
                    
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
