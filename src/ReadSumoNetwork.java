
import java.io.File;
import java.io.IOException;
import java.sql.ResultSet;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Properties;
import java.util.Scanner;
import java.util.Set;
import java.util.Stack;
import us.mn.state.dot.tms.R_NodeType;
import us.mn.state.dot.tms.TMSException;
import us.mn.state.dot.tms.server.ResultFactory;
import us.mn.state.dot.tms.server.SQLConnection;
import us.mn.state.dot.tms.utils.DevelCfg;
import us.mn.state.dot.tms.utils.PropertyLoader;

/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */

/**
 *
 * @author michael
 */
public class ReadSumoNetwork {
    static private final String PROP_FILE =
		DevelCfg.get("server.prop.file",
			"etc/iris/iris-server.properties");
    
    public static void main(String[] args) throws Exception {
        readSumo("site_trial.net.xml", "detectors.xml");
    }
    
    public static void readSumo(String netfile, String detectorfile) throws Exception{
        Properties props = PropertyLoader.load(PROP_FILE);
        
        SQLConnection store = createStore(props);
        
        
        store.update("DELETE FROM iris.device_preset;");
        store.update("DELETE FROM iris.controller_io;");
        store.update("DELETE FROM iris.beacon;");
        store.update("DELETE FROM iris.detector;");
        store.update("DELETE FROM iris._detector;");
        store.update("DELETE FROM iris.controller;");
        store.update("DELETE FROM iris.r_node;");
        store.update("DELETE FROM iris.ramp_meter;");
        
        store.update("DELETE FROM iris.geo_loc;");
        store.update("DELETE from iris.road");
        
        
        store.update("INSERT INTO iris.road VALUES ('mainline0', 'm0', 6, 0);");
        store.update("INSERT INTO iris.road VALUES ('mainline1', 'm1', 6, 1);");
        store.update("INSERT INTO iris.road VALUES ('none', 'none', 0, 2);");
        
        int drop_id = 1;
        
        boolean created_controller = false;
        
        String controllername = "sumo_ctl";
        
        drop_id++;
        
        int cont_pin = 0;
        
        Map<String, Object[]> junctions = new HashMap<>();
        Map<String, Object[]> lanes = new HashMap<>();
        Map<String, Object[]> edges = new HashMap<>();
        
        
        int pin = 0;
        int preset = 0;
        
        // read network file
        Scanner filein = new Scanner(new File(netfile));
        
        while(filein.hasNextLine()){
            String line = filein.nextLine();
            
            if(line.indexOf("<junction ") >= 0){
                String name = findVar("id", line);
                double x = Double.parseDouble(findVar("x", line));
                double y = Double.parseDouble(findVar("y", line));
                String type = findVar("type", line);
                
                junctions.put(name, new Object[]{name, null, null, type});
            }
            else if(line.indexOf("<edge ") >= 0){
                String name = findVar("id", line);
                String from = findVar("from", line);
                
                // ignore the internal function junctions
                if(from == null){
                    continue;
                }
                String to = findVar("to", line);
                double length = Double.parseDouble(findVar("length", line));
                
                edges.put(name, new Object[]{name, from, to, length, 1, null});
            }
            else if(line.indexOf("<lane ") >= 0){
                String name = findVar("id", line);
                double length = Double.parseDouble(findVar("length", line));

                lanes.put(name, new Object[]{name, null, null, length, null, 0});
            }
        }
        
        filein.close();
        
        
        
        
        
        // I need to redo the positions for the junctions based on distances
        // therefore: mainline has x=0 and ramps are 90 degrees
        // arbitrarily pick one junction to be (0,0).
        // one of the priority junctions should be (0,0)
        
        Stack<String> unsettled = new Stack<>();
        
        for(String name : junctions.keySet()){
            if(junctions.get(name)[3].equals("priority")){
                junctions.get(name)[1] = 0.0;
                junctions.get(name)[2] = 0.0;
                unsettled.push(name);
                break;
            }
        }
        
        while(!unsettled.isEmpty()){
            String junc = unsettled.pop();
            
            
            
            for(String name : edges.keySet()){
                String from = (String)edges.get(name)[1];
                String to = (String)edges.get(name)[2];
                
                String typeFrom = (String)junctions.get(from)[3];
                String typeTo = (String)junctions.get(to)[3];

                boolean ramp = isRamp(typeFrom, typeTo);
                
                if(ramp){
                    edges.get(name)[5] = "ramp";
                }
                else{
                    edges.get(name)[5] = "mainline";
                }

                if(from.equals(junc)){
                    Object[] data2 = junctions.get(to);
                    
                    if(data2[1] != null){
                        continue;
                    }
                    
                    if(data2[3].equals("dead_end")){
                        data2[3] = "exit";
                    }
                    
                    if(ramp){
                        junctions.get(to)[2] = junctions.get(from)[2];
                        junctions.get(to)[1] = (double)junctions.get(from)[1] + (double)edges.get(name)[3];
                    }
                    else{
                        junctions.get(to)[1] = 0.0;
                        junctions.get(to)[2] = (double)junctions.get(from)[2] - (double)edges.get(name)[3];
                    }
                    
                    unsettled.push(to);
                }
                else if(to.equals(junc)){
                    Object[] data2 = junctions.get(from);
                    
                    if(data2[1] != null){
                        continue;
                    }
                    
                    if(data2[3].equals("dead_end")){
                        data2[3] = "entrance";
                    }
                    
                    if(ramp){
                        junctions.get(from)[2] = junctions.get(to)[2];
                        junctions.get(from)[1] = (double)junctions.get(to)[1] - (double)edges.get(name)[3];
                    }
                    else{
                        junctions.get(from)[1] = 0.0;
                        junctions.get(from)[2] = (double)junctions.get(to)[2] + (double)edges.get(name)[3]; 
                    }
                    
                    unsettled.push(from);
                }
            }
        }
        
        
        
        for(String edge : edges.keySet()){
            int maxlaneid = 0;
            
            while(lanes.containsKey(edge+"_"+maxlaneid)){
                String lanename = edge+"_"+maxlaneid;
                lanes.get(lanename)[4] = edges.get(edge)[5];
                lanes.get(lanename)[1] = edges.get(edge)[1];
                lanes.get(lanename)[2] = edges.get(edge)[2];
                lanes.get(lanename)[5] = maxlaneid+1;
                
                maxlaneid++;
            }
            
            edges.get(edge)[4] = maxlaneid;
        }
        
        Map<String, String> createdNodes = new HashMap<>();
        
        
        String saved_loc = "";
        
        
        int nodekey = 0;
        
        int speedlimit = 60;
        
        for(String name : junctions.keySet()){

            
            double x = (double)junctions.get(name)[1];
            double y = (double)junctions.get(name)[2];
            String type = (String)junctions.get(name)[3];
            
            String locname = "loc-"+name+"-0";
            String locname2 = "loc-"+name+"-1";
            
            saved_loc = locname;
            
            
            
            
            String beaconname = "beacon-"+name;
            
            String stationname = "st-"+name;

            String cross = "none";
            
            for(String edge : edges.keySet()){
                if(edges.get(edge)[1].equals(name) &&  edge.indexOf("Start") > 0 && edges.get(edge)[5].equals("ramp")){
                    //System.out.println(edge);
                    cross = edge.substring(0, edge.indexOf("Start"));
                }
                else if(edges.get(edge)[2].equals(name) &&  edge.indexOf("Start") > 0 && edges.get(edge)[5].equals("ramp"))
                {
                    cross = edge.substring(0, edge.indexOf("Start"));
                    
                    
                }
            }
            
            if(!cross.equals("none")){
                ResultCounterFactory counter = new ResultCounterFactory();
                store.query("SELECT * FROM iris.road WHERE name='"+cross+"';", counter);

                if(counter.length == 0){
                    store.update("INSERT INTO iris.road VALUES ('"+cross+"', '"+cross+"', 6, 2);");
                }
            }
            

            // create geo_loc
            store.update("INSERT INTO iris.geo_loc VALUES ('"+locname+"', '', 'mainline0', '0', '"+cross+"', '2', '0', '', "+x+", "+y+");"); 
            store.update("INSERT INTO iris.geo_loc VALUES ('"+locname2+"', '', 'mainline1', '1', '"+cross+"', '2', '0', '', "+x+", "+y+");"); 
            
            if(!created_controller){
                store.update("INSERT INTO iris.controller VALUES ('"+controllername+"', "+drop_id+", 'sumo', 'sumo_fake', '"+saved_loc+"', 1, '', 'meter', 'true', 'true', '2100-Jan-01');");
                created_controller = true;
            }
            
            
            int node_type = 0; 
            
            
            
            if(type.equals("exit")){
                node_type = 2;
            }
            else if(type.equals("entrance")){
                node_type = 1;
            }
            else if(type.equals("traffic_light")){
                node_type = 1;
            }
            else{
                node_type = 0;
            }
            
            int numlanes = 1; 
            
            for(String edge : edges.keySet()){
                if(edges.get(edge)[1].equals(name) || edges.get(edge)[2].equals(name)){
                    numlanes = (int)Math.max(numlanes, (int)edges.get(edge)[4]);
                }
            }
            
            
            
            String nodename = "n-"+name;
             
            store.update("INSERT INTO iris.r_node VALUES ('"+nodename+"', '"+locname+"', "+nodekey+", "+node_type+", 'true', 'false', 0, "+numlanes+", 'true', "+0+", 'true', '"+stationname+"', "+speedlimit+", '');");

            nodekey++;
            
            createdNodes.put(nodename, locname);
  
            
            // use offset for lane
            
            
            
            
            if(type.equals("traffic_light")){

                
                //store.update("INSERT INTO iris.controller VALUES ('"+controllername+"', "+drop_id+", 'sumo', 'sumo_fake', '"+locname+"', 1, '', 'meter', 'true', 'true', '2100-Jan-01');");
                drop_id++;


                pin++;
                store.update("INSERT INTO iris.beacon VALUES ('"+beaconname+"', '"+locname+"', '"+controllername+"', "+(pin++)+", '', '', 0, 'true', '"+preset+"', 0);");
                preset++;
                store.update("INSERT INTO iris.ramp_meter VALUES ('meter-"+name+"', '"+locname+"', '"+controllername+"', "+(pin++)+", '', 2, 100, 240, 3, 1800, 1800, '"+beaconname+"', '"+preset+"', 'false', '{}');");
                preset++;
            }
                
		// create r node
            
        }
        
        
        
        // then read detectors file
        
        
        
        
        filein = new Scanner(new File(detectorfile));
        
        
        
        
        while(filein.hasNextLine()){
            String line = filein.nextLine();
            
            if(line.indexOf("<inductionLoop ") >= 0){
                String name = findVar("id", line);
                double period = Double.parseDouble(findVar("period", line));
                
                if(period != 30.0){
                    continue;
                }
                
                name = name.replaceAll("Demand", "Dem"); // name is too long!
                name = name.replaceAll("Down", "Do"); // name is too long!
                name = name.replaceAll("Pass", "Pa"); // name is too long!
                name = name.replaceAll("Green", "Gr"); // name is too long!
                name = name.replaceAll("Merge", "Me"); // name is too long!
                
                String lane = findVar("lane", line);
                String edge = lane.substring(0, lane.indexOf("_"));
                double pos = Double.parseDouble(findVar("pos", line));
                
                String nodeid = edge+"_"+pos;
                
                
                String loc = name;
                
                if(createdNodes.containsKey(nodeid)){
                    loc = createdNodes.get(nodeid);
                }
                
                String from = (String)lanes.get(lane)[1];
                boolean ramp = lanes.get(lane)[4].equals("ramp");
                
                int numlanes = (int)edges.get(edge)[4];
                
                char det_type = '0';
                
                String stationname = "st-"+loc;
                String nodename = "n-"+loc;
                String locname = "l-"+loc;
                int node_type = 0;
                
                double x = (double)junctions.get(from)[1];
                double y = (double)junctions.get(from)[2];
                
                if(ramp){
                    x += pos;
                }
                else{
                    y -= pos;
                }
                
                String cross = "none";
                
                if(edge.indexOf("Start") >= 0){
                    cross = edge.substring(0, edge.indexOf("Start"));
                    node_type = 1;
                    
                    if(name.indexOf("Dem") >= 0){
                        det_type = 'Q';
                    }
                    else if(name.indexOf("Gr") >= 0){
                        det_type = 'G';
                    }
                    
                    nodename = "n-"+edges.get(edge)[2];
                }
                else if(edge.indexOf("End") >= 0){
                    cross = edge.substring(0, edge.indexOf("End"));
                    node_type = 1;
                    
                    
                    if(name.indexOf("Pa") >= 0){
                        det_type = 'P';
                    }
                    else if(name.indexOf("Me") >= 0){
                        det_type = 'M';
                    }
                    else if(name.indexOf("Gr") >= 0){
                        det_type = 'G';
                    }
                    
                    nodename = "n-"+edges.get(edge)[1];
                    
                }
                
                if(!createdNodes.containsKey(nodeid) && !createdNodes.containsKey(nodename)){
                    
                    
                    store.update("INSERT INTO iris.geo_loc VALUES ('"+locname+"', '', 'mainline0', '0', '"+cross+"', '2', '0', '', "+x+", "+y+");");
                    nodekey++;

                    
                    store.update("INSERT INTO iris.r_node VALUES ('"+nodename+"', '"+locname+"', "+nodekey+", "+node_type+", 'true', 'false', 0, "+numlanes+", 'true', "+0+", 'true', '"+stationname+"', "+speedlimit+", '');");
                    
                    createdNodes.put(nodeid, loc);
                }
                
                
                int laneno = (int)lanes.get(lane)[5];
                

                //String controllername = "det_ctl_"+name;
                String detname = name;

                


                store.update("INSERT INTO iris.detector VALUES ('"+detname+"', '"+controllername+"', "+(pin++)+", '"+nodename+"', '"+det_type+"', "+laneno+", 'false', 'false', 'false', 1, 'sumo', '');");

            }
        }
        
        filein.close();
        
        store.update("insert into iris.meter_algorithm (id, description) VALUES (4, 'max-pressure');");
        store.update("update iris.ramp_meter set algorithm = 4;");
    }
    
    
    private static boolean isRamp(String fromType, String toType){
        
        // 3 types: priority, traffic_light, dead_end
        // priority - priority = mainline. priority-dead_end = mainline
        // priority-trafficlight = ramp. trafficlight-dead end = ramp
        if(toType.equals("priority")){
            if(fromType.equals("traffic_light")){
                return true;
            }
            else{
                return false;
            }
        }
        else if(fromType.equals("priority")){
            if(toType.equals("traffic_light")){
                return true;
            }
            else{
                return false;
            }
        }
        
        if(toType.equals("traffic_light") || fromType.equals("traffic_light")){
            return true;
        }
        else{
            return false;
        }
    }
            
    private static ResultFactory emptyResult(){
        return new ResultFactory(){
            public void create(ResultSet row) throws Exception {}
        };
    }
    
    private static String findVar(String key, String line){
        int idx = line.indexOf(key+"=");
        
        if(idx == -1){
            return null;
        }
        
        
        int start = line.indexOf("\"", idx+1)+1;
        int end = line.indexOf("\"", start);
        
        
        
        return line.substring(start, end);
    }
    
    static private SQLConnection createStore(Properties props)
		throws IOException, TMSException
    {
            return new SQLConnection(
                    props.getProperty("db.url"),
                    props.getProperty("db.user"),
                    props.getProperty("db.password")
            );
    }



    private static class ResultCounterFactory implements ResultFactory{
        public int length = 0;
        public void create(ResultSet row){
            length++;
        }
    }
}
