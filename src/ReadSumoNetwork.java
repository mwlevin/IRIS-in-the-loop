
import java.io.File;
import java.io.IOException;
import java.io.PrintStream;
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
    
    public static void main(String network, String netfile) throws Exception {
        readSumo("sumo code/"+network+"/"+netfile+".net.xml", "sumo code/"+network+"/detectors.xml", "sumo code/"+network+"/meters.txt", "sumo code/"+network+"/detectors.txt");
    }
    
    static class Edge{
        public String name, type;
        public Node from, to;
        public double length;
        public int numlanes;
        
        public Edge(String name, Node from, Node to, double length, int numlanes, String type){
            this.name = name;
            this.from = from;
            this.to = to;
            this.length = length;
            this.numlanes = numlanes;
            this.type = type;
        }
        
        public String toString(){
            return name;
        }
    }
    
    static class Node{
        public String name, type;
        public double x, y;
        
        public Node(String name){
            this(name, -1, -1, null);
        }
        
        public Node(String name, double x, double y, String type){
            this.name = name;
            this.x = x;
            this.y = y;
            this.type = type;
        }
        
        public String toString(){
            return name;
        }
    }
    
    static class Lane{
        public String name;
        public Node from, to;
        public double length;
        public String type;
        public int lane_no;
        
        public Lane(String name, Node from, Node to, double length, String type, int lane_no){
            this.name = name;
            this.from = from;
            this.to = to;
            this.length = length;
            this.type = type;
            this.lane_no = lane_no;
        }
        
        public String toString(){
            return name;
        }
    }
    
    public static void readSumo(String netfile, String detectorfile, String meters_out, String detectors_out) throws Exception{
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
        
        Map<String, String> meters = new HashMap<>();
        List<String> detectors = new ArrayList<>();
        
        
        int drop_id = 1;
        
        boolean created_controller = false;
        
        String controllername = "sumo_ctl";
        
        drop_id++;
        
        int cont_pin = 0;
        
        Map<String, Node> junctions = new HashMap<>();
        Map<String, Lane> lanes = new HashMap<>();
        Map<String, Edge> edges = new HashMap<>();
        
        
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
                
                if(junctions.containsKey(name)){
                    junctions.get(name).type = type;
                }
                else{
                    junctions.put(name, new Node(name, -1, -1, type));
                }
            }
            else if(line.indexOf("<edge ") >= 0){
                String function = findVar("function", line);
                
                if(function != null && function.equals("internal")){
                    continue;
                }
                String name = findVar("id", line);
                String from = findVar("from", line);
                
                // ignore the internal function junctions
                if(from == null){
                    continue;
                }
                String to = findVar("to", line);
                
                if(!junctions.containsKey(from)){
                    junctions.put(from, new Node(from));
                }
                
                if(!junctions.containsKey(to)){
                    junctions.put(to, new Node(to));
                }
    

                double length = Double.parseDouble(findVar("length", line));
                
                edges.put(name, new Edge(name, junctions.get(from), junctions.get(to), length, 1, null));
            }
            else if(line.indexOf("<lane ") >= 0){
                String name = findVar("id", line);
                double length = Double.parseDouble(findVar("length", line));

                lanes.put(name, new Lane(name, null, null, length, "none", 0));
            }
        }
        
        filein.close();
        
        
        
        
        
        // I need to redo the positions for the junctions based on distances
        // therefore: mainline has x=0 and ramps are 90 degrees
        // arbitrarily pick one junction to be (0,0).
        // one of the priority junctions should be (0,0)
        
        Stack<Node> unsettled = new Stack<>();
        
        for(String name : junctions.keySet()){
            Node n = junctions.get(name);
            
            if(n.type.equals("priority")){
                
                n.x = 0.0;
                n.y = 0.0;
                unsettled.push(n);
                break;
            }
        }
        
        while(!unsettled.isEmpty()){
            Node junc = unsettled.pop();
            
            
            
            for(String name : edges.keySet()){
                Node from = edges.get(name).from;
                Node to = edges.get(name).to;
                
                String typeFrom = from.type;
                String typeTo = to.type;

                String type = isRamp(typeFrom, typeTo);
                
                edges.get(name).type = type;


                if(from.equals(junc)){
                    Node data2 = to;
                    
                    if(data2.x != -1){
                        continue;
                    }
                    
                    if(data2.type.equals("dead_end")){
                        data2.type = "exit";
                    }
                    
                    if(!type.equals("mainline")){
                        to.y = from.y;
                        //junctions.get(to)[1] = (double)junctions.get(from)[1] + (double)edges.get(name)[3];
                        to.x = 0.0;
                    }
                    else{
                        to.x = 0.0;
                        to.y = (double)from.y - (double)edges.get(name).length;
                    }
                    
                    unsettled.push(to);
                }
                else if(to.equals(junc)){
                    Node data2 = from;
                    
                    if(data2.x != -1){
                        continue;
                    }
                    
                    if(data2.type.equals("dead_end")){
                        data2.type = "entrance";
                    }
                    
                    if(!type.equals("mainline")){
                        from.y = to.y;
                        //junctions.get(from)[1] = (double)junctions.get(to)[1] - (double)edges.get(name)[3];
                        from.x = 0.0;
                    }
                    else{
                        from.x = 0.0;
                        from.y = to.y + (double)edges.get(name).length; 
                    }
                    
                    unsettled.push(from);
                }
            }
        }
        
        
        
        for(String edge : edges.keySet()){
            int maxlaneid = 0;
            
            while(lanes.containsKey(edge+"_"+maxlaneid)){
                String lanename = edge+"_"+maxlaneid;
                lanes.get(lanename).length = edges.get(edge).length;
                lanes.get(lanename).from = edges.get(edge).from;
                lanes.get(lanename).to = edges.get(edge).to;
                lanes.get(lanename).lane_no = maxlaneid+1;
                
                maxlaneid++;
            }
            
            edges.get(edge).numlanes = maxlaneid;
        }
        
        Map<String, String> createdNodes = new HashMap<>();
        
        
        String saved_loc = "";
        
        
        int nodekey = 0;
        
        int speedlimit = 60;
        
        for(String name : junctions.keySet()){

            
            double x = junctions.get(name).x;
            double y = junctions.get(name).y;
            String type = junctions.get(name).type;
            
            String locname = "loc-"+name+"-0";
            String locname2 = "loc-"+name+"-1";
            
            saved_loc = locname;
            
            
            
            
            String beaconname = "beacon-"+name;
            
            String stationname = "st-"+name;

            String cross = "none";
            
            for(String edge : edges.keySet()){
                
                
                if(edges.get(edge).from.name.equals(name) && edges.get(edge).type.equals("on-ramp")){
                    //System.out.println(edge);
                    cross = "x-"+edge;
                    
                }
                else if(edges.get(edge).to.name.equals(name) && edges.get(edge).type.equals("on-ramp"))
                {
                    cross = "x-"+edge;
                    
                    
                }
            }
            
            if(cross.length() > 20){
                cross = cross.substring(0, 20);
            }
            
            if(!cross.equals("none")){
                ResultCounterFactory counter = new ResultCounterFactory();
                store.query("SELECT * FROM iris.road WHERE name='"+cross+"';", counter);

                if(counter.length == 0){
                    store.update("INSERT INTO iris.road VALUES ('"+cross+"', '"+cross.substring(0, 6)+"', 6, 2);");
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
                if(edges.get(edge).from.name.equals(name) || edges.get(edge).to.name.equals(name)){
                    numlanes = (int)Math.max(numlanes, (int)edges.get(edge).numlanes);
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
                
                String on_ramp = "";
                
                for(String lname : lanes.keySet()){
                    Lane l = lanes.get(lname);
                    
                    
                    
                    if(l.to == junctions.get(name) && l.lane_no == 1){
                        on_ramp = l.name;
                        break;
                    }
                }
                
                meters.put(name, on_ramp);
            }
                
		// create r node
            
        }
        
        
        
        // then read detectors file
        
        
        
        
        filein = new Scanner(new File(detectorfile));
        
        
        
        
        while(filein.hasNextLine()){
            String line = filein.nextLine();
            
            
            
            if(line.indexOf("<inductionLoop ") >= 0){
                String name = findVar("id", line);
                
                System.out.println("Adding detector "+name);
                double period = Double.parseDouble(findVar("period", line));
                
                if(period != 30.0){
                    continue;
                }
                
                detectors.add(name);
                
                name=name.replaceAll("det", "d");
                name = name.replaceAll("Demand", "D"); // name is too long!
                name = name.replaceAll("Down", "Do"); // name is too long!
                name = name.replaceAll("Pass", "P"); // name is too long!
                name = name.replaceAll("Green", "G"); // name is too long!
                name = name.replaceAll("Merge", "M"); // name is too long!
                name = name.replaceAll("Exit", "Ex"); // name is too long!
                
                String lane = findVar("lane", line);
                String edge = lane.substring(0, lane.indexOf("_"));
                double pos = Double.parseDouble(findVar("pos", line));
                
                String nodeid = edge+"_"+pos;
                
                
                String loc = name;
                
                if(createdNodes.containsKey(nodeid)){
                    loc = createdNodes.get(nodeid);
                }
                
                System.out.println(lane);
                Node from = lanes.get(lane).from;
                boolean ramp = lanes.get(lane).type.equals("on-ramp");
                
                int numlanes = (int)edges.get(edge).numlanes;
                
                char det_type = '0';
                
                String stationname = "st-"+loc;
                String nodename = "n-"+loc;
                String locname = "l-"+loc;
                int node_type = 0;
              
                
                double x = from.x;
                double y = from.y;
                
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
                    
                    nodename = "n-"+edges.get(edge).to;
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
                    
                    nodename = "n-"+edges.get(edge).from;
                    
                }
                
                if(!createdNodes.containsKey(nodeid) && !createdNodes.containsKey(nodename)){
                    
                    
                    store.update("INSERT INTO iris.geo_loc VALUES ('"+locname+"', '', 'mainline0', '0', '"+cross+"', '2', '0', '', "+x+", "+y+");");
                    nodekey++;
                    

                    
                    store.update("INSERT INTO iris.r_node VALUES ('"+nodename+"', '"+locname+"', "+nodekey+", "+node_type+", 'true', 'false', 0, "+numlanes+", 'true', "+0+", 'true', '"+stationname+"', "+speedlimit+", '');");
                    
                    createdNodes.put(nodeid, loc);
                }
                
                
                int laneno = (int)lanes.get(lane).lane_no;
                

                //String controllername = "det_ctl_"+name;
                String detname = name;

                


                store.update("INSERT INTO iris.detector VALUES ('"+detname+"', '"+controllername+"', "+(pin++)+", '"+nodename+"', '"+det_type+"', "+laneno+", 'false', 'false', 'false', 1, 'sumo', '');");

            }
        }
        
        filein.close();
        

        //store.update("update iris.ramp_meter set algorithm = 4;");
        
        
        
        PrintStream fileout = new PrintStream(new File(detectors_out));
        
        for(String d : detectors){
            fileout.println(d);
        }
        fileout.close();
        
        
        fileout = new PrintStream(new File(meters_out));
        
        for(String m : meters.keySet()){
            fileout.println(m+"\t"+meters.get(m));
        }
        fileout.close();
    }
    
    
    private static String isRamp(String fromType, String toType){
        
        // 3 types: priority, traffic_light, dead_end
        // priority - priority = mainline. priority-dead_end = mainline
        // priority-trafficlight = ramp. trafficlight-dead end = ramp
        if(toType.equals("priority")){
            if(fromType.equals("traffic_light")){
                return "merge";
            }
            else{
                return "mainline";
            }
        }
        else if(fromType.equals("priority")){
            if(toType.equals("traffic_light")){
                return "on-ramp";
            }
            else{
                return "mainline";
            }
        }
        
        return "mainline";
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
