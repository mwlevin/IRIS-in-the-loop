/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package us.mn.state.dot.tms.server.comm.sumo;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import us.mn.state.dot.sched.TimeSteward;
import us.mn.state.dot.tms.CommLink;
import us.mn.state.dot.tms.DeviceRequest;
import us.mn.state.dot.tms.server.ControllerImpl;
import us.mn.state.dot.tms.server.DetectorImpl;
import us.mn.state.dot.tms.server.RampMeterImpl;
import us.mn.state.dot.tms.server.comm.DevicePoller;
import us.mn.state.dot.tms.server.comm.MeterPoller;
import us.mn.state.dot.tms.server.comm.SamplePoller;
import us.mn.state.dot.tms.server.comm.mndot.MeterStatus;
import static us.mn.state.dot.tms.server.comm.mndot.OpQuerySamples5Min.SAMPLE_PERIOD_SEC;

/**
 * Use a server/socket connection to send the messages to a Python server connecting to Traci.
 * @author michael
 */
public class SumoPoller implements MeterPoller, DevicePoller, SamplePoller {
 
    private static final int serverPort = 5452;
    
    private ServerSocket server;
    private Socket connection;
    private OutputStream output;
    
    private Map<String, CCStorage> detData;
    private int firstPin = -1;
    
    /** Maximum number of scans in 5 minutes */
    static private final int MAX_SCANS = 18000;
    
    public SumoPoller(CommLink link){
        detData = new HashMap<>();
        
        
        System.out.println("created IRIS-SUMO server, port "+serverPort);
        try{
            server = new ServerSocket(serverPort);
        }
        catch(Exception ex){
            ex.printStackTrace(System.err);
            System.exit(1);
        }
        
        Thread t = new Thread(){
            public void run(){
                while(true){
                    try{
                        // I only expect to need 1 connection to a python script interfacing SUMO
                        connection = server.accept();
                        
                        System.out.println("Connected to SUMO client");
                        
                        final BufferedReader input = new BufferedReader(
                            new InputStreamReader(connection.getInputStream(), StandardCharsets.UTF_8));
                        output = connection.getOutputStream();
                        
                        // sendMessage("Hello client!");
                                
                        inner: while(true){
                            // expect 1 message every 30s
                            // messages may arrive out of order, hence the separate threading
                            String message = input.readLine();
                            
                            if(message == null){
                                System.out.println("connection closed");
                                closeConnection();
                                break inner;
                            }
                            
                            
                            System.out.println("Received: "+message);
                            
                            String[] split = message.split(",");
                            
                            if(split[0].equals("det")){
                                String name = split[1];
                                int count = Integer.parseInt(split[2].trim());
                                int occ = (int)Math.round(Double.parseDouble(split[3].trim()) * MAX_SCANS / 300) ; // data is time (sec) out of 30sec
                                if(!detData.containsKey(name)){
                                    System.out.println("could not find detector "+name);
                                    System.out.println("\tall detectors: "+detData.keySet());
                                }
                                else{
                                    //System.out.println("Updating detector "+name);
                                    detData.get(name).update(count, occ);
                                    
                                }
                            }
                            
                            
                            
                        }
                    }
                    catch(Exception ex){
                        closeConnection();
                    }

                }
            }
        };
                
        t.start();
    }
    
    public void sendRequest(RampMeterImpl meter, DeviceRequest r){
        //System.out.println("need to implement send request "+r);
        
        if(r == DeviceRequest.QUERY_STATUS){
            meter.setManualMode(false);
            meter.setPolicePanel(false);
            
            // store green count??
        }
        else if(r == DeviceRequest.SEND_SETTINGS){
            System.out.println("Found meter "+meter.getName());
            
            meter.setLock(null);
            meter.setOperating(true);
        }
    }
    
    public void sendRequest(ControllerImpl c, DeviceRequest r){
        if(r == DeviceRequest.SEND_SETTINGS){
            // call controllerimpl storevehcount
            if(firstPin == -1){

                int first = c.getDetectorPinFirst();
                int last = c.getDetectorPinLast();

                for(int p = first; p <= last; p++){
                    DetectorImpl det = c.getDetectorAtPin(p);
                    
                    System.out.println("Found detector "+det.getName());

                    if(!detData.containsKey(det.getName())){
                        detData.put(det.getName(), new CCStorage());
                    }

                    detData.get(det.getName()).pin = p;
                }

                firstPin = first;
                
                long stamp = TimeSteward.currentTimeMillis();
                int per_sec = 30;
                int[] scans = new int[detData.size()];
                int[] counts = new int[detData.size()];    
                
                c.storeVehCount(stamp, per_sec, firstPin, counts);
                c.storeOccupancy(stamp, per_sec, firstPin, scans, MAX_SCANS * per_sec/300);
            }
        }
    }
    
    public void sendReleaseRate(RampMeterImpl meter, Integer rate){
        
        sendMessage("meter-rate,"+meter.getName()+","+rate);
    }
    
    public void querySamples(ControllerImpl c, int per_sec){
        
        
        long stamp = TimeSteward.currentTimeMillis();
        
        
        
        int[] scans = new int[detData.size()];
        int[] counts = new int[detData.size()];
        
        if(connection != null){
            for(String name : detData.keySet()){
                CCStorage data = detData.get(name);
                int pin = data.pin;
                scans[pin-firstPin] = data.getScan(per_sec);
                counts[pin-firstPin] = data.getCount(per_sec);
            }
        }
        
        
        // c.storeVehCount
        // c.storeOccupancy
        //System.out.println("UPDATE detector");
        c.storeVehCount(stamp, per_sec, firstPin, counts);
        c.storeOccupancy(stamp, per_sec, firstPin, scans, MAX_SCANS * per_sec/300);
        
    }
    
    /** Check if the poller is currently connected */
    public boolean isConnected(){
        return connection != null && connection.isConnected();
    }

    /** Get max seconds an idle connection should be left open
     * (0 indicates indefinite). */
    public int getIdleDisconnectSec(){
        return 0;
    }

    /** Start communication test */
    public void startTesting(ControllerImpl c){
        /* none needed */
    }
    
 

    /** Destroy the poller */
    public void destroy(){
        if(connection != null){
            try{
                connection.close();
            }
            catch(IOException ex){

            }
            finally{
                connection = null;
            }
        }
    }
    
    private void sendMessage(String msg){
        System.out.println("sending message \""+msg+"\"");
        if(output != null){
            try{
                output.write((msg+"\n").getBytes(StandardCharsets.UTF_8));
            }
            catch(Exception ex){
                closeConnection();
            }
        }
    }
    
    private void closeConnection(){
        if(connection != null){
            try{
                connection.close();
            }
            catch(Exception ex){
            }
            finally{
                connection = null;
                
            }
        }
        
        for(String n : detData.keySet()){
            detData.get(n).clear();
        }
    }

    private static class CCStorage{

        // I should change the implementation later, I don't think this is best
        private ArrayDeque<Integer[]> counts;
        public int pin;
        
        public CCStorage() {
            counts = new ArrayDeque<>(10);
            
        }
        
        public void clear(){
            counts.clear();
        }
        

        // not a cumulative count
        // update count from last 30sec
        public synchronized void update(int count, int scan){
            
            // updates are requested at intervals of 30s and 300s
            if(counts.size() >= 300/30){
                counts.pollFirst();
            }
            
            counts.addLast(new Integer[]{count, scan});
            
            /*
            String tostr = "[";
            for(Integer[] o : counts){
                tostr += o[0]+", ";
            }
            tostr+="]";
            System.out.println("\t"+counts.peekLast()[0]+" "+tostr);
            */
        }
        
        public int getCount(int per_sec){
            return getCountHelp(per_sec, 0);
        }
        
        public int getScan(int per_sec){
            return getCountHelp(per_sec, 1);
        }
        
        public int getCountHelp(int per_sec, int idx){
            if(per_sec == 30){
                if(counts.isEmpty()){
                    return 0;
                }
                else{
                    return counts.peekLast()[idx];
                }
            }
            else if(per_sec == 300){
                int total = 0;
                
                for(Integer[] c : counts){
                    total += c[idx];
                }
                
                return total;
            }
            else{
                throw new RuntimeException("unrecognized period");
            }
        }
        
        
        
        
    }

    public String toString(){
        return "SUMO poller";
    }
}
