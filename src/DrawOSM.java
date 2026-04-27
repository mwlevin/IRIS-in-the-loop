

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Scanner;
import javax.imageio.ImageIO;
import javax.swing.JFrame;
import javax.swing.JPanel;

/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */

/**
 *
 * @author michael
 */
public class DrawOSM extends JPanel {
    
    public static void main(String[] args) throws IOException { 
        DrawOSM test = new DrawOSM();
        
        
        String input_file = "sumo-data.txt";
        
        test.readOSM(new File("sumo_display/geometry.txt"));
        test.readMapping(new File("sumo_display/osm-sumo-mapping.txt"));
        test.readData(new File(input_file));
        test.setDisplayRange(0, 60);
        
        
        JFrame frame = new JFrame();
        frame.add(test);
        frame.pack();
        frame.setVisible(true);
        
        frame.addWindowListener(new WindowAdapter(){
            public void windowClosing(WindowEvent e){
                System.exit(0);
            }
        });
        
        BufferedImage image = test.createImage();
        
        String imageType = "jpg";
        
        File outputfile = new File(input_file.substring(0, input_file.indexOf("."))+"."+imageType);

        ImageIO.write(image, imageType, outputfile);
    }
    
    
    public static final Color[] gradient = Gradient.createMultiGradient(new Color[]{Color.red, Color.orange, Color.yellow, Color.green}, 500);
    
    public DrawOSM(){
        int width = 1200;
        int height = 400;
        setSize(width, height);
        setPreferredSize(new Dimension(width, height));
    }
    
    
    double max_value;
    double min_value;
    
    public void setDisplayRange(double min, double max){
        this.min_value = min;
        this.max_value = max;
    }
    
    List<Way> ways;
    
    Map<Integer, String> osm_sumo;
    Map<String, Map<Integer, Double>> data;
    
    public void readData(File file) throws IOException{
        data = new HashMap<>();
        Scanner filein = new Scanner(file);
        
        while(filein.hasNext()){
            String name = filein.next();
            String link = name.substring(0, name.lastIndexOf("_"));
            int lane = Integer.parseInt(name.substring(name.lastIndexOf("_")+1));
            
            if(!data.containsKey(link)){
                data.put(link, new HashMap<>());
            }
            data.get(link).put(lane, filein.nextDouble());
        }
        
        filein.close();
    }
    
    public Color getColor(Way way){

        String sumo_name = osm_sumo.get(way.id).toUpperCase();
        
        if(data.containsKey(sumo_name)){
            double avg = 0;
            Map<Integer, Double> temp = data.get(sumo_name);
            
            for(int lane : temp.keySet()){
                avg += temp.get(lane);
            }
            
            avg /= temp.size();
            
            return getColor(avg);
        }
        else{
            System.out.println("Missing data for "+sumo_name);
            return Color.white;
        }
    }
    
    public Color getColor(double value){
        int idx = Math.min(gradient.length-1, Math.max(0, (int)Math.round( (value - min_value) / (max_value - min_value) * gradient.length)));

        
        return gradient[idx];
    }
    
    public void readMapping(File file) throws IOException{
        osm_sumo = new HashMap<>();
        Scanner filein = new Scanner(file);
        
        while(filein.hasNext()){
            osm_sumo.put(filein.nextInt(), filein.next());
        }
        
        filein.close();
    }
    
    public void readOSM(File file) throws IOException{
        
        ways = new ArrayList<>();
        
        
        Scanner filein = new Scanner(file);
        
        while(filein.hasNextLine()){
            if(filein.nextLine().trim().equals("\"type\": \"way\",")){
                
                
                String temp = filein.nextLine();
                int id = Integer.parseInt(temp.substring(temp.indexOf(":")+1, temp.indexOf(",")).trim());
                
                while(filein.nextLine().indexOf("geometry") < 0);
                
                Way way = new Way(id);
                
                
                while(true){
                    temp = filein.nextLine();
                    
                    if(temp.indexOf("lat") < 0){
                        break;
                    }
                    
                    double lat = Double.parseDouble(temp.substring(temp.indexOf(":")+1, temp.indexOf(",")).trim());
                    temp = temp.substring(temp.indexOf(",")+1);
                    double lon = Double.parseDouble(temp.substring(temp.indexOf(":")+1, temp.indexOf("}")).trim());
                    
                    way.geometry.add(new Node(lat, lon));
                }
                
                ways.add(way);
            }
        }
        
        minLat = Double.MAX_VALUE;
        maxLat = -Double.MAX_VALUE;
        minLon = Double.MAX_VALUE;
        maxLon = -Double.MAX_VALUE;

        for (Way way : ways) {
            for (Node p : way.geometry) {
                double lon = p.lon;
                double lat = p.lat;

                minLat = Math.min(minLat, lat);
                maxLat = Math.max(maxLat, lat);
                minLon = Math.min(minLon, lon);
                maxLon = Math.max(maxLon, lon);
            }
        }
    }
    
    
    protected void paintComponent(Graphics g_){
        Graphics2D g = (Graphics2D)g_;
        
        
        
        int width = getWidth()-10;
        int height = getHeight()-10-200;
        
        
        
        g.setColor(Color.white);
        
        g.fillRect(0, 0, getWidth(), getHeight());
        
        g.setStroke(new BasicStroke(8));
        
        int count = 0;
        
        //System.out.println("--");
        for(Way way : ways){
            
            
                


            Color color = getColor(way);

            if(color != Color.white){
                g.setColor(color);

                for (int i = 1; i < way.geometry.size(); i++) {

                    Node p1 = way.geometry.get(i-1);
                    Node p2 = way.geometry.get(i);

                    int x1 = (int) ((p1.lon - minLon) / (maxLon - minLon) * width)+5;
                    int y1 = (int) ((maxLat - p1.lat) / (maxLat - minLat) * height)+5;

                    int x2 = (int) ((p2.lon - minLon) / (maxLon - minLon) * width)+5;
                    int y2 = (int) ((maxLat - p2.lat) / (maxLat - minLat) * height)+5;


                    g.drawLine(x1, y1, x2, y2);

                    /*
                    if((i == 0 || i == way.geometry.size()-1) && !osm_sumo.containsKey(way.id))
                    {
                        System.out.println(way.id);
                        drawRotatedString(""+way.id, g, x1, y1+30, Math.PI/2);
                        count++;
                    }
                    */
                }

            }
            
        }
        
        
        int y = getHeight()-10-100;
        int colorheight = 20;
        
        width = getWidth()/2;
        
        int x_offset = getWidth()/4;
        
        int x = x_offset;
        
        for(int i = 0; i < gradient.length; i++){
            g.setColor(gradient[i]);
            
            int nextX = x_offset + (int)Math.round((double)(i+1)/gradient.length * width);
            
            g.fillRect(x, y, nextX-x, colorheight);
            x = nextX;
            
            if(i == 0 || i == gradient.length-1 || 
                i % (gradient.length/10) == 0    
                    ){
                double label_val = 0;
                
                if(i == gradient.length-1 ){
                    label_val = max_value;
                }
                else{
                    label_val = ((double)i / gradient.length) * (max_value - min_value) + min_value;
                }
                
                g.setColor(Color.black);
                drawRotatedString(String.format("%.1f", label_val), g, x, y+colorheight+5, Math.PI/2);
            }
        }
        
        
    }
    
    public void drawRotatedString(String text, Graphics2D g2, float textX, float textY, double angle){
        drawRotatedString(text, g2, textX, textY, angle, textX, textY);
    }
    
    public void drawRotatedString(String text,
                                        Graphics2D g2,
                                        float textX,
                                        float textY,
                                        double angle,
                                        float rotateX,
                                        float rotateY) {
       if ((text == null) || (text.equals(""))) {
           return;
       }
       AffineTransform saved = g2.getTransform();
       // apply the rotation...
       AffineTransform rotate = AffineTransform.getRotateInstance(angle, rotateX, rotateY);
       g2.transform(rotate);


           // replaces this code...
        g2.drawString(text, textX, textY);
       
       g2.setTransform(saved);
    }
    
    public BufferedImage createImage() {

        int w = this.getWidth();
        int h = this.getHeight();
        BufferedImage bi = new BufferedImage(w, h, BufferedImage.TYPE_INT_RGB);
        Graphics2D g = bi.createGraphics();
        this.paintComponent(g);
        g.dispose();
        return bi;
    }
    
    double minLat, maxLat, minLon, maxLon;
    
    
    class Node {
        double lat;
        double lon;
        
        public Node(double lat, double lon){
            this.lat = lat;
            this.lon = lon;
        }
        public String toString(){
            return "("+lat+","+lon+")";
        }
    }

    class Way {
        int id;
        List<Node> geometry;
        
        public Way(int id){
            this.id = id;
            geometry = new ArrayList<>();
        }
        
        public String toString(){
            String output = ""+id+"\n";
            for(Node n : geometry){
                output += "\t"+n+"\n";
            }
            return output;
        }
    }
    
    
}
