import org.openkinect.freenect.*;
import org.openkinect.processing.*;  

import spout.*;
Spout spout;

import themidibus.*; 
MidiBus myBus; 

import java.util.Collections;

Kinect k1;
Kinect k2;

float[] depthLookUp = new float[2048];

//angle for 3d space 
float a1 = 0;
float a2 = 0;
float a =0;

float ax = 0;
float ay = 0;

float ax_press = 0;
float ay_press = 0;

//alignment correction
float correction = 2.4;

//other variables
int skip = 3;
int maxDepth = 1000;
int minDepth = 500;
float scale = 2.5;
float factor = 400;
int c = 255;
float thick = 1.5;
int maxDist = 50;
int liningskip= 2;  
int lingcount = 10;
  
boolean simple = false;
boolean lining = false;
boolean pointing = true;
boolean triangling = false;
boolean shuffling = false;

boolean triangles = false;
boolean ellipses = false;
boolean squares = false;

boolean trans = false;
boolean rec = false;
boolean rotating = false;

Rectangle space = new Rectangle(-1500,-1500,-1500,3000,3000,3000); 
octtree ps = new octtree(space,4);

ArrayList<PVector> points = new ArrayList<PVector>();

//-----------------------------------------------------------------------------------------------------------------
//-------------------------------------------main code ------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------

void setup(){
  
  MidiBus.list();
  myBus = new MidiBus(this, 1, 4); 
  
  //frameRate(30);
    
  fullScreen(P3D,2);
  //size (1000 , 1000,P3D);
  
  spout = new Spout(this);
  
  k1 = new Kinect(this);
  k1.activateDevice(0);
  k1.initDepth();
  
  k2 = new Kinect(this);
  k2.activateDevice(1);
  k2.initDepth();

  a1 = k1.getTilt();
  a2 = k2.getTilt();

  // Lookup table for all possible depth values (0 - 2047)
  for (int i = 0; i < depthLookUp.length; i++) {
    depthLookUp[i] = rawDepthToMeters(i);
  }
}

// --------------------------------------------------------------------------------------
// ----------------------------draw function --------------------------------------------
// --------------------------------------------------------------------------------------

void draw(){
  background(0);
  
  if (frameCount % (lingcount) == 0){
    points.clear();
    ps = new octtree(space,4);
    
  }
    
  if (rotating)
    a += 0.0025f;
  
  translate(width/2,height/2,0);
  
  //shift
  //rotateX(0.25);
  //rotateZ(0.25);
  
  rotateY(ay+a);
  rotateX(ax);
  
  strokeWeight(1);
  stroke(206, 39, 161);
  
  pushMatrix();
  translate(-250,0);
  
 // line(0,0,0,500,0,0); //x  
  
  //stroke(255,255,0);
 // line(0,0,0,0,-500,0); //y
  
  //stroke(0,255,255);
 // line(0,0,0,0,0,500); //z
  
  popMatrix();
  
  int[] depth = k1.getRawDepth();
  int[] depth2 = k2.getRawDepth();

  for (int x = 0; x < k1.width; x += skip) {
    for (int y = 0; y < k1.height; y += skip) {
      
      int offset = x + y*k1.width;

      int rawDepth = depth[offset];
      int rawDepth2 = depth2[offset];
      
      if (!trans)
        stroke(c);
      
      strokeWeight(thick);
      noFill();
      
      if ((rawDepth <maxDepth)&&(rawDepth > minDepth)){
       
        PVector v = depthToWorld(x, y, rawDepth);
        PVector p1 = new PVector((-k1.width/2 + x)*scale, (-k1.height/2 + y)*scale, (factor*correction - v.z*factor)*scale);
       
       if (trans)
         stroke(c,floor(map(p1.z,-100,20,20,255)));
       
        pushMatrix();
        translate(p1.x,p1.y,p1.z);
        
        if (pointing)
          point(0,0);
          
        if (triangles)
          triangle(0,0,15,0,7,15);
          
        if (squares)
          rect(0,0,10,10);
        
        if (ellipses)
          ellipse(0,0,10,10);
          
        popMatrix();
        
        if (frameCount%lingcount == 0){
          points.add(p1);
          ps.insert(p1);
        }
      
      }
    
    
     
    }
  }
  
  if (frameCount % 100 == 0){
    println("total points in point cloud: "+points.size());
    println("framerate: "+frameRate);
  }
  
  
  if ((frameCount%lingcount == 0)&&(shuffling))
    Collections.shuffle(points);
  
  if(lining){
     boolean found = false;
     for (int k1 = 1; k1 < points.size(); k1+=liningskip){
      for (int k2 = 0; k2 < points.size(); k2+=liningskip){
        if (k1!=k2){
          PVector p1 = points.get(k1);
          PVector p2 = points.get(k2);
          if (p1.dist(p2) < maxDist){
            line(p1.x,p1.y,p1.z ,p2.x,p2.y,p2.z);
            found = true;
            break;
          }
        }
      }
      found = false;
     }
  }
  
  if (simple)
    for (int k1 = 1; k1 < points.size(); k1+=1){
  
       PVector p1 = points.get(k1);
       PVector p2 = points.get(k1-1);
      
       line(p1.x,p1.y,p1.z ,p2.x,p2.y,p2.z);
    }
    
      
   if(triangling)
     for (int k1 = 0; k1 < points.size(); k1+=liningskip){
         int countlines = 0;
        PVector t = points.get(k1);
        int range = maxDist+5;
        Rectangle r = new Rectangle(t.x-range,t.y-range,t.z-range,range*2,range*2,range*2);
        ArrayList<PVector> f = new ArrayList<PVector>();
        f.clear();
        f.addAll(ps.query(r));
        for(int k2 = 0; k2 < f.size(); k2++)
          if((f.get(k2).dist(t) < maxDist)&&(f.get(k2).dist(t) > maxDist/2)){
            line(f.get(k2).x, f.get(k2).y, f.get(k2).z, t.x, t.y, t.z);
            countlines++;
            if (countlines >=4)
              break;
          }
             
      }

 
   if (rec) 
    saveFrame("frames/####.tif");
  
  spout.sendTexture();
  
}


// ----------------------------------------------------------------------------------
// ---------------------------additional function for control -----------------------
// ----------------------------------------------------------------------------------

// These functions come from: http://graphics.stanford.edu/~mdfisher/Kinect.html
float rawDepthToMeters(int depthValue) {
  if (depthValue < 2047) {
    return (float)(1.0 / ((double)(depthValue) * -0.0030711016 + 3.3309495161));
  }
  return 0.0f;
}

// Only needed to make sense of the ouput depth values from the kinect
PVector depthToWorld(int x, int y, int depthValue) {

  final double fx_d = 1.0 / 5.9421434211923247e+02;
  final double fy_d = 1.0 / 5.9104053696870778e+02;
  final double cx_d = 3.3930780975300314e+02;
  final double cy_d = 2.4273913761751615e+02;

// Drawing the result vector to give each point its three-dimensional space
  PVector result = new PVector();
  double depth =  depthLookUp[depthValue];//rawDepthToMeters(depthValue);
  result.x = (float)((x - cx_d) * depth * fx_d);
  result.y = (float)((y - cy_d) * depth * fy_d);
  result.z = (float)(depth);
  return result;
}


void mousePressed() {
  ax_press = mouseX;
  ay_press = mouseY;
}

void mouseReleased() {
  ax_press = mouseX;
  ay_press = mouseY;
}

void mouseDragged(){
  float correct = 0.005;
   ay = (ax_press - mouseX)*correct;
   ax = (ay_press - mouseY)*correct;
  
}

void keyPressed(){

  if (key == 'q'){
    a1 = a1 + 1;
    k1.setTilt(a1);
  } else if (key == 'w'){
    a1 = a1 - 1;
    k1.setTilt(a1);
  } else   if (key == 'e'){
    a2 = a2 + 1;
    k2.setTilt(a2);
  } else   if (key == 'r'){
    a2 = a2 - 1;
    k2.setTilt(a2);
  }
}


void noteOn(int channel, int pitch, int velocity) {
  println("Note On: Channel: "+channel + "Pitch: "+pitch + "Velocity: "+velocity);
  if (channel == 9){
    switch(pitch){
      case 40:
        println("lining change to: "+ !lining);
        lining = !lining;
        break;
       case 41:
        println("pointing change  to: "+!pointing);
        pointing = !pointing;
         break;
       case 42:
        println("triangling change to: "+!triangling);
        triangling = !triangling;
         break;
       case 43:
        println("shuffling change to: "+!shuffling);
        shuffling = !shuffling;
         break;
       case 48:
        println("simple change to: "+!simple);
          simple = !simple;
         break;
         case 49:
        println("triangles change to: "+!triangles);
        triangles = !triangles;
         break;
       case 50:
        println("squares change to: "+!squares);
        squares = !squares;
         break;
       case 51:
        println("ellipses change to: "+!ellipses);
          ellipses = !ellipses;
         break;
         case 47:
        println("transparent change to: "+!trans);
          trans = !trans;
         break;
          case 46:
          println("rec: " + !rec);
          rec = !rec;
         break;
            case 45:
          println("rotating: " + !rotating);
rotating = !rotating;
         break;
    }
  }
}

void noteOff(int channel, int pitch, int velocity) {
 // println("Note On: Channel: "+channel + " Pitch: "+pitch + " Velocity: "+velocity);
}

void controllerChange(int channel, int number, int value) {
  //println("Controller Change: " + "Channel: "+channel + " Number: "+number + " Value: "+value);
 
  switch(number) {
    case 21: 
      c = int(map(value,0,127,0,255));
      println("color: "+c);  
      break;
    case 22: 
      thick = map(value,0,127,0.01,10);
      println("point thickness: "+ thick);  
      break;
    case 23: 
      skip = int(map(value,0,127,1,100));
      println("kinect skip: "+ skip);  
      break;
    case 24: 
      lingcount = int(map(value,0,127,1,500));
      println("points recaculate "+ lingcount);  
      break;
    case 25: 
      scale = map(value,0,127,0.1,20);
      println("scaling: "+ scale);  
      break;
    case 26: 
      liningskip = int(map(value,0,127,1,100));
      println("lining skip "+ liningskip);  
      break;
    case 27: 
      correction = map(value,0,127,2,3);
      println("correction: "+ correction);  
      break;
    case 28: 
      maxDist = int(map(value,0,127,1,100));
      println("max distance: " + maxDist);  
      break;
  } 

}

void delay(int time) {
  int current = millis();
  while (millis () < current+time) Thread.yield();
}
