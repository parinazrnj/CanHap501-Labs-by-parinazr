/**
 **********************************************************************************************************************
 * @file       Maze.pde 
 * @author     Elie Hymowitz, Steve Ding, Colin Gallacher 
 * @version    V4.0.0
 * @date       08-January-2021
 * @brief      Maze game example using 2-D physics engine
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */
 
 
 
/* library imports *****************************************************************************************************/
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
/* end library imports *************************************************************************************************/ 
 
 
 
/* scheduler definition ************************************************************************************************/
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/
 
 
 
/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;
 
byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                     = false;
/* end device block definition *****************************************************************************************/
 
 
 
/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/
 
 
 
/* elements definition *************************************************************************************************/
 
/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;
 
/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);
 
/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                 = new PVector(0, 0); 
 
/* World boundaries */
FWorld            world;
float             worldWidth                          = 25.0;  
float             worldHeight                         = 25.0; 
 
float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;
 
float             gravityAcceleration                 = 980; //cm/s2

float             Depth                          = 0;  //Depth varies depending on the progress of the tool


/* Initialization of virtual tool */
HVirtualCoupling  s;
 
/* define objects used */

final float WIDTH = 3*worldWidth/5;
final float HEIGHT = 3*worldHeight/5;

int rowh = 4;
int colh=8;

float x1= worldWidth/5;
float y1= worldHeight/5;
float lenx=(WIDTH /(rowh+2) );
float leny=(WIDTH / (colh+2));


//FBody[] rectangles = new FBody[rowh*colh];
//FPoly= rectangles ;
FBody[] particles    = new FBody[(rowh+1)*(colh+1)];
FDistanceJoint[] joints=new FDistanceJoint[(rowh+1)*(colh+1)];

FCircle           C1;
FBox              C2;
 
/* text font */
PFont             f;
 
// location and size of the walls
   //the wall in the bottom
 
 
/* end elements definition *********************************************************************************************/ 
 
 
 
/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
   
  /* screen size definition */
  size(1000, 1000);
   
  /* set font type and size */
  f                   = createFont("Arial", 16, true);
 
   
  /* device setup */
   
  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */
  haplyBoard          = new Board(this, Serial.list()[0], 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
   
  widgetOne.set_mechanism(pantograph);
 
  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);
  
  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
   
   
  widgetOne.device_set_parameters();
   
   
  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld(); 
   int k =0;
   for (int j = 0; j <rowh+1 ; j++) {
    for(int i = 0; i < colh; i ++)  { 
             strokeWeight(8);
             float x_pos= x1+i*lenx;
             float y_pos=y1+j*lenx;
             
             //println("i:"+i+ "    j:"+j+"   j*rowh+i"+(j*(colh)+i)+"   rowh:"+rowh);
            particles[j*colh+i] = new FCircle(2.5);
            particles[j*colh+i].setPosition(x_pos, y_pos);
            particles[j*colh+i].setNoStroke();
            particles[j*colh+i].setFriction(10);
            particles[j*colh+i].setHaptic(true);
            particles[j*colh+i].setFill(120, 120, 120);
            
            if (j==rowh){ // last row is static
            particles[j*colh+i].setStatic(true);}
            
            world.add(particles[j*colh+i]);
    
    
            line(x1+i*lenx,y1+j*lenx,x1+(i+1)*lenx,y1+j*lenx) ; 
            stroke(0);   
            
       if (i>=1 && i <colh){
 
            
          FDistanceJoint jt = new FDistanceJoint(particles[j*colh+i],particles[j*colh+i-1]);
          jt.setFrequency(50);
          jt.setDamping(400);
          jt.setFill(255);
          jt.setStroke(3);
          jt.setLength(4);
          jt.setDamping(100);
          //jt.calculateLength();
          world.add(jt);
        }
        
       if (j>=1 && j <rowh+1){
          FDistanceJoint jt = new FDistanceJoint(particles[j*colh+i],particles[(j-1)*colh+i]);
         // jt.setFrequency(f);
          jt.setDamping(400);
          jt.setFill(255);
          jt.setStroke(3);
          //jt.setNoStroke();
          jt.setLength(4);
          //jt.calculateLength();
          world.add(jt);
        }
        
 //       FPoly myPoly = new FPoly();
 //myPoly.vertex(7, 10);

 //myPoly.vertex(7, 7);
 // myPoly.vertex(10, 7);
 //myPoly.vertex(10, 10);
 // myPoly.setPosition(10, 10); 
 // myPoly.setDensity(50); 
 // myPoly.setFill(random(255),random(255),random(255));
 // world.add(myPoly);
            
    } 
  }
  
  //for (int j = 1; j <rowh+1 ; j++) {
  //  for(int i = 1; i < colh; i ++)  { 
      
  //     rectangles[(j*colh-1)+i+1] = new FPoly();
  //      rectangles[(j*colh-1)+i+1].vertex(particles[(j-1)*colh+i-1].getX(), particles[(j-1)*colh+i-1].getY());
  //      rectangles[(j*colh-1)+i+1].vertex(particles[(j-1)*colh+i].getX(), particles[(j-1)*colh+i].getY());
  //      rectangles[(j*colh-1)+i+1].vertex(particles[j*colh+i].getX(), particles[j*colh+i].getY());
  //      rectangles[(j*colh-1)+i+1].vertex(particles[j*colh+i-1].getX(), particles[j*colh+i-1].getY());
  //      world.add(rectangles[(j*colh-1)+i+1]);


       
  //  }
  //}
  

      //i=1;
      //j=1;
      // rectangles= new FPoly);
      //  rectangles.vertex(particles[int((j-1)*colh+i-1)].getX(), particles[(j-1)*colh+i-1].getY());
      //  rectangles.vertex(particles[(j-1)*colh+i].getX(), particles[(j-1)*colh+i].getY());
      //  rectangles.vertex(particles[j*colh+i].getX(), particles[j*colh+i].getY());
      //  rectangles.vertex(particles[j*colh+i-1].getX(), particles[j*colh+i-1].getY());
      //  world.add(rectangles]);


  
  
  //for (int j = 0; j <rowh+1 ; j++) {
  //          FDistanceJoint jt = new FDistanceJoint(particles[1],particles[2]);
  //       // jt.setFrequency(f);
  //        jt.setDamping(400);
  //        jt.setFill(255);
  //        jt.setNoStroke();
  //        jt.setLength(4);
  //        //jt.calculateLength();
  //        world.add(jt);
  //}
  //  for (int i = 0; i <colh+1 ; i++) {
  //  for(int j = 0; j < rowh; j ++)  {
    
  //      strokeWeight(8);
  //      if (i>=1 && i <colh){
  //      //line(x1+i*leny,y1+j*leny,x1+i*leny,y1+(j+1)*leny) ; 
  //          //  FDistanceJoint joints[k] = new FDistanceJoint(particles[i],particles[i+1]);
  //          //joints[k].setFrequency(f);
  //          //joints[k].setDamping(400);
  //          //joints[k].setFill(255);
  //          //joints[k].setNoStroke();
  //          //joints[k].setLength(4);
  //          ////jt.calculateLength();
  //          //world.add(jt);      
  //          //k=k+1;
            
  //        FDistanceJoint jt = new FDistanceJoint(particles[i],particles[i+1]);
  //       // jt.setFrequency(f);
  //        jt.setDamping(400);
  //        jt.setFill(255);
  //        jt.setNoStroke();
  //        jt.setLength(4);
  //        //jt.calculateLength();
  //        world.add(jt);
  //      }
            
  //  } 
  //} 

   
  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((0.75)); 
  s.h_avatar.setDensity(4); 
  s.h_avatar.setFill(255,0,0); 
  s.h_avatar.setSensor(false);
 
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
   
  /* World conditions setup */
  world.setGravity((0.0), gravityAcceleration); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);
  
  
  world.draw();
   
   
  /* setup framerate speed */
  frameRate(baseFrameRate);
   
   
  ///* setup simulation thread to run at 1kHz */
  //if (flag == 1) {
    /* Set viscous layer */
 
    SimulationThread st = new SimulationThread();
    scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
  //}
}
/* end setup section ***************************************************************************************************/
 
 
 
/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
    background(255);
 
 
   
    world.draw();
}
/* end draw section ****************************************************************************************************/
 
 
 
/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{
   
  public void run(){
     
 
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
     
    renderingForce = true;
     
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
     
      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));
      posEE.set(posEE.copy().mult(200));  
    }
     
    s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7); 
    s.updateCouplingForce();
  
  
    fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
    fEE.div(100000); //dynes to newtons
    
     
    //Updating depth location
    
      
      
    //set torques for haply output
    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
    
   
    world.step(1.0f/1000.0f);
   
    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/
 
 
 
/* helper functions section, place helper functions here ***************************************************************/
 
 
/* end helper functions section ****************************************************************************************/



       
      
    
