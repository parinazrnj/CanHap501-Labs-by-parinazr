/**
 **********************************************************************************************************************
 * @file       sketch_5_Shapes_Physics.pde
 * @author     Steve Ding, Colin Gallacher
 * @version    V5.0.0
 * @date       08-January-2021
 * @brief      Shapes haptic example using 2D physics engine 
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
boolean           renderingForce                      = false;
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

/* World boundaries in centimeters */
FWorld            world;
float             worldWidth                          = 25.0;  
float             worldHeight                         = 16.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;


/* Initialization of interative shapes */
FBox              b;
FPoly             t;
FCircle           g;
FCircle           e;
FBlob             f;
FBox              l1;

/* Initialization of virtual tool */
HVirtualCoupling  s;
PImage            haplyAvatar;



/* Windy region*/
PVector           penWind                             = new PVector(0, 0);
PVector           posPoint                             = new PVector(6, 6);
int iteration=0;
/* end elements definition *********************************************************************************************/ 



/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1000, 640);
  
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
  
  
  ///* creation of square shape */
  //b                   = new FBox(3.0, 3.0);
  //b.setPosition(edgeTopLeftX+worldWidth/4.0, edgeTopLeftY+worldHeight/2.0);
  //b.setDensity(30);
  //b.setFill(random(255), random(255), random(255));
  //world.add(b);
  
  
  /* creation of T shape */
  t                   = new FPoly(); 
  //t.vertex(-1.5, -1.0);
  //t.vertex( 1.5, -1.0);
  //t.vertex( 3.0/2.0, 0);
  //t.vertex( 1.0/2.0, 0);
  //t.vertex( 1.0/2.0, 4.0/2.0);
  //t.vertex(-1.0/2.0, 4.0/2.0);
  //t.vertex(-1.0/2.0, 0);
  //t.vertex(-3.0/2.0, 0);
  t.vertex(23.25,1.75 );
  for(int i=0;i<=14;i++){  
        if (i%2==0){
          t.vertex( 21,i+1.75 );  
    }
    else
    {
       t.vertex( 23,i+1.75);
    }
  }
  t.vertex( 23.25, 15.75);
  t.vertex( 23.25, 16);
  t.setPosition(edgeTopLeftX+1, edgeTopLeftY-1); 
  t.setDensity(20); 
  t.setFill(random(255),random(255),random(255));
  t.setStatic(true);
  world.add(t);
  
  
  
  /* Set viscous layer */
  l1                  = new FBox(24,8);
  l1.setPosition(25.2/2,12.5);
  l1.setFill(150,150,255,80);
  l1.setDensity(59);
  l1.setSensor(true);
  l1.setNoStroke();
  l1.setStatic(true);
  l1.setName("Water");
  world.add(l1);
  
  ///* creation of small circle shape */
  //g                   = new FCircle(1.0);
  //g.setPosition(7, 7);
  //g.setFill(random(255),random(255),random(255));
  //g.setDensity(10);
  //world.add(g);
  
  
  ///* creation of large circle shape */
  //e                   = new FCircle(5);
  //e.setPosition(5, 3);
  //e.setFill(random(255), random(255), random(255));
  //e.setDensity(18); //60g/cm2
  //world.add(e);
  
  
  ///* creation of blob shape, warning may slow down simulation */
  //f                   = new FBlob();
  //float sca           = random(4, 5);
  //sca = sca/2.0;
  //f.setAsCircle(9, 3, sca, 15);
  //f.setStroke(0);
  //f.setStrokeWeight(2);
  //f.setFill(255);
  //f.setFriction(0);
  //f.setDensity(18);
  ////f.setDensity(30);
  //f.setFill(random(255), random(255), random(255));
  //world.add(f);
  
  
  /* Haptic Tool Initialization */
  s                   = new HVirtualCoupling((1)); 
  s.h_avatar.setDensity(1); 
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
 
  
  /* If you are developing on a Mac users must update the path below 
   * from "../img/Haply_avatar.png" to "./img/Haply_avatar.png" 
   */
  haplyAvatar = loadImage("../img/Haply_avatar.png"); 
  haplyAvatar.resize((int)(hAPI_Fisica.worldToScreen(1)), (int)(hAPI_Fisica.worldToScreen(1)));
  s.h_avatar.attachImage(haplyAvatar); 


  /* world conditions setup */
  world.setGravity((0.0), (1000.0)); //1000 cm/(s^2)
  //world.setGravity((0.0), (0.0));
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);
  
  
  world.draw();
  
  
  /* setup framerate speed */
  frameRate(baseFrameRate);
  

  /* setup simulation thread to run at 1kHz */ 
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/



/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if(renderingForce == false){
    background(255);
    world.draw();
  }
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
    
    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
  
    /* Viscous layer codes */
    if (s.h_avatar.isTouchingBody(l1)){
      s.h_avatar.setDamping(700);
    }
    else{
      s.h_avatar.setDamping(10); 
    }
    
      // windy region
      penWind.set((posPoint.x - (posEE.x )), (posPoint.y - (posEE.y )));
   if (penWind.x<0 && penWind.y<0 ){
        s.updateCouplingForce();
    // fEE.set(-penWind.x*0.1, 0);
    //fEE.div(100000); //dynes to newtons
    
    //torques.set(widgetOne.set_device_torques(fEE.array()));
    //widgetOne.device_write_torques();
    
    
    //widgetOne.set_device_torques(new float[]{0.0001*sin(iteration), 0});
    //widgetOne.device_write_torques();
     
    s.h_avatar.setForce(0.0001*((-1)^iteration), 0);
    // fEE.set (0, 0);
   }
  
  
    world.step(1.0f/1000.0f);
  
    renderingForce = false;
    //int iteration;
    iteration++;
  }
}
/* end simulation section **********************************************************************************************/


/* helper functions section, place helper functions here ***************************************************************/
//void exit() {
//  widgetOne.set_device_torques(new float[]{0, 0});
//  widgetOne.device_write_torques();
//  super.exit();
//  }
/* end helper functions section ****************************************************************************************/
