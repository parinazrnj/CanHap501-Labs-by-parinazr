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
 
float             gravityAcceleration                 = 0; //cm/s2

float             Depth                          = 0;  //Depth varies depending on the progress of the tool


/* Initialization of virtual tool */
HVirtualCoupling  s;
 
/* define objects used */

FBox              Rwall;
FBox              Bwall;
FBox              Lwall;
FBox              border1;
FBox              border2;
FCircle           C1;
FBox              C2;
 
/* text font */
PFont             f;
 
// location and size of the walls
   //the wall in the bottom
 float Bwall_x=20-8;
 float Bwall_y=20-10;
 float Bwall_w=2;
 float Bwall_h=0.5;
 
 //the wall on the right
 float Rwall_x=19.25-8;     
 float Rwall_y=18.25-10;
 float Rwall_w=0.5;
 float Rwall_h=4;
 
 //the wall on the left
 float Lwall_x=20.75-8;
 float Lwall_y=Rwall_y;
 float Lwall_w=0.5;
 float Lwall_h=4;
 
  //the cutting/drilling tool
 float C2_x= Lwall_x-Rwall_x-Rwall_w-0.1;
 float C2_y=4;
 
 //border wall on the right
 float border1_w=6;
 float border1_h=0.5;
 float border1_x=Rwall_x-border1_w/2+Rwall_w/2;
 float border1_y=Rwall_y-Rwall_h/2+Rwall_w/2;

  //border wall on the left
 float border2_w=6;
 float border2_h=0.5;
 float border2_x=Lwall_x+border1_w/2+Rwall_w/2;
 float border2_y=Rwall_y-Rwall_h/2+Rwall_w/2;
 
 
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
   
//wall in the bottom
 Bwall                   = new FBox(Bwall_w,Bwall_h);
 Bwall.setPosition(Bwall_x,Bwall_y);
 Bwall.setFill(200,0,0);
 Bwall.setDensity(400);
 Bwall.setSensor(false);
 Bwall.setNoStroke();
 Bwall.setStatic(true);
 world.add(Bwall);
  

 //wall on the right
 Rwall                   = new FBox(Rwall_w,Rwall_h);
 Rwall.setPosition(Rwall_x,Rwall_y);
 Rwall.setFill(200,0,0);
 Rwall.setDensity(400);
 Rwall.setSensor(false);
 Rwall.setNoStroke();
 Rwall.setStatic(true);
 world.add(Rwall);

 //wall on the left
 Lwall                   = new FBox(Lwall_w,Lwall_h);
 Lwall.setPosition(Lwall_x,Lwall_y);
 Lwall.setFill(200,0,0);
 Lwall.setDensity(400);
 Lwall.setSensor(false);
 Lwall.setNoStroke();
 Lwall.setStatic(true); //changed true to false
 world.add(Lwall);


 
 border1                   = new FBox(border1_w,border1_h);
 border1.setPosition(border1_x,border1_y);
 border1.setFill(200,0,0);
 border1.setDensity(400);
 border1.setSensor(false);
 border1.setNoStroke();
 border1.setStatic(true); //changed true to false
 world.add(border1);
 
  border2                   = new FBox(border2_w,border2_h);
 border2.setPosition(border2_x,border2_y);
 border2.setFill(200,0,0);
 border2.setDensity(400);
 border2.setSensor(false);
 border2.setNoStroke();
 border2.setStatic(true); //changed true to false
 world.add(border2);
  
 // drilling/cutting tool

 C2                    = new FBox(C2_x,C2_y) ;
 C2.setPosition(Bwall_x,Rwall_y-Bwall_h-0.1);       // (20,17) to (20,10)
 C2.setFill(0,200,0);
 C2.setDamping(100);
 world.add(C2);
  

   
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
    if ( C2.isTouchingBody(Bwall)  ){ // add condition for force
    Depth=C2.getY()+C2.getHeight()/2+0.4;    
    }
   // Force calculations
    if (C2.getY() > 17-7  ){
      C2.addForce(0,-1000*(Depth - s.getToolPositionY()));
    }
    
       if (C2.getY()+C2.getHeight()/2- (Bwall.getY()-Bwall.getHeight()/2)< 0.4  ){
      C2.addForce(0,-1000*(Depth - s.getToolPositionY()));
    }
    
    if (s.getToolPositionY() >  Rwall.getY()-Rwall.getHeight()/2 && s.getToolPositionX() <Lwall.getX()-Lwall_w/2 && s.getToolPositionX() > Rwall.getX()+Rwall_w/2 ){
     // C2.addForce(0,-1000*(C2.getY() - s.getToolPositionY()));
      fEE.set(0.0,0.25*C2.getForceY());
    }// version 1
    

    if ( C2.isTouchingBody(s.h_avatar)==false ){
      C2.setStaticBody(true);
    }
    if (C2.isTouchingBody(s.h_avatar)){
      C2.setStaticBody(false);
    }
     if (C2.isTouchingBody(Bwall) && Depth>0){ // + add condition for force
      Bwall.setStaticBody(true);
      Bwall.setPosition(Bwall_x,Depth);
      Lwall.setPosition(Lwall_x,(Lwall_y)+0.5*(Depth-(Lwall_y+Lwall_h/2)));
      Lwall.setHeight(Lwall_h+(Depth-(Lwall_y+Lwall_h/2)));
      
      Rwall.setPosition(Rwall_x,(Rwall_y)+0.5*(Depth-(Rwall_y+Rwall_h/2)));
      Rwall.setHeight(Rwall_h+(Depth-(Rwall_y+Rwall_h/2)));
    }
     //println("C2 force" + C2.getForceY());
     //println("EE force" + fEE);
     float a= Rwall.getY()-Rwall.getHeight()/2;
     float b= Lwall.getX()-Lwall_w/2 ;
     float c=Rwall.getX()+Rwall_w/2;
     
       println("y:"+a+ "  xmax:"+b +"  xmin:"+c);
            float sa= s.getToolPositionX();
     float sb= s.getToolPositionY() ;
     
     
       println("sx:"+sa+ "  sy:"+sb );
       
     
      
      
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


 ////Updating depth location
 //   if ( C2.isTouchingBody(Bwall) || C2.getY()+C2.getHeight()/2+0.1>=Depth ){ //
 //   Depth=C2.getY()+C2.getHeight()/2+0.4; 
 //    C2.addForce(0,-1000*( Bwall.getY()+Bwall.getHeight()/2)- (C2.getY()+C2.getHeight()/2 ));
      
 //   }

 //   if (s.getToolPositionY() >  Rwall.getY()-Rwall.getHeight()/2 && s.getToolPositionY() <  Bwall.getY() && s.getToolPositionX() <Lwall.getX()-Lwall_w/2 && s.getToolPositionX() > Rwall.getX()+Rwall_w/2 ){
 //    fEE.set(0.0,0.25*C2.getForceY());
 //   }
    
 //   if (C2.isTouchingBody(Bwall) ==false || C2.isTouchingBody(s.h_avatar)==false ){
 //     C2.setStaticBody(true);
 //   }
 //   if (C2.isTouchingBody(s.h_avatar)){
 //     C2.setStaticBody(false);
 //   }
 //    if (C2.isTouchingBody(Bwall) && Depth>0){
 //     Bwall.setStaticBody(true);
 //     Bwall.setPosition(Bwall_x,Depth);
 //     Lwall.setPosition(Lwall_x,(Lwall_y)+0.5*(Depth-(Lwall_y+Lwall_h/2)));
 //     Lwall.setHeight(Lwall_h+(Depth-(Lwall_y+Lwall_h/2)));
      
 //     Rwall.setPosition(Rwall_x,(Rwall_y)+0.5*(Depth-(Rwall_y+Rwall_h/2)));
 //     Rwall.setHeight(Rwall_h+(Depth-(Rwall_y+Rwall_h/2)));
 //   }
 //    //println("C2 force" + C2.getForceY());
 //    //println("EE force" + fEE);
 //    float a= Rwall.getY()-Rwall.getHeight()/2;
 //    float b= Lwall.getX()-Lwall_w/2 ;
 //    float c=Rwall.getX()+Rwall_w/2;
     
 //      println("y:"+a+ "  xmax:"+b +"  xmin:"+c);
 //           float sa= s.getToolPositionX();
 //    float sb= s.getToolPositionY() ;
     
     
 //      println("sx:"+sa+ "  sy:"+sb );
       
      
    
