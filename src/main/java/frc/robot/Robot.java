/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//this is a test to revert a commit

package frc.robot;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SPI;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class Robot extends TimedRobot {
// create speed controller
  SpeedController BL;
  SpeedController BR;
  SpeedController FL;
  SpeedController FR;
// lifts
  SpeedController frontLift;
  SpeedController backLift;
//create the encoder for the motors and rotation
  Encoder encoderFR;      
  Encoder encoderFL;
  Encoder encoderBR;
  Encoder encoderBL;        
  UltrasonicSensor leftUS;
  UltrasonicSensor rightUS;
  //create the drive, PDP, and Gyro
  MecanumDrive drive;
  PowerDistributionPanel pdp;
  AHRS gyro;
// create variables for vision system and camera 
  CameraServer driveCamServ = CameraServer.getInstance();
  CameraServer visCamServ = CameraServer.getInstance();
  Vision vision;
// create joystick for controllers, buttons, and switches
  LaunchpadWrapper launchpadWrapper;
  ButtonManager buttonManager; 

  Arm arm;
  boolean boomMoving;    //boolean used to tell if boom and gripper is moving
  boolean gripperMoving;

 //tweaking variables
  double rotationTolerance = 0.1;       //for auto rotation, stops plus or minus this angle,                                      
  double distanceTolerance = 10;        //prevents rocking back and forth
  double armJoint1Tolerance = 0.1;
  double armJoint2Tolerance = 0.1;
  // stabilizes straight line movement. larger numbers corrects deviation faster
  double driveXRotation = 0.05;   
  double driveYRotation = 0.05;

  //loop control variables, forces some methods to only run once while a button is pressed
  int boomStopCounter = 0;
  int gripperStopCounter = 0;

  //------------------------------------------------------------------------------------------------------------------------------------------
  @Override
  public void robotInit() {
    BL = new WPI_VictorSPX(1);         // Motors and their CAN ID's (set in the "CRTE Phenonix" program)
    BR = new WPI_VictorSPX(2);
    FL = new WPI_VictorSPX(3);
    FR = new WPI_VictorSPX(4);

    frontLift = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
    backLift =  new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);

    arm = new Arm(this, 0, 1);
    boomMoving = false;
    gripperMoving = false;

    //practice robot speed contollers
    BL= new WPI_TalonSRX(1);
    BR= new WPI_TalonSRX(2);
    FL= new WPI_TalonSRX(4);
    FR= new WPI_TalonSRX(3);

    //create the encoders
    encoderFL = new Encoder(0,1);                                  
    encoderFL.setDistancePerPulse((double) 1/1024 * 18.850);        //set the distance per pulse of encoder, 1024 pulses per rotation of rod
                                                                  //wheel circumfrence = 18.850 in
    encoderFR = new Encoder(2,3);                                  //create the encoder 
    encoderFR.setDistancePerPulse((double) 1/1024 * 18.850);

    encoderBL = new Encoder(4,5);                                  //create the encoder 
    encoderBL.setDistancePerPulse((double) 1/1024 * 18.850);

    encoderBR = new Encoder(6,7);                                  //create the encoder 
    encoderBR.setDistancePerPulse((double) 1/1024 * 18.850);

    // create the ultrasonic sensors
    leftUS = new UltrasonicSensor(3);
    rightUS= new UltrasonicSensor(2);

    //create the drive, PDP, Gyro
    drive = new MecanumDrive(FL, BL, FR, BR); // stating the drive type for the bot
    drive.setSafetyEnabled(false);
    gyro = new AHRS(SPI.Port.kMXP);             // creating Gyro
    gyro.reset();                               //sets the gyro to a heading of 0
    pdp = new PowerDistributionPanel();      // creating Power Distributor Panel

    // creating the controller
    launchpadWrapper = new LaunchpadWrapper(2);
    buttonManager = new ButtonManager(this);
    


    // create variables for vision system and camera
    UsbCamera visionCamera = CameraServer.getInstance().startAutomaticCapture("visionCam",0);          //set camera settings
    visionCamera.setResolution(320, 240);
    visionCamera.setExposureManual(30);
    visionCamera.setFPS(20);
    visionCamera.setBrightness(20);
    vision = new Vision(this);
    
    UsbCamera driveCam = CameraServer.getInstance().startAutomaticCapture("driveCam",1);          //set camera settings
    driveCam.setResolution(320, 240);
    driveCam.setFPS(20);


    
  }

  //------------------------------------------------------------------------------------------------------------------------------------------
  public void disabledInit() {

    //interrupt all multithreaded taskes
    arm.armDriver.interrupt();
    arm.armIdler.interrupt();
    vision.interrupt();

    //set all enable variables to false to stop any other automatic methods
    enableRotate=false;
    enableSpecialRotate=false;
    enableSquareFrame=false;
    enableMoveBotY=false;
    enableMoveBotX=false;
    

  }
  //------------------------------------------------------------------------------------------------------------------------------------------
  public void disabledPeriodic() {


  }
  //------------------------------------------------------------------------------------------------------------------------------------------
  @Override
  public void autonomousInit() {
   
  }

  //------------------------------------------------------------------------------------------------------------------------------------------
  @Override
  public void autonomousPeriodic() {
    
    }
  //------------------------------------------------------------------------------------------------------------------------------------------
  @Override
  public void teleopInit() {
    gyro.reset();             //calibrate the gyro, the current bot angle is now 0 degrees

    arm.armIdler.start("joint1");

    
    boomMoving=false;
    gripperMoving=false;


    boomStopCounter =0;
    gripperStopCounter=0;
  }

  //------------------------------------------------------------------------------------------------------------------------------------------
  @Override
  public void teleopPeriodic() {
    buttonManager.updateButtons();

    drive.driveCartesian(buttonManager.controller.getX(), buttonManager.controller.getY()*-1, buttonManager.controller.getRawAxis(4));


    //manual arm movement
    if(buttonManager.isBu()){           //boom up
      if(!boomMoving){arm.armIdler.setJoint1(false);}         //if boom was not moving before this loop  
      boomMoving = true;
      boomStopCounter=0;    
      arm.joint1Controller.set(0.5);

    }else if(buttonManager.isBd()){     //boom down
      if(!boomMoving){arm.armIdler.setJoint1(false);}         //if boom was not moving before this loop
      boomMoving = true;
      boomStopCounter =0;
      arm.joint1Controller.set(-0.5);

    }else if (boomMoving){              //if no buttons are pressed, and boomMoving is true
      arm.joint1Controller.set(arm.joint1Controller.get()/2);
      if(boomStopCounter>=20){          //if looped 20 times since last button press
        arm.armIdler.setJoint1(true);
        boomMoving=false;
        boomStopCounter=0;
      }else{boomStopCounter++;}
    }

    //manual gripper movement
    if(buttonManager.isGtf()){          //gripper tilt forward
      if(!gripperMoving){arm.armIdler.setJoint2(false);}        //if gripper was not moving before this loop
      gripperMoving = true;
      gripperStopCounter = 0;
      arm.joint2Controller.set(0.5);

    }else if(buttonManager.isGtb()){    //gripper tilt back
      if(!gripperMoving){arm.armIdler.setJoint2(false);}        //if gripper was not moving before this loop
      gripperMoving = true;
      gripperStopCounter=0;
      arm.joint2Controller.set(-0.5);

    }else if(gripperMoving){
      if(gripperStopCounter>=50){
        arm.armIdler.setJoint2(true);;
        gripperMoving = false;
        gripperStopCounter=0;
      }else{gripperStopCounter++;}
    } 

    if(buttonManager.controller.getRawButton(3)){         //red button on controller
        rotateBot(180);                    
      }

    //front and back ascend
    if(buttonManager.isFA()|| buttonManager.isBA()){
      frontLift.set(1);
      backLift.set(1);
    }
    else{
      frontLift.set(0);
      backLift.set(0);
    }

    //front descend 
    if(buttonManager.isFD()){
      frontLift.set(-1);
    }else{
      frontLift.set(0);
    }

    //back descend
    if(buttonManager.isBD()){
      backLift.set(-1);
    }else{
      backLift.set(0);
    }


    //hatch control (switch)
    if(buttonManager.ishtp()){            //hatch pickup
      arm.setSuction(true);
    }else if (buttonManager.ishtr()){     //hatch release
      arm.setSuction(false);
    }

    //ball pickup
    if(buttonManager.isBp()){   
      arm.setBallIntake(1);
    }else{
      arm.setBallIntake(0);
    }

    //ball release
    if(buttonManager.isBr()){
      arm.setBallIntake(-1);
    }else{
      arm.setBallIntake(0);
    }

    //set arm positions
    if(buttonManager.isBlh()){
      arm.moveArm("ball high");
    }else if(buttonManager.isBlm()){
      arm.moveArm("ball medium");
    }else if(buttonManager.isBll()){
      arm.moveArm("ball low");
    }else if(buttonManager.isHth()){
      arm.moveArm("hatch high");
    }else if (buttonManager.isHtm()){
      arm.moveArm("hatch medium");
    }else if (buttonManager.isHtl()){
      arm.moveArm("hatch low");
    }
  }
  //------------------------------------------------------------------------------------------------------------------------------------------
  public void testInit() {
    launchpadWrapper.setLED("red");
    gyro.reset();             //calibrate the gyro, the current bot angle is now 0 degrees

    arm.armIdler.start("joint1");
  }
   
  //------------------------------------------------------------------------------------------------------------------------------------------
  @Override
  public void testPeriodic() {
    buttonManager.updateButtons();

    if (buttonManager.isHtl()){
      System.out.println("arm set low");
      arm.moveArm("hatch low");
    }

    if(buttonManager.isHtm()){
      System.out.println("arm set med");
      arm.moveArm("hatch medium");
    }    

   }
  //------------------------------------------------------------------------------------------------------------------------------------------
  /**
   * For automatic rotations.
   * Calculate the roation speed of the robot based on how far it has left to rotate.
   * @param x Remaining degree measure 0-360 as a double.
   * @return The speed to move from 0-1 as a double. 
   */
  public double calcRotSpeed(double x){           //calculates the rotation speed based on how far until robot reaches end angle
    //double y=(x*0.001)+0.15;                       //a linear function
     double a = (double) -70;
    double b = (double) -344;
    double c = (double) 0;
    
    double y= (double) a/(x+b)+c;        //a rational function
    if(x<10){
      y=0.17;
    }
    return y;
  }

  //------------------------------------------------------------------------------------------------------------------------------------------
  
  boolean enableRotate;

  /**
   * Rotate the robot a certain degree measure relative to the starting position.
   * A positive rotation is clockwise.
   * @param angle The angle measure to rotate as a double.
   */
  public void rotateBot(double angle){                        //rotate the robot a certain angle 
    enableRotate = true;
    double startAngle = gyro.getAngle();                      //get the absolute starting angle of the robot
    System.out.println("start angle: " +startAngle );
    double currentAngle = gyro.getAngle();                    //the current angle of the robot, this will be updated constantly
    double endAngle = startAngle + angle;                     //calculate the absolute end angle 

    while(enableRotate){                                       //loop this until the return keyword is hit or enable rotate is false
      currentAngle = gyro.getAngle();                         //update the robots current angle
      double rotDist = Math.abs(currentAngle-endAngle);         //find the distance yet to rotate
      double rotSpeed = calcRotSpeed(rotDist);                  //calculate the rotation speed
      if(currentAngle<endAngle-rotationTolerance){            //if robot angle is less than end angle(to the left)   
        drive.driveCartesian(0, 0, rotSpeed);                 //rotate clockwise (positive speed)
      }
      else if(currentAngle>endAngle+rotationTolerance){       //if robot angle is greater than end angle (to the right)
        drive.driveCartesian(0, 0, -rotSpeed);                //rotate counter-clockwise (negative rotation)
      }
      else{
        drive.driveCartesian(0, 0, 0);                        //stop the robot once robot is lined up
        System.out.println("end angle: "+ currentAngle);
        return;                                               //exit the loop
      }
    }
  }
  //------------------------------------------------------------------------------------------------------------------------------------------
 
  boolean enableSpecialRotate;

  /**
   * Rotate the robot aound a point on the outer frame a certain degree measure relative to the starting position.
   * A positive rotation is clockwise.
   * @param angle The angle measure to rotate as a double.
   * @param rotationPoint An int that selects which point to rotate around, 1=front left, 2=front middle, 3=front left.
   */
  public void specialRotateBot(double angle,int rotationPoint){                        //rotate the robot a certain angle 
    enableSpecialRotate = true;
    double startAngle = gyro.getAngle();                      //get the absolute starting angle of the robot
    double currentAngle = gyro.getAngle();                    //the current angle of the robot, this will be updated constantly
    double endAngle = startAngle + angle;                     //calculate the absolute end angle 
    double rotDist = Math.abs(currentAngle-endAngle);         //find the distance yet to rotate
    double rotSpeed = calcRotSpeed(rotDist);                  //calculate the rotation speed
    //TODO: program cases for other special rotate cases
    switch (rotationPoint){
      case 1: 
        break;
      case 2:       //front middle
        while(enableSpecialRotate){                               //loop this until the return keyword is hit, or the enable special rotate is false
          currentAngle = gyro.getAngle();                         //update the robots current angle
          rotDist = Math.abs(currentAngle-endAngle);
          rotSpeed = calcRotSpeed(rotDist);
          if(currentAngle<endAngle-rotationTolerance){            //if robot angle is less than end angle(to the left)   
            drive.driveCartesian(-rotSpeed, 0, rotSpeed);                 //rotate clockwise (positive speed)
          }
          else if(currentAngle>endAngle+rotationTolerance){       //if robot angle is greater than end angle (to the right)
            drive.driveCartesian(rotSpeed, 0, -rotSpeed);                //rotate counter-clockwise (negative rotation)
          }
          else{
            drive.driveCartesian(0, 0, 0);                        //stop the robot once robot is lined up
            break;                                               //exit the loop
          }
        }
        break;
    }
  }
  //------------------------------------------------------------------------------------------------------------------------------------------
  
  boolean enableSquareFrame;
  
  /**
   * using the ultrasonic sensors, squares the frame of the robot up with the wall 
   * that it is currently facing. 
   */
  public void squareFrame(){
    enableSquareFrame=true;
    int rotationSteps = 5;                                      //the number of degrees to rotate the bot each time
    int squareTolerance = 1;                                    //tolerance in inches, robot will stop plus or minus this much

    double leftSensorDist = leftUS.getDistance();
    double rightSensorDist = rightUS.getDistance();

    double startingAngle = gyro.getAngle();

    if(leftSensorDist >12*4 || rightSensorDist >12*4 || Math.abs(rightSensorDist-leftSensorDist) >15) {         //if either reads more than 4 feet, or if the difference is more than 15 in, abort 
      fancyErrorReport("squaring failed", false);
      return;
    }

    while(enableSquareFrame){
      leftSensorDist = leftUS.getDistance();
      rightSensorDist = rightUS.getDistance();

      if(Math.abs(gyro.getAngle()-startingAngle) > 45){
        break;
      }

      if(leftSensorDist>rightSensorDist+squareTolerance){
        specialRotateBot(rotationSteps, 2);       //rotate the bot clockwise, around the front middle of the frame
      }else if (leftSensorDist<rightSensorDist-squareTolerance){
        specialRotateBot(-rotationSteps, 2);       //rotate the bot counter-clockwise, around the front middle of the frame
      }else{
        break;
      }
    }

  }
  //------------------------------------------------------------------------------------------------------------------------------------------
 
  boolean enableMoveBotY;
 
  /**
   * Move the robot in the Y axis, forward or backwards. 
   * @param distance The distance to travel, this should always be positive.
   * @param speed    The speed at which the robot will move from -1 to 1. Positive is forward, negative is backwards.
   */
  public void moveBotY(double distance, double speed){
    enableMoveBotY=true;
    gyro.reset();
    double startDist = encoderFR.getDistance();
    double endDist = startDist + distance;
    double currentDist = encoderFR.getDistance();
    speed = Math.abs(speed);

    while(enableMoveBotY){  
      currentDist=encoderFR.getDistance();                     //loop this until the return keyword is hit or enable move bot y is false
      double angle = gyro.getAngle();
      if(currentDist<endDist-distanceTolerance){               //if current distance is less than end distance, it will continue to move
        drive.driveCartesian(0, speed, -angle*driveYRotation);
      }
      else if(currentDist>endDist+distanceTolerance){         //if current distance is greater than end distance, it will back up
        drive.driveCartesian(0, -speed, -angle*driveYRotation);
      }
      else{
        drive.driveCartesian(0, 0, 0);                       // if neither greater nor less than end distance it will stop
        return;
      }
    }
  }
  //------------------------------------------------------------------------------------------------------------------------------------------
  
  boolean enableMoveBotX;
  
  /**
   * Move the robot in the X axis, left or right. 
   * @param distance The distance to travel, this should always be positive.
   * @param speed    The speed at which the robot will move from -1 to 1. Positive is right, negative is left.
   */
  public void moveBotX(double distance, double speed){
    enableMoveBotX=true;
    gyro.reset();
    double startDist = encoderFR.getDistance();
    double endDist = startDist + distance;
    double currentDist = encoderFR.getDistance();
  
    speed = Math.abs(speed);
    while(enableMoveBotX){     
      currentDist=encoderFR.getDistance();
      double angle = gyro.getAngle();                                          //loop this until the return keyword is hit
      if(currentDist<endDist-distanceTolerance){               //if current distance is less than end distance, it will continue to move
        drive.driveCartesian(0, speed, -angle*driveXRotation);
      }
      else if(currentDist>endDist+distanceTolerance){         //if current distance is greater than end distance, it will back up
        drive.driveCartesian(0, -speed,-angle*driveXRotation);
      }
      else{
        drive.driveCartesian(0, 0, 0);                       // if neither greater nor less than end distance it will stop
        return;
      }
    }
  }
  
  /**
   * 
   * @return The launchpad in use.
   */
  public LaunchpadWrapper getLaunchpadWrapper (){
    return launchpadWrapper;
  }
  //------------------------------------------------------------------------------------------------------------------------------------------
  /**
   * @return The current camera server in use.
   */
  public CameraServer getVisionCamServer(){
    return visCamServ;
  }
  //------------------------------------------------------------------------------------------------------------------------------------------
  /**
   * @param message the string that contains the error message
   * @param stackTrace if true, append stack trace to the end of error
   * in addition to a normal error report, flash the driverstation LEDs magenta 
   */
  public void fancyErrorReport (String message, boolean stackTrace){
    DriverStation.reportError(message, stackTrace);
    getLaunchpadWrapper().errorBlink();
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
/* Note:
25.1 for an 8- inch wheel
 18.8 for a 6 in wheel
 12.6 for a 4 inch wheel

 */
