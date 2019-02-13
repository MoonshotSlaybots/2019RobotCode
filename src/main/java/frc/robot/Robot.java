/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

public class Robot extends TimedRobot {
  SpeedController BL;
  SpeedController BR;
  SpeedController FR;
  SpeedController FL;

  Encoder encoderFL;        //create the encoder for the front motor 

  MecanumDrive drive;
  AnalogInput lineSensor;
  Joystick controller;
  LaunchpadWrapper launchpad;
  NavXMXP_Gyro gyro;
  PowerDistributionPanel pdp;
  CameraServer camServ = CameraServer.getInstance();


 //tweaking variables
 double rotationTolerance = 10;       //for auto rotation, stops plus or minus this angle, 
                                      //prevents rocking back and forth

  @Override
  public void robotInit() {
  // BL = new WPI_VictorSPX(1);         // Motors and where they are plugged into the bot
  // BR = new WPI_VictorSPX(2);
  // FR = new WPI_VictorSPX(3);
   FL = new WPI_VictorSPX(0);

        //practice robot speed contollers
   BL= new WPI_TalonSRX(1);
   BR= new WPI_TalonSRX(2);
   FR= new WPI_TalonSRX(3);
  // FL= new WPI_TalonSRX(4);

   encoderFL = new Encoder(0,1);                                  //create the encoder 
   encoderFL.setDistancePerPulse((double) 1/1024 * 18.72);        //set the distance per pulse of encoder, 1024 pulses per rotation of rod
                                                                  //wheel circumfrence = 18.72 in

   drive = new MecanumDrive(FL, BL, FR, BR); // stating the drive type for the bot
   drive.setSafetyEnabled(false);
   controller = new Joystick(0);            // creating the controller

   launchpad = new LaunchpadWrapper(1);

   gyro = new NavXMXP_Gyro();              // creating Gyro
   gyro.calibrate();                        //calibrate the gyro

   pdp = new PowerDistributionPanel();      // creating Power Distributor Panel

   lineSensor = new AnalogInput(0);         //create the line sensor on analog port 0


   UsbCamera camera = CameraServer.getInstance().startAutomaticCapture("cam 0",0);          //set camera settings
   
   camera.setResolution(320, 240);
   camera.setExposureManual(10);
   camera.setFPS(20);
   camera.setBrightness(20);
  }

  
  @Override
  public void robotPeriodic() {
  }

  
  @Override
  public void autonomousInit() {
   
  }


  @Override
  public void autonomousPeriodic() {
    
    }
  @Override
  public void teleopInit() {
    gyro.calibrate();             //calibrate the gyro, the current bot angle is now 0 degrees
  }

 
  @Override
  public void teleopPeriodic() {
    //System.out.println("lineSensor voltage: " + lineSensor.getVoltage());
    //System.out.println("Battery voltage is: " + pdp.getVoltage());
    
    drive.driveCartesian(controller.getX()*-1, controller.getY(), controller.getRawAxis(4));
    //possible field oriented drive mode 
    //drive.driveCartesian(controller.getX(), controller.getY(), controller.getZ(), gyro.getAngle());

    System.out.println(gyro.getAngle());

    if(controller.getRawButton(3)){         //red button on controller
        rotateBot(-180);                    
      }



    //vision code
    // if(controller.getRawButton(5)){
    //  Vision visProc = new Vision(camServ);
    //  visProc.start();
    // }
  }


  @Override
  public void testPeriodic() {
    if (launchpad.launchpad.getRawButton(1)){
      launchpad.setLED("magenta");
      launchpad.blinkLED(50,100);
    }
      

   }


  public double calcRotSpeed(double x){           //calculates the rotation speed based on how far until robot reaches end angle
    double y=(x*0.001)+0.2;                       //a linear function
    return y;
  }


  public void rotateBot(double angle){                        //rotate the robot a certain angle 
    double startAngle = gyro.getAngle();                      //get the absolute starting angle of the robor
    double endAngle = startAngle + angle;                     //calculate the absolute end angle 

    double currentAngle = gyro.getAngle();                    //the current angle of the robot, this will be updated constantly

    while(true){                                              //loop this until the return keyword is hit
      currentAngle = gyro.getAngle();                         //update the robots current angle
      double rotDist = Math.abs(currentAngle-endAngle);       //find the distance yet to rotate
      double rotSpeed = calcRotSpeed(rotDist);                //calculate the rotation speed

      System.out.println("rotation speed= "+rotSpeed);        
      if(currentAngle<endAngle+rotationTolerance){            //if robot angle is less than end angle(to the left)   
        drive.driveCartesian(0, 0, rotSpeed);                 //rotate clockwise (positive speed)
      }
      else if(currentAngle>endAngle+rotationTolerance){       //if robot angle is greater than end angle (to the right)
        drive.driveCartesian(0, 0, -rotSpeed);                //rotate counter-clockwise (negative rotation)
      }
      else{
        drive.driveCartesian(0, 0, 0);                        //stop the robot once robot is lined up
        return;                                               //exit the loop
      }
    }
  }

  public LaunchpadWrapper getLaunchpad (){
    return launchpad;
  }

  public CameraServer getCamServer(){
    return camServ;
  }


}
