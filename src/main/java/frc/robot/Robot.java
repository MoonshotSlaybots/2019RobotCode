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
import edu.wpi.cscore.CvSink;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


public class Robot extends TimedRobot {
  SpeedController BL;
  SpeedController BR;
  SpeedController FR;
  SpeedController FL;
  MecanumDrive drive;
  AnalogInput lineSensor;
  Joystick controller;
  AnalogGyro gyro;
 PowerDistributionPanel pdp;
 CameraServer camServ = CameraServer.getInstance();


  @Override
  public void robotInit() {
   BL = new WPI_VictorSPX(1);         // Motors and where they are plugged into the bot
   BR = new WPI_VictorSPX(2);
   FR = new WPI_VictorSPX(3);
   FL = new WPI_VictorSPX(4);
   drive = new MecanumDrive(FR, BL, FR, BR); // stating the drive type for the bot
   drive.setSafetyEnabled(false);
   controller = new Joystick(0);           // creating the controller

   SPI.Port gyroPort = SPI.Port.kOnboardCS0;
   gyro = new AnalogGyro(0);               // creating Gyro

   gyro.initGyro();
   gyro.calibrate();

   pdp = new PowerDistributionPanel();     // creating Power Distributor Panel

   //lineSensor = new AnalogInput(0);


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
    System.out.println("Gyro angle: " + gyro.getAngle());

    drive.driveCartesian(controller.getX(), controller.getY(), controller.getZ());
    //possible field oriented drive mode 
    //drive.driveCartesian(controller.getX(), controller.getY(), controller.getZ(), gyro.getAngle());

    if(controller.getRawButton(1)){
      }
    else{
      }



    //vision code
    // if(controller.getRawButton(5)){
    //  Vision visProc = new Vision(camServ);
    //  visProc.start();
    // }
  }


  @Override
  public void testPeriodic() {
  }
}
