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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.smartdashboard.*;



import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


public class Robot extends TimedRobot {
  SpeedController BL;
  SpeedController BR;
  SpeedController FR;
  SpeedController FL;
  MecanumDrive drive; 
  Joystick controller;
  Solenoid sol1;
  Solenoid sol2;
  AnalogGyro gyro;
  Compressor comp;
 MecanumDrive m_myRobot;

  @Override
  public void robotInit() {
   BL = new WPI_VictorSPX(1);
   BR = new WPI_VictorSPX(2);
   FR = new WPI_VictorSPX(3);
   FL = new WPI_VictorSPX(4);
   drive = new MecanumDrive(FR, BL, FR, BR);
   CameraServer.getInstance().startAutomaticCapture("cam0",0);
   drive.setSafetyEnabled(false);
   controller = new Joystick(0);
   sol1 = new Solenoid(0);
   sol2 = new Solenoid(1);
   gyro = new AnalogGyro(0);
   comp = new Compressor(0);
  m_myRobot = new MecanumDrive(FL, BL, FR, BR);


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
  public void teleopPeriodic() {
    m_myRobot.MecanumDrive_Cartesian(controller.getX(), controller.getY(), controller.getZ());
    if(controller.getRawButton(1)){
      sol1.set(true);
      }
    else{
      sol1.set(false);
      }
  System.out.println(gyro.getRate());
    if(controller.getRawButton(3)){
      comp.start();
    }
    else{
      comp.stop();
    }
  }


  @Override
  public void testPeriodic() {
  }
}
