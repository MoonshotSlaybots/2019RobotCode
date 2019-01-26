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
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


public class Robot extends TimedRobot {
  SpeedController BL;
  SpeedController BR;
  SpeedController FR;
  SpeedController FL;
  MecanumDrive drive; 
  Joystick controller = new Joystick(0);
  Solenoid sol1 = new Solenoid(0);
  Solenoid sol2 = new Solenoid(1);
  
  @Override
  public void robotInit() {
   BL = new WPI_VictorSPX(1);
   BR = new WPI_VictorSPX(2);
   FR = new WPI_VictorSPX(3);
   FL = new WPI_VictorSPX(4);
   drive = new MecanumDrive(FR, BL, FR, BR);
   CameraServer.getInstance().startAutomaticCapture("cam0",0);
   drive.setSafetyEnabled(false);
   



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
    if(controller.getRawButton(1)){
      sol1.set(true);

    }
    else{
      sol1.set(false);
    }
    if(controller.getRawButton(2)){
      sol2.set(true);

    }
    else{
      sol2.set(false);
    }
  }
 

  @Override
  public void testPeriodic() {
  }
}
