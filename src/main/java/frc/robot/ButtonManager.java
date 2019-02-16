package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class ButtonManager{
    Robot robot;
    Joystick grip = robot.grip;
    Joystick wheels = robot.wheels;
    Joystick controller = robot.controller;

    // grip buttons. bl = ball load. bh = bulkhead. h = high. m = medium. l = low
    boolean blh = grip.getRawButton(0);
    boolean blm = grip.getRawButton(0);
    boolean bll = grip.getRawButton(0);
    boolean bhh = grip.getRawButton(0);
    boolean bhm = grip.getRawButton(0);
    boolean bhl = grip.getRawButton(0);
    // wheel buttons. A = acscend. D = descend. F = front. B = back.
    boolean FA = wheels.getRawButton(0);
    boolean FD = wheels.getRawButton(0);
    boolean BA = wheels.getRawButton(0);
    boolean BD = wheels.getRawButton(0);
    
    public ButtonManager(Robot robot){
        this.robot = robot;
    }
}