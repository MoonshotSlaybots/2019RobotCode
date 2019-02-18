package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
/**
 * Class to manage the buttons of the controllers. Can update 
 * and get the value of each button defined in this class.
 */
public class ButtonManager{
    Robot robot;
    Joystick grip = robot.grip;
    Joystick wheels = robot.wheels;
    Joystick controller = robot.controller;

    // grip buttons. bl = ball load. ht= hatch. h = high. m = medium. l = low
    boolean blh = grip.getRawButton(0);
    boolean blm = grip.getRawButton(0);
    boolean bll = grip.getRawButton(0);
    boolean hth = grip.getRawButton(0);
    boolean htm = grip.getRawButton(0);
    boolean htl = grip.getRawButton(0);
    // gripper joystick.gtf = gripper tilt forward. gtb = gripper tilt back. bd = boom down. bu = boom up.  
    boolean gtf = grip.getRawButton(0);
    boolean gtb = grip.getRawButton(0);
    boolean bd = grip.getRawButton(0); 
    boolean bu = grip.getRawButton(0); 
    // create Switches. ht = hatch. b = Ball. p = pickup. r = release. 
    boolean htp = grip.getRawButton(0);
    boolean htr = grip.getRawButton(0);
    boolean br = grip.getRawButton(0);
    boolean bp = grip.getRawButton(0);
    // wheel buttons. F = front. B = back. A = acscend. D = descend.
    boolean FA = wheels.getRawButton(0);
    boolean FD = wheels.getRawButton(0);
    boolean BA = wheels.getRawButton(0);
    boolean BD = wheels.getRawButton(0);
    
    public ButtonManager(Robot robot){
        this.robot = robot;
    }
    /**
     * Update all buttons to their current values;
     */
    public void updateButtons(){
     // grip buttons. 
     blh = grip.getRawButton(0);
     blm = grip.getRawButton(0);
     bll = grip.getRawButton(0);
     hth = grip.getRawButton(0);
     htm = grip.getRawButton(0);
     htl = grip.getRawButton(0);
    // gripper joystick.  
     gtf = grip.getRawButton(0);
     gtb = grip.getRawButton(0);
     bd = grip.getRawButton(0); 
     bu = grip.getRawButton(0); 
    // create Switches. 
     htp = grip.getRawButton(0);
     htr = grip.getRawButton(0);
     br = grip.getRawButton(0);
     bp = grip.getRawButton(0);
// wheel buttons.
     FA = wheels.getRawButton(0);
     FD = wheels.getRawButton(0);
    }

    public boolean isBlh() {
        return this.blh;
    }

    public boolean isBlm() {
        return this.blm;
    }

    public boolean isBll() {
        return this.bll;
    }

    public boolean isHth() {
        return this.hth;
    }

    public boolean isHtm() {
        return this.htm;
    }

    public boolean isHtl() {
        return this.htl;
    }

    public boolean isGtf() {
        return this.gtf;
    }

    public boolean isGtb() {
        return this.gtb;
    }

    public boolean isBd() {
        return this.bd;
    }

    public boolean isBu() {
        return this.bu;
    }

    public boolean isHtp() {
        return this.htp;
    }

    public boolean isHtr() {
        return this.htr;
    }

    public boolean isBr() {
        return this.br;
    }

    public boolean isBp() {
        return this.bp;
    }

    public boolean isFA() {
        return this.FA;
    }

    public boolean isFD() {
        return this.FD;
    }

    public boolean isBA() {
        return this.BA;
    }

    public boolean isBD() {
        return this.BD;
    }
}