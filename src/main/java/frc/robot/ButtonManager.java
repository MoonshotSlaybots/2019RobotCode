package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
/**
 * Class to manage the buttons of the controllers. Can update 
 * and get the value of each button defined in this class.
 */
public class ButtonManager{
    
    Robot robot;
    Joystick grip;
    Joystick wheels;
    Joystick controller;
    Joystick controller2;

    boolean blh;
    boolean blm;
    boolean bll;
    boolean hth;
    boolean htm;
    boolean htl;
    boolean gtf;
    boolean gtb;
    boolean bd;
    boolean bu;
    boolean htp;
    boolean htr;
    boolean br;
    boolean bp;
    boolean FA;
    boolean FD;
    boolean BA;
    boolean BD;
    
    
    public ButtonManager(Robot robot){
        this.robot = robot;
        controller = new Joystick(0); 
        wheels = new Joystick(1);
        grip = robot.getLaunchpadWrapper().launchpad;
        controller2 = new Joystick(3);

        // grip buttons. bl = ball load. ht= hatch. h = high. m = medium. l = low
        blh = grip.getRawButton(14);
        blm = grip.getRawButton(1);
        bll = grip.getRawButton(2);
        hth = grip.getRawButton(3);
        htm = grip.getRawButton(4);
        htl = grip.getRawButton(5);
        // gripper joystick. gtf = gripper tilt forward. gtb = gripper tilt back. bd = boom down. bu = boom up.  
        gtf = grip.getRawButton(6);
        gtb = grip.getRawButton(7);
        bd = grip.getRawButton(8); 
        bu = grip.getRawButton(9); 
        // hatch and ball load Switches. ht = hatch. b = Ball. p = pickup. r = release. 
        htp = grip.getRawButton(10);
        htr = grip.getRawButton(11);
        br = grip.getRawButton(12);
        bp = grip.getRawButton(13);
        // wheel buttons. F = front. B = back. A = acscend. D = descend.
        FA = wheels.getRawButton(4);
        FD = wheels.getRawButton(1);
        BA = wheels.getRawButton(2);
        BD = wheels.getRawButton(3);
        
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
    // create switches. 
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