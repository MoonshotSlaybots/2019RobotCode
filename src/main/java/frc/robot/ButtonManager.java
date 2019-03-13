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
    boolean ht;
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

        // grip buttons. bl = ball load. ht= hatch. h = high. m = medium. l = low
        blh = grip.getRawButton(1);
        blm = grip.getRawButton(2);
        bll = grip.getRawButton(3);
        hth = grip.getRawButton(4);
        htm = grip.getRawButton(5);
        htl = grip.getRawButton(6);

        // gripper joystick. gtf = gripper tilt forward. gtb = gripper tilt back. bd = boom down. bu = boom up.  
        gtf = grip.getRawButton(7);
        gtb = grip.getRawButton(8);
        bd = grip.getRawButton(9); 
        bu = grip.getRawButton(10); 

        // hatch and ball load Switches. ht = hatch. b = Ball. p = pickup. r = release. 
        htp = grip.getRawButton(11);
        htr = grip.getRawButton(12);  //suction position
        br = grip.getRawButton(13);
        bp = grip.getRawButton(14);

        // wheel buttons. F = front. B = back. A = acscend. D = descend.
        FA = wheels.getRawButton(1);
        FD = wheels.getRawButton(2);
        BA = wheels.getRawButton(3);
        BD = wheels.getRawButton(4);
        
    }
    /**
     * Update all buttons to their current values;
     */
    public void updateButtons(){
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
        ht = grip.getRawButton(10);
        br = grip.getRawButton(12);
        bp = grip.getRawButton(13);
        
        // wheel buttons. F = front. B = back. A = acscend. D = descend.
        FA = wheels.getRawButton(4);
        FD = wheels.getRawButton(1);
        BA = wheels.getRawButton(2);
        BD = wheels.getRawButton(3);
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

    public boolean isht() {
        return this.ht;
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