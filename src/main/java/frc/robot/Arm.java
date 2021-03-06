package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SpeedController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class Arm {
    Robot robot;
    ArmEncoder joint1Encoder;
    ArmEncoder joint2Encoder;
    SpeedController joint1Controller;
    SpeedController joint2Controller;

    Piston suctionPiston;

    ArmDriver armDriver;
    ArmIdler armIdler;

    final double TOWER_HEIGHT = 40.0;           //height of the tower from the ground
    final double L1 = 20.0;                     //length of the first arm segment
    final double L2 = 10.0;                     //length of the second arm segment

    /**
     * Constructor for an Arm object
     * 
     * @param robot             the robot that is createing this arm
     * @param joint1EncoderPort the port number that the first encoder is plugged in to
     * @param joint2EncoderPort the port number that the second encoder is plugged in to
     */
    public Arm(Robot robot, int joint1EncoderPort, int joint2EncoderPort) {
        this.robot = robot;
        joint1Controller = new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless);
        joint2Controller = new WPI_VictorSPX(8);

        suctionPiston = new Piston(0, 1);

        joint1Encoder = new ArmEncoder(joint1EncoderPort);
        joint2Encoder = new ArmEncoder(joint2EncoderPort);
        //TODO remeasure arm starting angle
        joint1Encoder.setStartAngle(30);
        joint2Encoder.setStartAngle(350);

        armDriver = new ArmDriver(this);
        armIdler = new ArmIdler(this);
        armIdler.startArm("void");                 //begins the arm idler, but sets its state to none
    }

    /**
     * set the speed of the motor cotrolling joint 1.
     * 
     * @param speed a double from -1 to 1
     */
    public void setJoint1Controller(double speed) {

        joint1Controller.set(speed);
    }

    /**
     * set the speed of the motor cotrolling joint 2.
     * 
     * @param speed a double from -1 to 1
     */
    public void setJoint2Controller(double speed) {
        joint2Controller.set(speed);
    }

    /**
     * set the state of the suction to pick up hatches
     * 
     * @param state a boolean, true to enable suction, false to disable/blow
     */
    public void setSuction(boolean state) {
        if (state) {
            suctionPiston.extend();
        } else {
            suctionPiston.retract();
        }
    }
    /**
     * sets the arm in a idle state so it will maintain the current position of both joints 
     */
    public void setArmIdle(){
        //should be "both" not joint1
        armIdler.startArm("joint1");
    }

    public void resetIdler(){
        armIdler.interrupt();
        joint1Controller.set(0);
        joint2Controller.set(0);
    }

    /**
     * moves the arm to set positions. Takes in a string defining the position, the
     * string can be: hatch high, hatch medium, hatch low, ball high, ball medium,
     * ball low. The heights and angles requried are pre-programmed
     * 
     * @param position the string that gives the position the arm should go to
     */
    public void moveArm(String position) {
        armIdler.setBothJoints(false);
        switch (position) {
        case "hatch high":
            armDriver.start(141, 130);
            break;
        case "hatch medium":
            armDriver.start(96, 173);
            break;
        case "hatch low":
            armDriver.start(54, 217);
            break;
        case "ball high":
            armDriver.start(170, 100);
            break;
        case "ball medium":
            armDriver.start(108, 162);
            break;
        case "ball low":
            armDriver.start(68, 202);
            break;
        default:
            robot.fancyErrorReport("ERROR: Arm position not defined", true);
        }
    }

}
// ------------------------------------------------------------------------------------------------------------------------------------------

/**
 * Controlls automatic movement of the arm.
 */
class ArmDriver implements Runnable {
    Thread t;
    Arm arm;
    double armJoint1Tolerance;
    double armJoint2Tolerance;

    double joint1EndAngle;
    double joint2EndAngle;

    public ArmDriver(Arm arm) {
        this.arm = arm;
        armJoint1Tolerance = arm.robot.armJoint1Tolerance;
        armJoint2Tolerance = arm.robot.armJoint2Tolerance;
    }

    /**
     * Begin automatic arm movments, creates a new thread to being a loop
     * 
     * @param joint1EndAngle the end angle of the first joint
     * @param joint2EndAngle the end angle of the second joint
     */
    public void start(double joint1EndAngle, double joint2EndAngle){                            //starts the thread and calls the run method
        if(t==null){
            t = new Thread(this);
            System.out.println("Starting driver thread");
            this.joint1EndAngle = joint1EndAngle;
            this.joint2EndAngle = joint2EndAngle;
            t.start();
        }
    }

    public void interrupt() {
        if (t != null) {
            t.interrupt();                  // set the interruption flag that is tested for in the run method.
            try {
                t.join();                   // wait for the thread "t" to finish execution
            } catch (Exception e) {
            }
            t = null;                       // set t to null for future threads.
        }
    }
    

    public void run() {
        arm.robot.launchpadWrapper.setLED("yellow");

        boolean joint1Done = false;
        boolean joint2Done = false;

        int vt1 = 0; // counting variale for velocity calculations for joint 1 and joint2
        int vt2 = 0;

        int maxVT1 =0;
        double startingDelta1=Math.abs(joint1EndAngle-arm.joint1Encoder.getAngle());
        double startingDelta2=Math.abs(joint2EndAngle-arm.joint2Encoder.getAngle());
        double currentDelta1;


        while(true){

           if (Thread.interrupted()) {                     // call armDriver.interrupt; to set the interrupt flag and enter this if statment
                System.out.println("arm driver interrupted");
                arm.armIdler.setBothJoints(false);
                return;
            }
            

            double joint1CurrentAngle = arm.joint1Encoder.getAngle();
            double joint2CurrentAngle = arm.joint2Encoder.getAngle();

            //Joint 1 control
            if(joint1CurrentAngle<=joint1EndAngle-armJoint1Tolerance){
                arm.joint1Controller.set(calcSpeed(vt1,true));
            }
            else if(joint1CurrentAngle>joint1EndAngle+armJoint1Tolerance){
                arm.joint1Controller.set(calcSpeed(vt1,false));
            }
            else{
                joint1Done = true;
                if(joint2Done){
                    arm.joint1Controller.stopMotor();
                    arm.armIdler.setBothJoints(true);;
                }else{
                    arm.joint1Controller.stopMotor();
                    arm.armIdler.setJoint1(true);
            }
            }
            currentDelta1 = Math.abs(joint1EndAngle - joint1CurrentAngle);

            if(currentDelta1 >=(startingDelta1 / 2)){                            //if motor has reached half way point, move back down the curve
                vt1++;
            } else {
                vt1--;
            }
            if(vt1>maxVT1){
                maxVT1=vt1;
            }
            
            //System.out.println("vt1 = " + vt1);
            //System.out.println("currentdelta1 = " + currentDelta1);
            //System.out.println("startingdelta1 = " + startingDelta1);
            //System.out.println("joint1Done = "+joint1Done);
            
            //Joint 2 control
            if(joint2CurrentAngle<=joint2EndAngle-armJoint2Tolerance){
                arm.joint2Controller.set(calcSpeed(vt2,true));
            }
            else if(joint2CurrentAngle>joint2EndAngle+armJoint2Tolerance){
                arm.joint2Controller.set(calcSpeed(vt2,false));
            }
            else{
                joint2Done = true;
                if(joint1Done){
                    arm.joint1Controller.stopMotor();
                    arm.armIdler.setBothJoints(true);
                }else{
                    arm.joint1Controller.stopMotor();
                    arm.armIdler.setJoint2(true);
                }
            }

            double currentDelta2 = Math.abs(joint2EndAngle - joint2CurrentAngle);
            if (currentDelta2 >= startingDelta2 / 2) {
                vt2++;
            } else {
                vt2--;
            }
            

            if(joint1Done && joint2Done){
                vt1=0;
                vt2=0;
                break;
            }
            
        }
        System.out.println("maxVT1 = "+maxVT1);
        t = null;
        arm.robot.launchpadWrapper.setLED("teamColor");
    }

    /**
     * Calculate the speed to rotate based on a counting variable vt. Increment vt
     * to get new speed values. This allows for custom speed curves. Once halfway
     * has been reached, decrement vt.
     * 
     * @param vt       The counting variable, equals the x value of the given curve.
     * @param positive A boolean, true if the speed value should be positive.
     * @return the speed to move (range from 0 to 1)
     */
    private double calcSpeed(int vt, boolean positive) {
        // parameters to tweak velociy curve: delay is c, steepness is k, max speed is a
        // desmos graph: https://www.desmos.com/calculator/kppwi6pzt1

        double c = 3;
        double k = 0.007;
        double a = 0.5;

        if (positive) {
            return (((Math.atan(k * vt - c) * 2 / Math.PI) + 1) * (a / 2));
        } else {
            return (-((Math.atan(k * vt - c) * 2 / Math.PI) + 1) * (a / 2));
        }
    }

}

// ------------------------------------------------------------------------------------------------------------------------------------------
/**
 * Holds the arm on the current position by polling the rotery encoders and
 * moving the motors to correct any drift.
 */
class ArmIdler implements Runnable {
    Thread t;
    Arm arm;
    ArmEncoder joint1Encoder;
    ArmEncoder joint2Encoder;

    double joint1Start;
    double joint2Start;

    String selection;

    boolean joint1;
    boolean joint2;

    double joint1CorrectionSpeed;
    double joint2CorrectionSpeed;

    /**
     * Holds the arm on the current position by polling the rotery encoders and
     * moving the motors to correct any drift.
     */
    public ArmIdler(Arm arm) {
        this.arm = arm;
        joint1Encoder = arm.joint1Encoder;
        joint2Encoder = arm.joint2Encoder;

        joint1CorrectionSpeed = 0.02; // correction speed of the motors, the higher the speed, the more likely
                                      // oscilation will be.
        joint2CorrectionSpeed = 0.02;

        // selection
        joint1 = false;
        joint2 = false;
    }

    /**
     * call this method to begin the arm idler thread
     * 
     * @param selection A string that tells the idler what to control.
     *                  "joint1", "joint2", "both", or "none"
     */
    public void startArm(String selection) { // starts the thread and calls the run method
        if (t == null) {
            switch (selection) {
            case "joint1":
                joint1 = true;
                break;
            case "joint2":
                joint2 = true;
                break;
            case "both":
                joint1 = true;
                joint2 = true;
                break;
            case "void":
                joint1 = false;
                joint2 = false;
                break;
            default:
            
            DriverStation.reportError("arm idle failed, could not determain selection.", true);

            }
            System.out.println("Starting thread");
            t = new Thread(this);
            t.start();
        } else { // if called when idler is already running, interrupt it and create the new
                 // idler
            interrupt(); // calls the interrupt method to begin stopping the current thread
            startArm(selection); // starts a new thread
        }
    }

    /**
     * call this method to interrupt the arm idler thread
     */
    public void interrupt() {
        if (t != null) {
            t.interrupt();                  // set the interruption flag that is tested for in the run method.
            try {
                t.join();                   // wait for the thread "t" to finish execution
            } catch (Exception e) {
            }
            t = null;                       // set t to null for future threads.
        }
    }

    @Override
    public void run() {
        joint1Start = joint1Encoder.getAngle();
        joint2Start = joint2Encoder.getAngle();

        while (true) {

            if (Thread.interrupted()) { // call armIdler.interrupt; to set the interrupt flag and enter this if statment
                System.out.println("arm idle interrupted");
                return;
            }
            if (joint1 && joint2) {
                checkJoint1();
                checkJoint2();
            } else if (joint1) {
                checkJoint1();
            } else if (joint2) {
                checkJoint2();
            }
        }
    }
    /**
     * Enable or disable the idler for joint1.
     * @param state A boolean to set the state, true is enable, false is disable.
     */
    public void setJoint1(boolean state) {
        if (state) {                // enable
            joint1Start = joint1Encoder.getAngle();
            joint1 = true;
        } else {                    // disable
            joint1 = false;

            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            arm.joint1Controller.stopMotor();
        }
    }
    /**
     * Enable or disable the idler for joint2.
     * @param state A boolean to set the state, true is enable, false is disable. 
     */
    public void setJoint2 (boolean state){
        if (state) {                // enable
            joint2Start = joint2Encoder.getAngle();
            joint2 = true;
        } else {                    // disable
            joint2 = false;

            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            arm.joint2Controller.stopMotor();
        }
    }

    public void setBothJoints (boolean state){
       setJoint1(state);
       setJoint2(state);
    }

    private void checkJoint1(){
        if(joint1){
           arm.joint1Controller.set(-(joint1Encoder.getAngle()-joint1Start)*joint1CorrectionSpeed);
        }
    }

    private void checkJoint2(){
        if(joint2){
            arm.joint2Controller.set(-(joint2Encoder.getAngle()-joint2Start)*joint2CorrectionSpeed);
        }
    }
}