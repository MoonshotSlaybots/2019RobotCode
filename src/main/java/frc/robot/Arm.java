package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SpeedController;

public class Arm {
    Robot robot;
    ArmEncoder joint1Encoder;
    ArmEncoder joint2Encoder;
    SpeedController joint1Controller;
    SpeedController joint2Controller;
    SpeedController ballIntakController;

    Piston suctionPistion;

    ArmDriver armDriver;
    boolean isArmDriverWorking;

    final double TOWER_HEIGHT = 40.0;
    final double L1 = 20.0;
    final double L2 = 10.0;

    /**
     * Constructor for an Arm object 
     * 
     * @param robot             the robot that is createing this arm
     * @param joint1EncoderPort the port number that the first encoder is plugged in to
     * @param joint2EncoderPort the port number that the second encoder is plugged in to
     */
    public Arm(Robot robot, int joint1EncoderPort, int joint2EncoderPort){
        this.robot = robot;
        joint1Controller = new WPI_TalonSRX(5);
        joint2Controller = new WPI_TalonSRX(6);
        ballIntakController = new WPI_TalonSRX(7);
        suctionPistion = new Piston(0, 1);

        joint1Encoder = new ArmEncoder(joint1EncoderPort);
        joint2Encoder = new ArmEncoder(joint2EncoderPort);

        armDriver = new ArmDriver(this);
        isArmDriverWorking = false;
    }
    /**
     * set the speed of the motor cotrolling joint 1
     * @param speed a double from -1 to 1 
     */
    public void setJoint1Controller(double speed){
        joint1Controller.set(speed);
    }

    /**
     * set the speed of the motor cotrolling joint 2
     * @param speed a double from -1 to 1 
     */
    public void setJoint2Controller(double speed){
        joint2Controller.set(speed);
    }
    /**
     * set the speed of the motor cotrolling the ball intake
     * @param speed a double from -1 to 1 
     */
    public void setBallIntake(double speed){
        ballIntakController.set(speed);
    }
    /**
     * set the state of the suction to pick up hatches
     * 
     * @param state a boolean, true to enable suction, false to disable/blow
     */
    public void setSuction(boolean state){
        if(state){
            suctionPistion.extend();
        }else{
            suctionPistion.retract();
        }
    }

    /**
     * moves the arm to set positions.
     * Takes in a string defining the position, the string can be:
     * hatch top, hatch mid, hatch low, ball top, ball mid, ball low.
     * The heights and angles requried are pre-programmed
     * 
     * @param position the string that gives the position the arm should go to 
     */
    public void moveArm(String position){
        if (isArmDriverWorking==false){
            switch (position){
                case "hatch top":
                    armDriver.start(116, 153);
                    break;
                case "hatch mid":
                    break;
                case "hatch low":
                    break;
                case "ball top":
                    break;
                case "ball mid":
                    break;
                case "ball low":
                    break;
                default:
                    DriverStation.reportError("ERROR: Arm position not defined", true);
            }
        }
    }
   

}
/**
 * Controlls automatic movement of the arm.
 */
class ArmDriver implements Runnable{
    Thread t;
    Arm arm;
    double armJoint1Tolerance;
    double armJoint2Tolerance;

    double joint1EndAngle;
    double joint2EndAngle;

    public ArmDriver(Arm arm){
        this.arm = arm;
        armJoint1Tolerance = arm.robot.armJoint1Tolerance;
        armJoint2Tolerance = arm.robot.armJoint2Tolerance;
    }

    /**
     * Begin automatic arm movments, creates a new thread to being a loop
     * @param joint1EndAngle the end angle of the first joint
     * @param joint2EndAngle the end angle of the second joint
     */
    public void start(double joint1EndAngle, double joint2EndAngle){                            //starts the thread and calls the run method
        if(t==null){
            arm.isArmDriverWorking = true;
            System.out.println("Starting thread");
            this.joint1EndAngle = joint1EndAngle;
            this.joint2EndAngle = joint2EndAngle;
            t=new Thread(this);
            t.start();
        }
    }

    public void run() {
        boolean joint1Done = false;         
        boolean joint2Done = false;

        while(true){
            double joint1CurrentAngle = arm.joint1Encoder.getAngle();
            double joint2CurrentAngle = arm.joint2Encoder.getAngle();

            //Joint 1 control
            if(joint1CurrentAngle<joint1EndAngle-armJoint1Tolerance){
                arm.joint1Controller.set(calcSpeed(joint1EndAngle-joint1CurrentAngle));
            }
            else if(joint1CurrentAngle>joint1EndAngle+armJoint1Tolerance){
                arm.joint1Controller.set(calcSpeed(joint1EndAngle-joint1CurrentAngle));
            }
            else{
                arm.joint1Controller.set(0);
                joint1Done = true;
            }

            //Joint 2 control
            if(joint2CurrentAngle<joint2EndAngle-armJoint2Tolerance){
                arm.joint2Controller.set(calcSpeed(joint2EndAngle-joint2CurrentAngle));
            }
            else if(joint2CurrentAngle>joint2EndAngle+armJoint2Tolerance){
                arm.joint2Controller.set(calcSpeed(joint2EndAngle-joint2CurrentAngle));
            }
            else{
                arm.joint2Controller.set(0);
                joint2Done = true;
            }

            if(joint1Done && joint2Done){
                break;
            }
        }
        t=null;
        arm.isArmDriverWorking=false;
    }
    /**
     * calculate the speed to rotate the arm based on the degrees left to rotate
     * @param x the degrees remaining (can be negative or positive)
     * @return the speed to move (range from 0 to 1)
     */
    private double calcSpeed(double x){
        x = Math.abs(x);
        return x*0.001+0.2;
    }

}