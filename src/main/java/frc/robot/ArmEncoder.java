package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
 
public class ArmEncoder{
    AnalogInput encoder;
    double startAngle;
    double offset;
    double angle;

    public ArmEncoder(int port){
        encoder = new AnalogInput(port);
        startAngle=0;
        calcOffset();
    }

    private void calcOffset(){
        offset = startAngle-((360*encoder.getVoltage())/5)+360;
    }

    public void setStartAngle(double angle){
        startAngle = angle;
        calcOffset();
    }

    public double getVoltage(){
        return encoder.getVoltage();
    }

    public double getRawAngle(){
        return calcRawAngle();
    }

    private double calcRawAngle(){
        return (encoder.getVoltage()*(360/5));
    }

    public double getAngle(){
        angle= encoder.getVoltage()*(360/5)+offset;
        if(angle>360){
            angle = angle-360;
        }
        return angle;
    }

    

    public void reset(){                    //set the current angle to start angle, recalculates the offset
        calcOffset();
    }
}