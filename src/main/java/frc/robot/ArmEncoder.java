package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
 
public class ArmEncoder{
    AnalogInput encoder;
    double angleDelta;
    double startAngle;

    double angle;
    public ArmEncoder(int port){
        encoder = new AnalogInput(port);
        angleDelta =0;
        startAngle=0;
    }

    public double getVoltage(){
        return encoder.getVoltage();
    }

    public void setStartAngle(double angle){
        startAngle = angle;
        angleDelta = Math.abs(this.getRawAngle()-angle);
    }

    public double getRawAngle(){
        return calcAngle();
    }

    private double calcAngle(){
        return (encoder.getVoltage()/5)*360;
    }

    public double getAngle(){
        return this.getRawAngle() - angleDelta;
    }

    public void reset(){                    //change angle delta to reset the encoder back to the start angle 
        setStartAngle(startAngle);
    }
}