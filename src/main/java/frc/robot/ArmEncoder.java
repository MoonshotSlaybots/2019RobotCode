package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
 /**
  * read values from rotary encoder.
  */
public class ArmEncoder{
    AnalogInput encoder;
    double startAngle;
    double offset;
    double angle;
    /**
     * Create a new arm encoder to measure angle.
     * @param port the analog input port number that this encoder is plugged into.
     */
    public ArmEncoder(int port){
        encoder = new AnalogInput(port);
        setStartAngle(0);
    }
    /**
     * Calculate the offset of the encoder based on the provided starting angle.
     * @param angle the starting angle of the encoder as a double.
     */
    public void setStartAngle(double angle){
        startAngle = angle;
        offset = startAngle-((360*encoder.getVoltage())/5)+360;
    }
    /**
     * Get the current voltage of the encoder.
     * @return The voltage from 0 to 5 as a double.
     */
    public double getVoltage(){
        return encoder.getVoltage();
    }
    /**
     * Returns the raw angle of the encoder, ignoring the offset.
     * @return
     */
    public double getRawAngle(){
        return calcRawAngle();
    }
    /**
     * Calculates the raw angle of the encoder, ignoring the offset.
     * @return An angle from 0 to 360 as a double.
     */
    private double calcRawAngle(){
        return (encoder.getVoltage()*(360/5));
    }
    /**
     * Get the current angle of the encoder, relative to the set start angle.
     * @return An angle from 0 to 360 as a double.
     */
    public double getAngle(){
        angle= encoder.getVoltage()*(360/5)+offset;
        if(angle>360){
            angle = angle-360;
        }
        return angle;
    }    
    /**
     * Recalculates the offset with the current angle, resetting the encoder.
     */
    public void reset(){
        setStartAngle(startAngle);
    }
}