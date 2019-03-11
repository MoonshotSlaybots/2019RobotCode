package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;

public class UltrasonicSensor {
    int port;
    AnalogInput analogIn;
    
    /**
     * Constructor for an ultrasonic sensor to measure distance in inches.
     * @param port The analog input port that the sensor is connected to.
     */
    public UltrasonicSensor(int port){
        this.port = port;
        analogIn = new AnalogInput(port);
    }
    /**
     * @return The raw voltage of the sensor.
     */
    public double getVoltage(){
        return analogIn.getVoltage();
    }
    /**
     * Calculates and returns the distance based on the voltage recieved from the sensor.
     * Averages voltage over a short period of time to get a smoother reading.
     * @return Distance in inches as a double.
     */
    public double getDistance(){
        /*volts per 5 mm = supplied voltage / 1024
        upplied voltage = 5v
        0.004882813v per 5 mm
        0.000976562v per 1 mm
        25.4 mm per inch
        0.024804687v per 1 in
        */
        return (analogIn.getAverageVoltage() / 0.024804687);
    }
    /** 
     * @return The analog input object
     */
    public AnalogInput getAnalogInput(){
        return analogIn;
    }
}