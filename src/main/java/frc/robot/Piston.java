package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;


public class Piston{
    Solenoid sideA;             
    Solenoid sideB;             
                                
    /**
     * Constructor for the Piston Class, defines sideA and sideB,
     * along with their respective ports.
     * @param portA The port on the PCM that sideA of the valve is connected to.
     * @param portB The port on the PCM that sideB of the valve is connected to.
     */
    Piston(int portA,int portB){             //constructor for the Piston Class
        sideA = new Solenoid(portA);         //portA and portB are the ports that the valve is pluged into for this piston
        sideB = new Solenoid(portB);         //this class was made with a double solenoid valve in mind,
                                             //each valve has two solenoids to control
                                             //activate one side to extend the pistion, then activate the other side to retract
    }
    /**
     * Disables sideB then enables sideA to extend the piston.
     */
    public void extend(){           //extends the piston and holds it 
        sideB.set(false);           //disables side B to let the air into side A 
        sideA.set(true);            //enables side A to let the air in
    }
    /**
     * Disables sideA then enables sideB to retract the piston.
     */
    public void retract(){          //retracts the piston and holds it
        sideA.set(false);           //disables side A to let the air into side B 
        sideB.set(true);            //enables side B to let the air in
    }

}