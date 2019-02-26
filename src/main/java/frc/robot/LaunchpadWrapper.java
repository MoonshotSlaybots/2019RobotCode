package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

/**
 * A class to "wrap" the launchpad (a custom HID device) to more easily control an LED strip connected to it
 */
public class LaunchpadWrapper {
    Joystick launchpad;
                                        //default pins on the launch pad that each LED is connected to
    int redPin = 2;
    int greenPin = 1;
    int bluePin = 5;
    int whitePin = 3;
                                        //the states that the leds are in at any given time
    boolean redState=false;
    boolean greenState=false;
    boolean blueState=false;
    boolean whiteState=false;

    String teamColor=null;

    /**
     * Creates a joystick to reference the launchpad and sets the team color automatically.
     * @param port The USB port that the launchpad is connected to.
     */
    public LaunchpadWrapper(final int port){           //constructor for the launchpad, creates a joystick on USB port provided 
        launchpad = new Joystick(port);
        setTeamColor();
    }
    /**
     * Sets the team color from the driver station alliance.
     */
    private void setTeamColor(){
       DriverStation.Alliance color = DriverStation.getInstance().getAlliance();
       if(color==DriverStation.Alliance.Blue){
           teamColor="blue";
       }else{
           teamColor="red";
       }
    }
    /**
     * @return The team color as a string.
     */
    public String getTeamColor(){
        return teamColor;
    }
    /**
     * Get the alliance from the driver station and update the team color to match.
     */
    public void updateTeamColor(){
        setTeamColor();
    }
    /**
     * Set which pins that the LEDs are connected to on the launchpad.
     * @param red   The pin number that controlls the red LEDs.
     * @param green The pin number that controlls the green LEDs.
     * @param blue  The pin number that controlls the blue LEDs.
     * @param white The pin number that controlls the white LEDs.
     */
    public void setLEDPins(int red, int green, int blue, int white){
        redPin=red;
        greenPin=green;
        bluePin=blue;
        whitePin=white;
    }
    /**
     * Sets the LEDs through a string. Available colors:
     * red, yellow, green, cyan, blue, magenta, white, teamColor, and off.
     * @param color A string that tells the color, ex: "blue"
     */
    public void setLED(String color){
        launchpad.setOutput(redPin, false);                 //clears the LEDs and sets their states to false       
        launchpad.setOutput(greenPin, false);
        launchpad.setOutput(bluePin, false);
        launchpad.setOutput(whitePin, false);
        redState=false;
        greenState=false;
        blueState=false;
        whiteState=false;

        switch (color){
            case "red":                                     //if color = "red", set the pin to true and change the state 
                launchpad.setOutput(redPin, true);
                redState=true;
                break;
            case "yellow":
                launchpad.setOutput(redPin, true);
                launchpad.setOutput(greenPin, true);
                redState=true;
                greenState=true;
                break;
            case "green":
                launchpad.setOutput(greenPin, true);
                greenState=true;
                break;
            case "cyan":
                launchpad.setOutput(greenPin, true);
                launchpad.setOutput(bluePin, true);
                greenState=true;
                blueState=true;
                break;
            case "blue":
                launchpad.setOutput(bluePin, true);
                blueState=true;
                break;
            case "magenta":
                launchpad.setOutput(redPin, true);
                launchpad.setOutput(bluePin, true);
                redState=true;
                blueState=true;
                break;
            case "white":
                launchpad.setOutput(whitePin, true);
                whiteState=true;
                break;
            case "teamColor":
                this.setLED(teamColor);
                break;
            case "off":
                //colors already cleared at beginning
                break;
            default:                                                        //if the input is none defined, throw an error
                DriverStation.reportError("Launchpad: LED color not defined", true);       
                break;
        }
        
    }
    /**
     * Set the LEDs by turning each color on or off.
     * @param red   A boolean that controls the red LEDs.
     * @param green A boolean that controls the green LEDs.
     * @param blue  A boolean that controls the blue LEDs.
     * @param white A boolean that controls the white LEDs.
     */
    public void setLED(boolean red, boolean green, boolean blue, boolean white){        //set the LED color through RBG values 
        launchpad.setOutput(redPin, red);
        launchpad.setOutput(greenPin, green);
        launchpad.setOutput(bluePin, blue);
        launchpad.setOutput(whitePin, white);
        redState=red;
        greenState=green;
        blueState=blue;
        whiteState=white;
    }
    /**
     * Blink the current color of the LED strip. WARNING: Do not call this method in quick succession,
     * multiple threads will be made, causing the robot to CRASH.
     * @param delay  The delay between turning the led on or off in miliseconds.
     * @param cycles The number of times the light cycles, once turns the lights off then on.
     */                                                                
    public void blinkLED(int delay, int cycles){                            //blink the current color
        BlinkLED blinkLED = new BlinkLED(this, delay, cycles);              //create a new object to run this on a different thread
        Thread thread = new Thread(blinkLED);                               //create the new thread
        thread.start();                                                     //start the thread (will call the run() method)
    }
                                                                    //this is in a new thread so it does not tie down the system
                                                                    //by using the thread.sleep() method which stops the code for a certain time
}

class BlinkLED implements Runnable{             //a class to handle blinking the LEDs
    LaunchpadWrapper launchpadWrapper;
    int delay;                                  //the delay between state changes
    int cycles;                                 //the number of cycles 
    /**
     * Constructor to blink the LEDs.
     * @param launchpad The launchpad to control.
     * @param delay     The delay between turning the led on or off in miliseconds.
     * @param cycles    The number of times the light cycles, once turns the lights off then on.
     */
    public BlinkLED(LaunchpadWrapper launchpad,int delay,int cycles){         //constructor for this object
        this.launchpadWrapper = launchpad;
        this.delay = delay;
        this.cycles = cycles;
    }

    /**
     * starts the blinking process.
     * This is ran when the .start() method is called on the thread.
     */
    public void run(){                         
        try {
            blink();                            //the blink method, with a try catch becuase the thread.sleep can be interrupted
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    /**
     * The method that blinks the LEDs with a <code> for </code> loop.
     * @throws InterruptedException
     */
    private void blink() throws InterruptedException{       
        boolean red = launchpadWrapper.redState;            //gets the current state of the launchpad LEDs
        boolean green = launchpadWrapper.greenState;
        boolean blue = launchpadWrapper.blueState;
        boolean white = launchpadWrapper.whiteState;

        for(int i=0;i<cycles;i++){
            launchpadWrapper.setLED("off");                     //turns LEDs off

            Thread.sleep(delay);                                //waits 

            launchpadWrapper.setLED(red,green,blue,white);      //turn LEDs on to what they used to be
    
            Thread.sleep(delay);                                //wait then loop "cycles" number of times
        }
    }


    
}