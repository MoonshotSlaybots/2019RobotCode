package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

//a class to "wrap" the launchpad (a custom HID device) to more easily control a LED strip connected to it

public class LaunchpadWrapper {
    Joystick launchpad;
                                        //default pins on the launch pad that each LED is connected to
    int redPin = 3;
    int greenPin = 6;
    int bluePin = 5;
    int whitePin = 4;
                                        //the states that the leds are in at any given time
    boolean redState=false;
    boolean greenState=false;
    boolean blueState=false;
    boolean whiteState=false;


    public LaunchpadWrapper(final int port){           //constructor for the launchpad, creates a joystick on USB port provided 
        launchpad = new Joystick(port);
    }

    public void setLEDPins(int red, int green, int blue, int white){            //set the pins that the LEDs are connected to on the launchpad
        redPin=red;
        greenPin=green;
        bluePin=blue;
        whitePin=white;
    }


    //sets the LEDs through a color available colors:
    //red, yellow, green, cyan, blue, magenta, white, and off

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
            case "off":
                //colors already cleared at beginning
                break;
            default:                                                        //if the input is none defined, throw an error
                DriverStation.reportError("Launchpad: LED color not defined", true);       
                break;
        }
        
    }
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

                                                                    //WARNING: Do not call this method in quick succession,
                                                                    //multiple threads will be made, causing the robot to CRASH
                                                                    
    public void blinkLED(){                                         //blink the current color
        BlinkLED blinkLED = new BlinkLED(this, 100, 50);            //create a new object to run this on a different thread
        Thread thread = new Thread(blinkLED);                       //create the new thread
        thread.start();                                             //start the thread (will call the run() method)
    }
                                                                    //this is in a new thread so it does not tie down the system
                                                                    //by using the thread.sleep() method which stops the code for a certain time
}

class BlinkLED implements Runnable{             //a class to handle blinking the LEDs
    LaunchpadWrapper launchpadWrapper;
    int delay;                                  //the delay between state changes
    int cycles;                                   //the number of cycles 

    public BlinkLED(LaunchpadWrapper launchpad,int delay,int cycles){         //constructor for this object
        this.launchpadWrapper = launchpad;
        this.delay = delay;
        this.cycles = cycles;
    }

    @Override
    public void run(){                          //this is ran with the .start() method is called on the thread
        try {
            blink();                            //the blink method, with a try catch becuase the thread.sleep can be interrupted
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    private void blink() throws InterruptedException{       //the method that blinks the LEDs
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