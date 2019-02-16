package frc.robot;

import java.awt.Rectangle;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.HashMap;


import edu.wpi.first.vision.*;
import edu.wpi.cscore.CvSink;
import edu.wpi.first.cameraserver.CameraServer;

import org.opencv.core.*;
import org.opencv.imgproc.*;
import org.opencv.core.Core.*;
import org.opencv.features2d.FeatureDetector;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.objdetect.*;

public class Vision implements Runnable{
    //vision object vairables        
    private Thread t;
    private CameraServer camServ;
    private Robot robot;
    //private LaunchpadWrapper launchpad;

    //proccessing outputs
    private Mat hsvThresholdOutput = new Mat();   
    private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
    private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>();

    //array lists
    private ArrayList<Tape> tapeList = new ArrayList<Tape>();
    private ArrayList<Target> targetList = new ArrayList<Target>();

    //confirmation vairables
    boolean contourFound=false;
    boolean tapeFound=false;
    boolean targetFound=false;


    private Mat imageA;
    private Mat imageB;

    public Vision(Robot robot){                                  //constructor for a Vision object
        this.robot = robot;                                      //sets the robot variable in the vision object to the robot passed in 
        camServ = this.robot.getCamServer();                     //sets the objects camera server, comes from the getCamServ method in the robot object
    //    launchpad = this.robot.getLaunchpad();                                      
    }

    public void start(){                            //starts the thread and calls the run method
        if(t==null){
            System.out.println("Starting thread");
            t=new Thread(this);
            t.start();
        }
    }

    @Override
    public void run() {                                  //the begining of the new thread
        //launchpad.setLED("yellow");                      //set the color of the driver station led strip
        CvSink camSink = camServ.getVideo("cam 0");      //creates an object to capture images from cam
        imageA = new Mat();                              //create a new matrix that will hold an image
        camSink.grabFrame(imageA);                       //get the next frame from the camera and store it in imageA
        
        process(imageA);                            //process the raw image to find the countours of the tapes        

        if (filterContoursOutput.size()>0){         //if there are contours in the list set the vairable and continue
            contourFound = true;
        }else{                                      //if there are not contours, call the vision failed method and exit the run method
            System.out.println("contour size is 0");
            visionFailed();
            return;
        }

        findTapes();                                //look through the contour list to find tapes and add them to the tapeList

        if(tapeList.size()>0){
            tapeFound=true;
        }else{
            System.out.println("tape size is 0");
            visionFailed();
            return;
        }

        findTargets();                              //look through the tape list and define targets from the tapes

        if(targetList.size()>0){                    
            targetFound=true;
        }else{
            System.out.println("target size is 0");
            visionFailed();
            return;
        }
    }

    private void visionFailed(){                //called when vision proccessing failes, blinks the launchpad LEDs, a return in the run method
                                                //should be called after this method to end the thread
        System.out.println("vision failed");
        t=null;
       // launchpad.setLED("red");        
        //launchpad.blinkLED(50, 10);
        //launchpad.setLED("teamColor");
    }

   
    private void process(Mat source0){               //processes the image and finds contours 
        System.out.println("processing image");

        //step 1: HSV threshold
        Mat hsvThresholdinput = source0;
        double[] h = {12.9,44.006};
        double[] s = {91.46,255.0};
        double[] v = {133.381,255.0};
        hsvThreshold(hsvThresholdinput, h, s, v,hsvThresholdOutput);   
        
        //step 2: Find countours 
        Mat findContoursInput = hsvThresholdOutput;
        findContours(findContoursInput, false, findContoursOutput); 

        //step 3: Filter countours
        ArrayList<MatOfPoint> filterContoursContours = findContoursOutput;
		double filterContoursMinArea = 400.0;
		double filterContoursMinPerimeter = 0;
		double filterContoursMinWidth = 0.0;
		double filterContoursMaxWidth = 1000.0;
		double filterContoursMinHeight = 0;
		double filterContoursMaxHeight = 1000.0;
		double[] filterContoursSolidity = {0, 100};
		double filterContoursMaxVertices = 1000000.0;
		double filterContoursMinVertices = 0;
		double filterContoursMinRatio = 0.51;
		double filterContoursMaxRatio = 1.0;
        filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, 
        filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, 
        filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, 
        filterContoursMaxRatio, filterContoursOutput);
    }


    //Segment an image based on hue, saturation, and value ranges.
	private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val, Mat out) {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
        Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
            new Scalar(hue[1], sat[1], val[1]), out);
    }

    
	//Sets the values of pixels in a binary image to their distance to the nearest black pixel.
	private void findContours(Mat input, boolean externalOnly, List<MatOfPoint> contours) {
        Mat hierarchy = new Mat();
        contours.clear();
        int mode;
        if (externalOnly) {
            mode = Imgproc.RETR_EXTERNAL;
        }
        else {
            mode = Imgproc.RETR_LIST;
        }
        int method = Imgproc.CHAIN_APPROX_SIMPLE;
        Imgproc.findContours(input, contours, hierarchy, mode, method);
    }

    /**
	 * Filters out contours that do not meet certain criteria.
	 * @param inputContours is the input list of contours
	 * @param output is the the output list of contours
	 * @param minArea is the minimum area of a contour that will be kept
	 * @param minPerimeter is the minimum perimeter of a contour that will be kept
	 * @param minWidth minimum width of a contour
	 * @param maxWidth maximum width
	 * @param minHeight minimum height
	 * @param maxHeight maximimum height
	 * @param Solidity the minimum and maximum solidity of a contour
	 * @param minVertexCount minimum vertex Count of the contours
	 * @param maxVertexCount maximum vertex Count
	 * @param minRatio minimum ratio of width to height
	 * @param maxRatio maximum ratio of width to height
	 */

	private void filterContours(List<MatOfPoint> inputContours, double minArea,
        double minPerimeter, double minWidth, double maxWidth, double minHeight, double
        maxHeight, double[] solidity, double maxVertexCount, double minVertexCount, double
        minRatio, double maxRatio, List<MatOfPoint> output) {

        final MatOfInt hull = new MatOfInt();
        output.clear();

        //operation
        for (int i = 0; i < inputContours.size(); i++) {
            final MatOfPoint contour = inputContours.get(i);
            final Rect bb = Imgproc.boundingRect(contour);
            if (bb.width < minWidth || bb.width > maxWidth) continue;
            if (bb.height < minHeight || bb.height > maxHeight) continue;
            final double area = Imgproc.contourArea(contour);
            if (area < minArea) continue;
            if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter) continue;
            Imgproc.convexHull(contour, hull);
            MatOfPoint mopHull = new MatOfPoint();
            mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);

            for (int j = 0; j < hull.size().height; j++) {
                int index = (int)hull.get(j, 0)[0];
                double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
                mopHull.put(j, 0, point);
            }

            final double solid = 100 * area / Imgproc.contourArea(mopHull);
            if (solid < solidity[0] || solid > solidity[1]) continue;
            if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)	continue;
            final double ratio = bb.width / (double)bb.height;
            if (ratio < minRatio || ratio > maxRatio) continue;
            output.add(contour);
        }
    }


    private void findTapes(){
        System.out.println("finding tapes");

        MatOfPoint2f approxCurve = new MatOfPoint2f();

        for(int i=0; i<filterContoursOutput.size(); i++ ){              //iterate through the contours in the filter output
            MatOfPoint2f contour2f = new MatOfPoint2f( filterContoursOutput.get(i).toArray() );

            //Processing on mMOP2f1 which is in type MatOfPoint2f
            double approxDistance = Imgproc.arcLength(contour2f, true)*0.02;
            Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);

            // Get rotated bounding rect of contour
            RotatedRect rect = Imgproc.minAreaRect(approxCurve);

            //TODO:filter new tapes by rotation and size?
            Tape tape = new Tape (rect);

            tapeList.add(tape);
            //TODO: may need to sort tape List by x values 
        }
    }

    private void findTargets(){                     //look through the tape list and find sets of tapes that make up a target
        System.out.println("finding targets");
        
        Tape leftTape=null;
        Tape rightTape=null;

        for(int i=0; i<tapeList.size(); i++){       //iterate through the tape list
            if(tapeList.get(i).rotation>0){         //a positive rotation value is the left tape of a target 
                leftTape = tapeList.get(i);
            }else{                                  //if negative rotation, it is the right tape
                rightTape = tapeList.get(i);
            }

            if(leftTape != null & rightTape != null){               //if both tapes have been defined
                Target target = new Target(leftTape, rightTape);    //create a new target
                targetList.add(target);                             //add that target to the target list

                leftTape = null;                                    //reset the tapes for the next loop
                rightTape = null;
            }
        }
    }
}

class Tape {                                            //class that holds variables about a specific tape

    Rect boundingRect;                                  //a bounding rectangle around the rotated rectangle
    double area;                                        //the area of that bounding rectangle
    double rotation;                                    //the angle of the rotated rectangle

    public Tape(RotatedRect rotRect){                   //constructor: takes in a Rotated rectangle and defines the tape's variables
        boundingRect = rotRect.boundingRect();       
        
        area = boundingRect.area();
        rotation = rotRect.angle;
    }

    public Rect getBoundingRect(){
        return boundingRect;
    }
    public double getArea(){
        return area;
    }
    public double getRotation(){
        return rotation;
    }
}

class Target{                                           //class that holds information about targets
    Tape leftTape;                                      //the left tape of a target
    Tape rightTape;                                     //the right tape
    double distance;                                    //distance from the robot
    double angle;                                       //angle to the robot

    public Target(Tape L, Tape R){                      //constructor: defines the left and right tape
        leftTape = L;
        rightTape = R;
    }
                                                        //getters and setters for the rest of the variables 
    public double getDistance(){
        return distance;
    }
    public double getAngle(){
        return angle;
    }
    public void setDistance(double distance){
        this.distance = distance;
    }
    public void setAngle(double angle){
        this.angle = angle;
    }
}