package frc.robot;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
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

    //proccessing outputs
    private Mat hsvThresholdOutput = new Mat();   
    private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
    private ArrayList<MatOfPoint> tapeList = new ArrayList<MatOfPoint>();
    private ArrayList<MatOfPoint> targetList = new ArrayList<MatOfPoint>();

    //object vairables        
    private Thread t;
    private CameraServer camServ;
    private Robot robot;
    private LaunchpadWrapper launchpad;

    private Mat imageA;
    private Mat imageB;

    boolean contourFound=false;
    boolean targetFound=false;


    public Vision(Robot robot){                                  //constructor for a Vision object
        this.robot = robot;                                      //sets the robot variable in the vision object to the robot passed in 
        camServ = this.robot.getCamServer();                     //sets the objects camera server, comes from the getCamServ method in the robot object
        launchpad = this.robot.getLaunchpad();                                      
    }
    public void start(){                            //starts the thread and calls the run method
        System.out.println("Starting thread");
        if(t==null){
            t=new Thread(this);
            t.start();
        }

    }

    @Override
    public void run() {                                  //the begining of the new thread
        launchpad.setLED("yellow");                      //set the color of the driver station led strip
        CvSink camSink = camServ.getVideo("cam 0");      //creates an object to capture images from cam
        imageA = new Mat();                              //create a new matrix that will hold an image
        camSink.grabFrame(imageA);                       //get the next frame from the camera and store it in imageA
        
        process(imageA);

        if (tapeList.size()>0){
            contourFound = true;
        }else{                                 //if countourFound = false, change driver station LEDs
            launchpad.setLED("red");
            launchpad.blinkLED(50, 10);
            launchpad.setLED("teamColor");
            return;
        }


    }

   
    public void process(Mat source0){               //processes the image and finds contours 
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
        filterContoursMaxRatio, tapeList);
    }

    public void findTargets(){
        for(int i=0; i<tapeList.size(); i++ ){              //iterate through the contours in the tapelist
            MatOfPoint tape = tapeList.get(i);
            Imgproc proc = new Imgproc();

        }

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
}