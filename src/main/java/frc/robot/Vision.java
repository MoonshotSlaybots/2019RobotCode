package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.cscore.CvSink;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;

import org.opencv.core.*;
import org.opencv.imgproc.*;

/**
 * Vision class, uses camera and a ring light to see retro reflective targets.
 */
public class Vision implements Runnable{
    //vision object vairables        
    private Thread t;
    private boolean isVisionWorking=false;
    private CameraServer camServ;
    private Robot robot;
    private LaunchpadWrapper launchpad;

    //proccessing outputs
    private Mat hsvThresholdOutput = new Mat();   
    private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
    private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>();
    
    public Target selectedTarget;

    //array lists
    private ArrayList<Tape> tapeList = new ArrayList<Tape>();
    private ArrayList<Target> targetList = new ArrayList<Target>();

    //confirmation vairables
    boolean contourFound=false;
    boolean tapeFound=false;
    boolean targetFound=false;


    private Mat imageA;
    double cameraOffset = 5;            //distance from camera to front of robot
    /**
     * Constructor for a Vision object, sets the camera server and launchpad for vision.
     * @param robot The robot that created this vision object.
     */
    public Vision(Robot robot){                                  //constructor for a Vision object
        this.robot = robot;                                      //sets the robot variable in the vision object to the robot passed in 
        camServ = this.robot.getVisionCamServer();                     //sets the objects camera server, comes from the getCamServ method in the robot object
        launchpad = this.robot.getLaunchpadWrapper();                                      
    }
    /**
     * Looks to see if another vision thread is running, if not begin a new one and run the "run" method.
     */
    public void start(){                            //starts the thread and calls the run method
        System.out.println("call to start vision, isVisionWorking=" + isVisionWorking);
        if(!isVisionWorking){
                System.out.println("Starting thread");
                t=new Thread(this);
                t.start();
        }
    }

    /**
     * The main vision method. The vision thread starts from here.
     */
    public void run() {                                         //the begining of the new thread
        isVisionWorking = true;
        launchpad.setLED("yellow");                             //set the color of the driver station led strip
        CvSink camSink = camServ.getVideo("visionCam");         //creates an object to capture images from cam
        imageA = new Mat();                                     //create a new matrix that will hold an image
        camSink.grabFrame(imageA);                              //get the next frame from the camera and store it in imageA
        System.out.println("imageA = " + imageA.elemSize());
        process(imageA);                                        //process the raw image to find the countours of the tapes        

        if (filterContoursOutput.size()>0){                     //if there are contours in the list set the vairable and continue
            contourFound = true;
        }else{                                                  //if there are not contours, call the vision failed method and exit the run method
            visionFailed();
            return;
        }

        findTapes();                                            //look through the contour list to find tapes and add them to the tapeList

        if(tapeList.size()>0){
            tapeFound=true;
        }else{
            visionFailed();
            return;
        }

        findTargets();                              //look through the tape list and define targets from the tapes

        if(targetList.size()>0){                    
            targetFound=true;
        }else{
            visionFailed();
            return;
        }

        processTargets();

        if (targetList.size()>1){
            selectTarget();
        }else{
            selectedTarget = targetList.get(0);
        }

        launchpad.setLED("green");          //blink green to show that a target has been found
        launchpad.blinkLED(100, 5);
        launchpad.setLED("yellow");         //solid yellow to show robot is moving



        robot.specialRotateBot(selectedTarget.angle, 2);            //rotate robot so target will be in the center of camera vision

        robot.moveBotY(selectedTarget.distance*0.75, 0.8);          //drive the bot 75% of the way to the target at 80% speed

        robot.squareFrame();                                        //square the frame with the wall


        visionSuccess();
    }
    /**
     * Called when vision processing fails. reports and error and blinks launchpad LEDs.
     * A return in the run method should be called after this method to end the thread.
     */
    private void visionFailed(){                
        robot.fancyErrorReport("ERROR: Vision failed", false);

        endVision();
    }

    private void visionSuccess(){
       
        isVisionWorking = false;
        launchpad.setLED("green");        
        launchpad.blinkLED(50, 10);
        launchpad.setLED("teamColor");
        endVision();
    }

    private void endVision(){
        tapeList.clear();
        targetList.clear();
        
        isVisionWorking = false;
    }
   /**
    * Process the image and find countours
    * @param source0 The source image as a matrix.
    */
    private void process(Mat source0){               
        //step 1: HSV threshold
        Mat hsvThresholdinput = source0;
        double[] h = {46.94244604316547, 135.68760611205434};
        double[] s = {84.84712230215827, 255.0};
        double[] v = {103.19244604316546, 255.0};
        hsvThreshold(hsvThresholdinput, h, s, v,hsvThresholdOutput);   
        System.out.println("hsv out: "+ hsvThresholdOutput.elemSize());
        
        //step 2: Find countours 
        Mat findContoursInput = hsvThresholdOutput;
        findContours(findContoursInput, false, findContoursOutput); 
        System.out.println("find contours: "+ findContoursOutput.size());

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
		double filterContoursMinRatio = 0;
		double filterContoursMaxRatio = 1.0;
        filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, 
        filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, 
        filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, 
        filterContoursMaxRatio, filterContoursOutput);

        System.out.println("filter contours: "+filterContoursOutput.size());
    }


    /**
     * Segment an image based on hue, saturation, and value ranges.
     * @param input 
     * @param hue   
     * @param sat
     * @param val
     * @param out
     */
	private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val, Mat out) {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
        Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
            new Scalar(hue[1], sat[1], val[1]), out);
    }

    /**
     * 	//Sets the values of pixels in a binary image to their distance to the nearest black pixel.
     * @param input
     * @param externalOnly
     * @param contours
     */
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

    /**
     * Looks through the contours list to find tapes. Then puts those into the tapeList.
     */
    private void findTapes(){
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

        System.out.println("tape List: " + tapeList.size());
    }
    /**
     * Look through the tape list and find sets of tapes that make up a target, add them to the targetList.
     */
    private void findTargets(){                 
        Tape leftTape=null;
        Tape rightTape=null;

        for(int i=0; i<tapeList.size(); i++){       //iterate through the tape list
            System.out.println("tape i= " + i + "tape rotation= " +tapeList.get(i).rotation);
            if(tapeList.get(i).rotation<20){         //~15 degrees is the left tape of a target 
                leftTape = tapeList.get(i);
            }else{                                  //if not left, it is the right tape
                rightTape = tapeList.get(i);
            }

            if(leftTape != null & rightTape != null){               //if both tapes have been defined
                Target target = new Target(leftTape, rightTape);    //create a new target
                targetList.add(target);                             //add that target to the target list

                leftTape = null;                                    //reset the tapes for the next loop
                rightTape = null;
            }
        }
        System.out.println("target List: "+targetList.size());
    }
    /**
     * Runs calculations on all the targets in the targetList.
     */
    private void processTargets(){
        System.out.println("processing targets");
        for(int i=0; i<targetList.size(); i++){
            Target target = targetList.get(i);
            target.calcDimensions();
            target.calcDistance();
            target.calcAngle();
        }
    }
    /**
     * select the center most target
     */
    private void selectTarget(){
        selectedTarget = targetList.get(0);
        for (int i=1; i < targetList.size(); i++){
            if(targetList.get(i).angle < selectedTarget.angle){
                selectedTarget = targetList.get(i);
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
        rotation = Math.abs(rotRect.angle);             //angle is coming out negative, seting it to be positive
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

    Rect boundingRect;                                  //bounding rectangle around the target
    double width;                                       //the width of the target
    double height;                                      //the height of the target
    double center;

    double distance;                                    //distance from the robot
    double angle;                                       //angle to the robot
    /**
     * Constructor: Defines the left and right tapes of a target.
     * @param L The left tape object.
     * @param R The right tape object.
     */
    public Target(Tape L, Tape R){                      //constructor: defines the left and right tape
        leftTape = L;
        rightTape = R;
    }
                                                        //getters and setters for the rest of the variables 
    
    public void calcDimensions(){
      
        width = leftTape.boundingRect.x - rightTape.boundingRect.x + leftTape.boundingRect.width;
       
        center = (((width/2) + rightTape.boundingRect.x )-160)/160;        //the center of the target, in the normalized image
       
    }
    /**
     * Calculate the distance from the camera to the target based on its size.
     */
    public void calcDistance(){
        distance = 4937/width;
        /*
        equation from team 525:
         Target Distance: Image Inch width = (Target inch width / Target pixel width) * Image pixel width
                                           = (14.62 /Target pixel width) * 320
                                           = 4678 / Target pixel width
                          Distance = (Image inch width / 2) / Tan(Camera image angle / 2)
                                   = ((4678 / Target pixel width) / 2) / Tan(25.35)
                                   = (2339 / Tan(25.35)) / Target pixel width
                                   = 4937 / Target pixel width
        */
    }
    /**
     * Calculate the angle of the target based on its x value in the image.
     */
    public void calcAngle(){
        angle = (double) Math.atan2(center, (double) 2.414) * (double) 180 / Math.PI;
        System.out.println("target center: " +center);
        System.out.println("target width: " +width);
        System.out.println("target angle: "+angle);
        /*
        equation from team 525:
         Target Angle:    Tan = x / y where x is the normalized location of the target center (-1 to 1)
                          When:  x = 1 when the Target Angle is 1/2 of camera view angle      (Axis=48.2/2; LifeCam=50.7/2)
                                 y = 1 / tan(25.35) = 2.11                                    (Axis=2.2355; LifeCam=2.11)
                                 
                                changed camera angle to 48 degrees
                                y= 1/ tan(24) = 2.246
                          Target Angle = atan(x, y)
                                       = atan(x, 2.11)
                                        =atan(x,2.246)
        */
    }

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