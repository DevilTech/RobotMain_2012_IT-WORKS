/*

This class represent the turret of the 2012 robot.

It is a thread so that auto-tracking runs in the background in terms of the rest
of the robot operation.

*/

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport;



//------------------------------------------------------------------------------
public class Turret extends Thread {
//------------------------------------------------------------------------------

    public class FoundRectangle {
        public double target;
        public double distance;
        public boolean confidence;
        public int pixelWidth;

        public FoundRectangle(double target, double dist, boolean confidence, int pixelWidth) {
            this.target = target;
            this.distance = dist;
            this.confidence = confidence;
            this.pixelWidth = pixelWidth;
        }
    }

    /**
     * sweep back and forth between positive and negative angles
     */
    class Sweeper {
        int state = 0;
        double val[];

        /**
         * create a sweeper class
         *
         * @param val an array of alternating (pod/neg) angles. if first pos, last should be neg/
         */
        public Sweeper(double[] val) {
            this.val = val;
        }

        /**
         * make sweep start over
         */
        public void reset() {
            state = 0;
        }

        /**
         * do one iteration of the sweep returning a relative movement value
         *
         * @param turretAngle specify the current turret position
         * @param inc the value of the increment passed back
         * @return the increment as either positive or negative for direction
         */
        public double sweep(double turretAngle, double inc) {
            for (int i=0; i<3; i++) {
                if (state >= val.length)
                    state = 0;

                double sval = val[state];

                if (sval < 0) {
                    if (turretAngle < sval)
                        state++;
                    else
                        return Math.min(sval, -inc);
                } else {
                    if (turretAngle > sval)
                        state++;
                    else
                        return Math.min(sval, inc);
                }
            }
            return 0;
        }
    }

    final int TARGET_LOCK_RANGE_PIXELS = 5;   // offset around centered for tracking

    final int STATE_TRACK           = 1;
    final int STATE_SPECIAL_SEARCH  = 2;
    final int STATE_BACK            = 3;
    final int STATE_GENERAL_SEARCH  = 4;
    final int STATE_GENERAL_SEARCH1 = 5;

    int state = STATE_GENERAL_SEARCH;

    final int camera_Angle = 22; //HARTFORD CHANGE "22 for original angle"

    final int RGB_FILTER_R_MIN =   0; //0 HARTFORD CHANGE
    final int RGB_FILTER_R_MAX = 150; //150
    final int RGB_FILTER_G_MIN = 200; //200
    final int RGB_FILTER_G_MAX = 255; //255
    final int RGB_FILTER_B_MIN = 100; //100
    final int RGB_FILTER_B_MAX = 255; //255

    final double qualityMin = 20.0;
    final double qualityMax = 40.0;

    final double SMALLEST_TARGETING_INCREMENT_DEGREES  = 0.13;
    double lsInc = 8.6;  // degrees
    double lastDistance = 15.0;

    // used to control whether tracking thread is enabled
    boolean image = true;
    boolean shouldTrack = false;
    boolean holdHome = false;
    boolean testState = false;
    boolean manualTrack = false;
    boolean isTargetLocked = false;

    FoundRectangle result;
    AxisCamera cam;
    DriverStationLCD screen;
    TurretRotationControl turretRCntl = new TurretRotationControl();

    // define a sweep pattern relative to 0
    double s[] = {3, -5, 10, -15, 20, -25, 5, -10, 25, -30, 30, -3};
    Sweeper sweeper = new Sweeper(s);


    //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    public void run() {
    //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        ColorImage img;
        BinaryImage binImg;
        double screenCenter;
        ParticleAnalysisReport[] particle;

        cam = AxisCamera.getInstance();
        screen = DriverStationLCD.getInstance();

        // assume the robot is correctly aligned at power up

        try {
            System.out.println("\n $$$$$$$$$$$$$$$$$ THREAD IS HERE $$$$$$$$$$$$$$$$$$\n");

            while (true) {

                if (!cam.freshImage()) {
                    yield();
                    continue;
                }

               // sleep1(3000);

                isTargetLocked = false;



                //
                // image acquistion and basic analysis
                //
                img = cam.getImage();
                if (image)
                {
                img.write("/ColorImage.jpg");
                image  = false;
                }
                binImg = img.thresholdRGB(RGB_FILTER_R_MIN, RGB_FILTER_R_MAX, RGB_FILTER_G_MIN, RGB_FILTER_G_MAX, RGB_FILTER_B_MIN, RGB_FILTER_B_MAX);

                img.free();

                particle = binImg.getOrderedParticleAnalysisReports(10);
                screenCenter = binImg.getWidth()/2.0;
                binImg.free();


                result = isFound(particle);
                if (result != null) {
                    double error = result.target - screenCenter;

                    //System.out.println("\nTargeting Error (pixels)" + error);

                    double d = result.distance/12;
                    int ft = (int)Math.floor(d);
                    //System.out.println("Distance: " + ft + " ft " + (d-ft)*12 + " in");
                    screen.println(DriverStationLCD.Line.kMain6, 1, "Distance: " + ft + " ft " + (d - ft) * 12 + " in");


                    // see if we are close enough
                    if (Math.abs(error) <= TARGET_LOCK_RANGE_PIXELS) {
                        isTargetLocked = result.confidence;
                        //System.out.println("\n############################## TRACKING LOCK ###############################\n");
                        screen.println(DriverStationLCD.Line.kUser5, 1, "------@@@@@@@@@@@@@@@@@@@@@@@@TARGET LOCK@@@@@@@@@@@@@@@@@@@@@---------");
                    // else apply correction
                    }else{
                        screen.println(DriverStationLCD.Line.kUser5, 1, "                                                                                       ");
                    }
                    screen.updateLCD();
                    if (doAim()){
                        double ca = getTargetingCorrectionAngle(error, result.distance, result.pixelWidth);
                       // System.out.println("Tracking (correction angle applied): " + ca);
                        turretRCntl.setAngleRelative(ca, 1000);
                    }
                } else{
                    if(doAim())
                    {
                        turretRCntl.goHome();
                    }
                }


                //
                // do we see the rectangles
                //

            }

        } catch (AxisCameraException ex) {
            ex.printStackTrace();
        } catch (NIVisionException ex) {
            ex.printStackTrace();
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
        }

        try {
            turretRCntl.goHome();
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
        }

        image = true;
    }


    /**
     * Calculates the relative movement angle needed to correct the targeting
     * error.
     *
     * @param  targetingErrorPixels specifies the number of pixels and direction of the error
     * @param  targetDistanceInches  specifies the distance from the target in inches
     * @return the relative angle in degrees that should be moved.
     */
    protected double getTargetingCorrectionAngle(double targetingErrorPixels, double targetDistanceInches, int pixelWidth) {
        double corr;
        double pixelPerDeg = Math.tan(Math.toRadians(1)) * targetDistanceInches * pixelWidth/24.0;
       // System.out.println("PixelWidth" + pixelWidth);
       // System.out.println("pixel per degrees  " + pixelPerDeg);

        corr = Math.abs(targetingErrorPixels / pixelPerDeg);

        corr = Math.max(corr, SMALLEST_TARGETING_INCREMENT_DEGREES);
        return targetingErrorPixels > 0.0 ? corr: -corr;
    }
    protected double getSearchCorrectionAngle(double targetDistanceInches) {
        double corr;
        if (targetDistanceInches < 10*12) {
            corr = 5;
        } else if (targetDistanceInches < 20*12) {
            corr = 10;
        } else {
            corr = 7;
        }

        return corr;
    }

    /**
     * returns the distance to the target in inches
     *
     * @param width  width of one of the found rectangles
     */
    protected double distanceInches(int width) {
        return 24 * 320 / (width * Math.tan(Math.toRadians(camera_Angle))); //HARTFORD CHANGE
    }


    /**
     * determines if the two center height rectangles are visible to the camera
     *
     * @param particle  array from the NIvision system
     # @return null if false else the data needed for tracking
     */
    protected FoundRectangle isFound(ParticleAnalysisReport[] particle) {
        int cnt = 0, rcnt = 0;
        double dist = 0.0;
        int left = 100000;
        int right = -100000;
        int top = 1000000;
        double topcenter = 10000000.0;
        double lavg = 0;
        int leftTop = 0, rightTop = 0, rightWidth = 0, pixelWidth = 0;
        //System.out.println();

        //
        // must search all found particles for rectangles (particles with desired a 'quality')
        //
        for (int i=0; i < particle.length; i++) {
            ParticleAnalysisReport p = particle[i];
            double ratio = p.boundingRectWidth * 1.0 / p.boundingRectHeight;
            //System.out.println("ratio  "+ratio);
            //System.out.println(+ i + " , " + p.boundingRectLeft + " , " + p.boundingRectTop+ " , " + p.particleQuality + " , " + p.boundingRectWidth + " , " + p.boundingRectHeight);

            // based on ratio
            if (1.15 < ratio && ratio < 1.45 && p.boundingRectWidth > 45) {

                if (p.boundingRectTop < top){
                    lastDistance = dist = distanceInches(p.boundingRectWidth);
                    top = p.boundingRectTop;
                    topcenter = p.boundingRectLeft + p.boundingRectWidth/2.0;
                }

                //System.out.println("ratio: " + ratio + "Left Top Coor. "+ p.boundingRectLeft+ " , "+ p.boundingRectTop);
                rcnt++;
            }

            // based on quality
            if (qualityMin < p.particleQuality && p.particleQuality < qualityMax) {
                if (cnt == 0){
                    lastDistance = dist = distanceInches(p.boundingRectWidth);
                    pixelWidth = p.boundingRectWidth;
                }
                cnt++;

                if (p.boundingRectLeft < left) {
                    left = p.boundingRectLeft;
                    leftTop = p.boundingRectTop;
                }

                if (p.boundingRectLeft > right){
                    right = p.boundingRectLeft;
                    rightTop = p.boundingRectTop;
                    rightWidth = p.boundingRectWidth;
                }
            } else {
                lavg += p.boundingRectTop;
            }
        }

        //System.out.println("Average Left:" + (lavg/(particle.length - cnt)));

        // see if we have the middle row
        if (cnt >= 2 && Math.abs(leftTop - rightTop) < 11) {
           // System.out.println("\n ####Two Rectangle Search#### \n");
            return new FoundRectangle(left + (right + rightWidth -left)/2.0, dist, true, pixelWidth);
        }
        else if (rcnt >= 1){
          //  System.out.println("\n ****Top Rectangle Search**** \n");
            return new FoundRectangle(topcenter, dist, rcnt > 1, pixelWidth);
        }
        else
            return null;
    }


    /**
     * check to see of the robot has it's back to our targets
     *
     * @return boolean of the check
     */

    public synchronized boolean doAim() {
        return manualTrack;
    }

    public synchronized void setAim(boolean manualTrack) {
        this.manualTrack = manualTrack;
    }


    public synchronized void setTestState(boolean testState) {
        this.testState = testState;
    }

    public synchronized boolean getTestState() {
        return testState;
    }

    protected void sleep1(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ex) {

        }
    }

    public void setX(double xVal){
        try {
            turretRCntl.setAngleRelative(xVal * 4.0, 10);
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
        }
    }

    /* try new light ,
     * fps for diff resolution: 640-480 is approx.4.5 FPS: 480-360 is approx. 10 FPS :320-240 is approx. 15 FPS: 240-180 is approx. 17 FPS: 160-120 is approx. 24 however it is inaccurate due to an occilating camera ,
     * gyro,
     * RGB far distance is  R: 0/150 G: 170/255 B: 0/255;
     * distance */
}
