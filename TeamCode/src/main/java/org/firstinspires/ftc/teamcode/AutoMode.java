/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import java.util.List;


/*
 * This OpMode illustrates the basics of TensorFlow Object Detection, using
 * the easiest way.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */

public abstract class AutoMode extends RobotParent {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    protected static final int DIRECTION_RIGHT = 1;
    protected static final int DIRECTION_LEFT = -1;
    private int direction = DIRECTION_RIGHT; // set default direction to right for sliding

    /**
     * Which path do we need to take to drop of the purple pixel. Outside is toward the outer walls
     * while inside is toward the center gate/barrier.
     */
    protected static final int PATH_INSIDE = 1;
    protected static final int PATH_CENTER = 2;
    protected static final int PATH_OUTSIDE = 3;

    /** Used to override the locate prop logic to force a location, used by subclasses for testing */
    protected int testLocation = 0;

    /**
     * The April tag IDs for each location on the backdrops
     */
    protected static final int BLUE_LEFT_TAG = 1;
    protected static final int BLUE_CENTER_TAG = 2;
    protected static final int BLUE_RIGHT_TAG = 3;
    protected static final int RED_LEFT_TAG = 4;
    protected static final int RED_CENTER_TAG = 5;
    protected static final int RED_RIGHT_TAG = 6;

    protected int targetTagID;

    protected String debugMsg = "";

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;
    private AprilTagProcessor aprilTag;


    /**
     * The variable to store our instance of the vision portal.
     */

    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        initHardware();
        initCamera();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();
        closeLeftClaw();
        closeRightClaw();

        if (opModeIsActive()) {

            if ((isRed() && isFar()) || (isBlue() && isNear())) //set slide direction to left
            {
                direction = DIRECTION_LEFT;
            }
            telemetry.addData("direction: ", direction);
            encoderDrive(DRIVE_SPEED, 3, 3, 3, 3, 5.0);

            if((isBlue()&&isNear())||(isRed()&&isFar())){
                slide(direction,6);
            }
            int location = locateProp(); // identify where is the team prop
            targetTagID = getTargetTagIDForLocation(location);

            telemetry.addData("Position x: ", location);
            // move to the prep pos before drop off the pixel
            if (location == PATH_CENTER || location == PATH_OUTSIDE) {
                if((isBlue()&&isNear())||(isRed()&&isFar())){
                    slide(direction, 16);
                }
                else {
                    slide(direction, 22);
                }
                encoderDrive(DRIVE_SPEED, 37, 37, 37, 37, 20.0);
                if (location == PATH_CENTER) {
                    slide(-direction, 18);
                    dropPixel();
                } else {
                    slide(-direction, 11);
                    dropPixel();
                    if((isBlue()&&isFar())&&(isRed()&&isNear())){
                        slide(-direction, 7);
                    }
                    sleep(100);
                }
                encoderDrive(DRIVE_SPEED, 3, 3, 3, 3, 5.0);
            }

            if (location == PATH_INSIDE){
                encoderDrive(DRIVE_SPEED, 17,17,17,17,10.0);
                if(isBlue()&&isNear()||isRed()&&isFar()){
                    turnLeft90();
                    encoderDrive(DRIVE_SPEED,-7,-7,-7,-7,5.0);
                    dropPixel();
                    encoderDrive(DRIVE_SPEED,7,7,7,7,5.0);
                }
                else{
                    turnRight90();
                    encoderDrive(DRIVE_SPEED,-5,-5,-5,-5,5.0);
                    dropPixel();
                    encoderDrive(DRIVE_SPEED,5,5,5,5,5.0);
                }

                slide(-direction,25);
                if(isBlue()&&isNear()||isRed()&&isFar()){
                    turnRight90();
                }
                else{
                    turnLeft90();
                }
            }

            // different scenerio



            // turn 90 degres angles before moving through the bar
            if (isBlue()){
                encoderDrive(DRIVE_SPEED,   -17.25, 17.25, -17.25, 17.25,4.0);
            }
            else {
                // safety margin to clear bar
                encoderDrive(DRIVE_SPEED, 2, 2, 2, 2, 10.0); // back up to drop the pixel
                encoderDrive(DRIVE_SPEED,   17, -17, 17, -17,4.0);
            }
            // if far, drive to the position where close one ends
            if (isFar()){
                    encoderDrive(DRIVE_SPEED, 78, 78, 78, 78, 10.0); // back up to drop the pixel
            }
            else{
                encoderDrive(DRIVE_SPEED, 13, 13, 13, 13, 10.0); // back up to drop the pixel
                // Running off into other team backstage and driving into board
                //slide(-direction,40);
                //encoderDrive(DRIVE_SPEED, 15, 15, 15, 15, 10.0); // back up to drop the pixel
            }

            if ((isBlue() && isNear()) || (isRed()) && isFar()){
                direction = -direction;
            }

            if(justPark()){
                if(isRed()){
                    slide(direction,5);
                }
                else{
                    slide(-direction,5);
                }
                encoderDrive(DRIVE_SPEED, 20,20,20,20, 5.0);
                openLeftClaw();
                openRightClaw();
                sleep(200);
                closeRightClaw();
                closeLeftClaw();
                return;
            }
            // turn to face the board

            if(isRed()){
                slide(direction,26);
            }
            else{
                slide(-direction,30);
            }
            sleep(200);
            closeRightClaw();
            closeLeftClaw();
            moveToTag();
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    private int getTargetTagIDForLocation(int location) {
        if (isBlue()) {
            if (isNear()) {
                switch (location) {
                    case PATH_INSIDE:
                        return BLUE_RIGHT_TAG;
                    case PATH_CENTER:
                        return BLUE_CENTER_TAG;
                    case PATH_OUTSIDE:
                        return BLUE_LEFT_TAG;
                }
            }
            else {
                switch (location) {
                    case PATH_INSIDE:
                        return BLUE_LEFT_TAG;
                    case PATH_CENTER:
                        return BLUE_CENTER_TAG;
                    case PATH_OUTSIDE:
                        return BLUE_RIGHT_TAG;
                }
            }
        }
        else {
            if (isNear()) {
                switch (location) {
                    case PATH_INSIDE:
                        return RED_LEFT_TAG;
                    case PATH_CENTER:
                        return RED_CENTER_TAG;
                    case PATH_OUTSIDE:
                        return RED_RIGHT_TAG;
                }
            }
            else {
                switch (location) {
                    case PATH_INSIDE:
                        return RED_RIGHT_TAG;
                    case PATH_CENTER:
                        return RED_CENTER_TAG;
                    case PATH_OUTSIDE:
                        return RED_LEFT_TAG;
                }
            }
        }

        return 0;
    }

    //    determine red & blue, far & near
    abstract protected TfodProcessor getProcessor();
    private boolean isRed(){return !isBlue();}
    private boolean isFar(){return !isNear();}

    protected boolean justPark() { return false; }
    abstract protected boolean isBlue();
    abstract protected boolean isNear();

    /**
     * Initialize the TensorFlow Object Detection processor.
     */

    private void initCamera() {

        // Create the TensorFlow processor the easy way.
//        tfod = TfodProcessor.easyCreateWithDefaults();
//        String[] labels = {"Red Prop"};

        tfod = getProcessor();
        aprilTag = new AprilTagProcessor.Builder().build();
        // Create the vision portal the easy way.
        // visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam1"), tfod);
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam1"))
                .addProcessor(tfod)
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();


        //

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */

    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        telemetry.addData("# Debug Message", debugMsg);

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

    private void moveToTag(){
        boolean atDestination = false;
        while(!atDestination && opModeIsActive()){
            AprilTagDetection tagData = getBestTagForLocation();
            if(tagData == null){
                break;
            }
            atDestination = driveTowardTag(tagData);

            telemetryAprilTag();
            telemetry.update();
        }
        backDrop();
    }


    private AprilTagDetection getBestTagForLocation() {
        List<AprilTagDetection> currentDetections = null;
        for (int x = 0; x<10; x++){
            debugMsg = "Tries:" + x;
            telemetryAprilTag();
            telemetry.update();
            sleep(100);
            currentDetections = aprilTag.getDetections();
            if (currentDetections.size()>0){
                debugMsg = "current detections size:" + currentDetections.size();
                break;
            }
        }

        AprilTagDetection bestMatch = null;
        for (AprilTagDetection detection : currentDetections) {
            if (bestMatch == null){
                bestMatch = detection;
                continue;
            }
            if (Math.abs(bestMatch.id - targetTagID) > (Math.abs(detection.id - targetTagID)))
            {
                bestMatch = detection;
            }
        }
        if(bestMatch == null){
            debugMsg = "best match null:";
            telemetryAprilTag();
            telemetry.update();
            return null;
        }

        debugMsg = "best match:" + bestMatch.id;
        telemetryAprilTag();
        telemetry.update();
        return bestMatch;
    }

    private boolean driveTowardTag(AprilTagDetection tagData){
        int tagId = tagData.id;
        double tagDistance = (tagId - targetTagID)*8-tagData.ftcPose.x;

        if (tagData.ftcPose.yaw>5 || tagData.ftcPose.yaw<-5){
            turn(-1, tagData.ftcPose.yaw*0.2);
            debugMsg += "yaw = " + tagData.ftcPose.yaw + ":";
            return false;
        }
        if (tagDistance>2 || tagDistance < -2){
            debugMsg += "slide = " + tagDistance + ":";
            telemetryAprilTag();
            telemetry.update();
            slide(-1,(int)(tagDistance));
            return false;
        }
        if(tagData.ftcPose.y<13 && tagData.ftcPose.y>11){
            return true;
        }
        double driveYDistance = tagData.ftcPose.y - 12;
        encoderDrive(0.5, driveYDistance,driveYDistance,driveYDistance,driveYDistance, 5.0);
        return false;
    }
    private int locateProp() {
        int location = 0;
        if (testLocation > 0){
            return testLocation;
        }
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        int trys = 1;
        while (currentRecognitions.size() == 0 && trys < 15) {
            telemetry.addData("First recognization failed, try times", trys);
            telemetry.update();
            sleep(100);
            currentRecognitions = tfod.getRecognitions();
            trys++;
        }

        if (currentRecognitions.size() == 1) { // if team prop is recognized at straight position
            Recognition found = currentRecognitions.get(0);
            if((isBlue() && isFar())||(isRed())&&isNear()) {
                if(found.getLeft() < 320)
                {
                    location = PATH_CENTER;
                }
                else{
                    location = PATH_OUTSIDE;
                }
                telemetry.addData("Position method recognized: ", location);
            }
            else{
                if(found.getLeft() > 320)
                {
                    location = PATH_CENTER;
                }
                else{
                    location = PATH_OUTSIDE;
                }
                telemetry.addData("Position method recognized: ", location);
            }
            telemetry.update();

//            slide(direction,10);
        }
        else {
           location = PATH_INSIDE;
        }
        return location;
    }

    private void slide (int direction, int inches){
        if (direction == DIRECTION_RIGHT){ // slide to right
            encoderDrive(0.4, inches , -inches, -inches, inches, 10.0);
        }
        if (direction == DIRECTION_LEFT){ // slide to left
            encoderDrive(0.4, -inches , inches, inches, -inches, 10.0);
        }
    }


    private void turn (int direction, double degrees){
        if (direction == DIRECTION_RIGHT) { // turn right in a cerntain degrees
            encoderDrive(DRIVE_SPEED, degrees, -degrees, degrees, -degrees, 10.0);
        }
        if (direction == DIRECTION_LEFT) { // turn right in a cerntain degrees
            encoderDrive(DRIVE_SPEED, -degrees, degrees, -degrees, degrees, 10.0);
        }
    }
    // drop pixel at the line
    private void dropPixel(){
        if((isRed()&&isFar()) || (isBlue() && isNear()) ){
            openRightClaw();
        }
        else{
            openLeftClaw();
        }
    }
    protected void backDrop(){

        closeRightClaw();
        closeLeftClaw();

        if((isBlue()&&isNear()) || (isRed()&& isFar())){
            slide(-1,-4);
        }

        if(isRed()){
            slide(1,2);
        }

        moveArmUp();
//        encoderDrive(DRIVE_SPEED,   1, 1, 1, 1,4.0);
        openRightClaw();
        openLeftClaw();
        sleep(500);
        moveArmDown();
        sleep(1500);
        if(isNear()){
            if(isBlue()){
                int slideDistance = 16 + targetTagID*6;
                slide(-1,slideDistance);
                encoderDrive(DRIVE_SPEED,10,10,10,10, 5.0);
            }
            else{
                int slideDistance = 16 + (7-targetTagID)*6;
                slide(1,slideDistance);
                encoderDrive(DRIVE_SPEED,5,5,5,5, 5.0);
            }
        }
    }
    protected void turnLeft90(){
        encoderDrive(DRIVE_SPEED,   -17.5, 17.5, -17.5, 17.5,4.0);
    }
    protected void turnRight90(){
        encoderDrive(DRIVE_SPEED,   17.5, -17.5, 17.5, -17.5,4.0);
    }
}   // end class

