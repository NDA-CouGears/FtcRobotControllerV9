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
    private static final String TFOD_MODEL_FILE ="file:///android_asset/quuen1.tflite" ;
    private int direction = 1; // set default direction to right for sliding


    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */

    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        initHardware();
        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();
        this.closeLeftClaw();
        this.closeRightClaw();

        if (opModeIsActive()) {
            telemetryTfod();
            // Push telemetry to the Driver Station.
            telemetry.update();


            if ((isRed() && isFar()) || (isBlue() && isNear())) //set slide direction to left
            {
                direction = -1;
            }
            telemetry.addData("direction: ", direction);
            int location = locateProp(); // identify where is the team prop
            telemetry.addData("Position x: ", location);
            // move to the prep pos before drop off the pixel
            slide(direction, 25);
            encoderDrive(DRIVE_SPEED, 35, 35, 35, 35, 20.0);

            // different scenerio

            if (location == 2){
                slide (-direction, 16);
                dropPixel();
                encoderDrive(DRIVE_SPEED, 5, 5, 5, 5, 10.0); // back up to drop the pixel
            }

            else if (location == 3){
                slide(-direction, 12);
                encoderDrive(DRIVE_SPEED, -5, -5, -5, -5, 10.0); // back up to drop the pixel
                dropPixel();
                slide(-direction, 4);
                encoderDrive(DRIVE_SPEED, 10, 10, 10, 10, 10.0); // back up to drop the pixel
            }

            else if (location == 1){
                slide (-direction, 25);
                encoderDrive(DRIVE_SPEED, -9, -9, -9, -9, 10.0); // back up to drop the pixel
                turn(direction,10);
                encoderDrive(DRIVE_SPEED, -6, -6, -6, -6, 10.0); // back up to drop the pixel
                dropPixel();
                encoderDrive(DRIVE_SPEED, 6, 6, 6, 6, 10.0); // back up to drop the pixel
                turn (-direction,10);
                encoderDrive(DRIVE_SPEED, 14, 14, 14, 14, 10.0); // back up to drop the pixel
            }

            else{ // identify failed, can circle back to gain some basic points

            }

            // turn 90 degress angles before moving through the bar
            if (isBlue()){
                turnLeft90();
            }
            else {
                // safety margin to clear bar
                encoderDrive(DRIVE_SPEED, 2, 2, 2, 2, 10.0); // back up to drop the pixel
                turnRight90();
            }
            // if far, drive to the position where close one ends
            if (isFar()){
                encoderDrive(DRIVE_SPEED, 95, 95, 95, 95, 10.0); // back up to drop the pixel
            }
            else{
                encoderDrive(DRIVE_SPEED, 40, 40, 40, 40, 10.0); // back up to drop the pixel
                // Running off into other team backstage and driving into board
                //slide(-direction,40);
                //encoderDrive(DRIVE_SPEED, 15, 15, 15, 15, 10.0); // back up to drop the pixel
            }

            // turn to face the board

            sleep(20);
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

//    determine red & blue, far & near
    abstract protected TfodProcessor getProcessor();
    private boolean isRed(){return !isBlue();}
    private boolean isFar(){return !isNear();}
    abstract protected boolean isBlue();
    abstract protected boolean isNear();

    /**
     * Initialize the TensorFlow Object Detection processor.
     */

    private void initTfod() {

        // Create the TensorFlow processor the easy way.
//        tfod = TfodProcessor.easyCreateWithDefaults();
//        String[] labels = {"Red Prop"};

        tfod = getProcessor();
        // Create the vision portal the easy way.
        // visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam1"), tfod);
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam1"))
                .addProcessor(tfod)
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


    private int locateProp() {
        int location = 0;

        encoderDrive(DRIVE_SPEED, 7 , 7, 7, 7, 5.0);
        sleep(100);
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
            location = 2;
            telemetry.addData("Position method recognized: ", location);

//            slide(direction,10);
        }
        else {
            if (isRed()){
                slide(direction,3);
            }
            turn (direction, 4.725);         //move to the right a bit to identify prop on the right position
             trys = 1;
            while (currentRecognitions.size() == 0 && trys < 15) {
                telemetry.addData("Second recognization failed, try times", trys);
                telemetry.update();
                sleep(100);
                currentRecognitions = tfod.getRecognitions();
                trys++;
            }
            if (currentRecognitions.size() == 1) { // if team prop is recognized at right position
                location = 3;
                telemetry.addData("Position method recognized: ", location);
                turn (-direction,4.725);
            }
            else{
                location = 1;
                telemetry.addData("Position method recognized: ", location);
                turn(-direction,4.725);
            }
            if (isRed()){
                slide(-direction,3);
            }
        }
        return location;
    }

    private void slide (int direction, int inches){
        if (direction == 1){ // slide to right
            encoderDrive(DRIVE_SPEED, inches , -inches, -inches, inches, 10.0);
        }
        if (direction == -1){ // slide to left
            encoderDrive(DRIVE_SPEED, -inches , inches, inches, -inches, 10.0);
        }
    }


    private void turn (int direction, double degrees){
        if (direction == 1) { // turn right in a cerntain degrees
            encoderDrive(DRIVE_SPEED, degrees, -degrees, degrees, -degrees, 10.0);
        }

        if (direction == -1) { // turn right in a cerntain degrees
            encoderDrive(DRIVE_SPEED, -degrees, degrees, -degrees, degrees, 10.0);
        }
    }
    // drop pixel at the line
    private void dropPixel(){
        if(isRed()){
            openRightClaw();
        }
        else{
            openLeftClaw();
        }
    }

    protected void turnLeft90(){
        encoderDrive(DRIVE_SPEED,   -18, 18, -18, 18,4.0);
    }
    protected void turnRight90(){
        encoderDrive(DRIVE_SPEED,   17.5, -17.5, 17.5, -17.5,4.0);
    }
}   // end class
