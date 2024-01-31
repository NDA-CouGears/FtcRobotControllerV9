/*
    author: delia jasper
    purpose: another way to drive using mecanum wheels
 */

// i want to try this out soon and see if a it works and if b if the drivers like robot or field centric better

package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.io.FileOutputStream;
import java.util.List;
import java.util.Properties;


abstract public class RobotParent extends LinearOpMode {

    // initialize narrators
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor winchMotor = null;
    private DcMotor armMotor = null;
    private Servo clawLeftServo;
    private Servo clawRightServo;
    private Servo wristServo;
    private Servo droneServo;
    private TouchSensor touchSensor;


    // final variables
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_MM = 97;
    static final double COUNTS_PER_MOTOR_REV = 1440;
    static final double COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * 3.1415);
    private ElapsedTime runtime = new ElapsedTime();

//    // final variables
//    static final double DRIVE_GEAR_REDUCTION = 1.0;
//    static final double WHEEL_DIAMETER_MM = 37.0;
//    static final double COUNTS_PER_MOTOR_REV = 1440;
//    static final double middleServo = 58;
//    static final double middleServo2= 77;
//    static final double topServo = 67;
//    static final double topServo2 = 96;
//    static final double servoMax = 69;
//    static final double servo2Max = 96;

    static final double clawLeftServoOpen = .5;
    static final double clawLeftServoClosed = 0.99;
    static final double clawRightServoOpen = 0.785;
    static final double clawRightServoClosed = 0.278;
    static final double wristServoFloor = 0.38667;
    static final double wristServoBoardTop = 1.0;
    static final double wristServoBoardBottom = 0.709;
    static final double armPosFloor = 5000;
    static final double armPosBoardTop = 14410;
    static final double armPosBoardBottom = 17194;
    static final double droneServoLocked = 1;
    static final double droneServoLaunch = 0.38;
    private boolean winchLock = false;
    private double winchLockPos = 0;
    private boolean winchErrorCorrector = false;
    protected void encoderDrive(double speed,
                                double leftFrontInches,
                                double rightFrontInches,
                                double leftBackInches,
                                double rightBackInches,
                                double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;
        speed = 0.5;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() - (int) (25.4 * leftFrontInches * COUNTS_PER_MM);
            newLeftBackTarget = leftBackDrive.getCurrentPosition() - (int) (25.4 * leftBackInches * COUNTS_PER_MM);
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() - (int) (25.4 * rightFrontInches * COUNTS_PER_MM);
            newRightBackTarget = rightBackDrive.getCurrentPosition() - (int) (25.4 * rightBackInches * COUNTS_PER_MM);
            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("Currently at", " at %7d :%7d",
                    leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                    rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
            telemetry.update();

            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);

            telemetry.addData("Moved to", " %7d :%7d", newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget);
            telemetry.update();

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    protected void encoderArmDrive(double speed,
                                double armPos,
                                double timeoutS) {

        speed = 0.5;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            armMotor.setTargetPosition((int)armPos);

            // Turn On RUN_TO_POSITION
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("Currently at", " at %7d",
                    armMotor.getCurrentPosition());
            telemetry.update();

            // reset the timeout time and start motion.
            runtime.reset();
            armMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) && touchSensor.isPressed() && armMotor.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7f", armPos);
                telemetry.addData("Currently at", " at %7d", armMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            armMotor.setPower(0);


            telemetry.addData("Moved to", " %7f", armPos);
            telemetry.update();

            // Turn off RUN_TO_POSITION
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    protected void mecanumDrive() {
        double y = signPreserveSquare(gamepad1.left_stick_y); // Remember, this is reversed!
        double x = signPreserveSquare(gamepad1.left_stick_x * -1.1); // Counteract imperfect strafing
        double rx = signPreserveSquare(gamepad1.right_stick_x * -1);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFrontDrive.setPower(frontLeftPower);
        leftBackDrive.setPower(backLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        rightBackDrive.setPower(backRightPower);
    }

    protected void initHardware() {

        // Initialize the drive system variables.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive"); //port 0 on control hub
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive"); //port 1 on control hub
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive"); //port 0 on ex hub
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive"); //port 1 on ex hub
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor"); // port 2 on control hub
        winchMotor = hardwareMap.get(DcMotor.class, "winch_motor"); // port 3 on control hub
        clawLeftServo = hardwareMap.servo.get("servo1"); // port 0 on ex hub
        clawRightServo = hardwareMap.servo.get("servo2"); // port 1 ex hub
        wristServo = hardwareMap.servo.get("servo3"); // port 2 ex hub
        droneServo = hardwareMap.servo.get("servo4"); //port 3 on ex hub
        touchSensor = hardwareMap.get(TouchSensor.class, "arm_zero");

        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // setting direction for motors
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        winchMotor.setDirection(DcMotor.Direction.FORWARD);

        //Setting Encorders for motors
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        winchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Turns off the blinking LED
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setConstant(0);
        }
    }

    private double signPreserveSquare(double value) {
        wristServo.getController();
        if (value > 0) {
            return value * value;
        }
        else {
            return -(value * value);
        }
    }

    protected void winch() {
        double winchPower = signPreserveSquare(gamepad2.right_stick_y);
        winchPower = Range.clip(winchPower, -1.0, 1.0);
        winchMotor.setPower(winchPower);
        if (gamepad2.b){
            winchLock = !winchLock;
            winchLockPos = winchMotor.getCurrentPosition();
        }
        if(winchLock){
            if (winchMotor.getCurrentPosition() > (winchLockPos + 20)){
                winchErrorCorrector = true;
            }
            if(winchErrorCorrector){
                winchMotor.setPower(-0.5);
            }
            if(winchMotor.getCurrentPosition() <= (winchLockPos - 20)){
                winchErrorCorrector = false;
            }
        }
    }

    protected void launchDrone(){
        droneServo.setPosition(droneServoLaunch);
    }
    protected void resetDrone(){
        droneServo.setPosition(droneServoLocked);
    }

    protected void openLeftClaw(){
        clawLeftServo.setPosition(clawLeftServoOpen);
    }
    protected void closeLeftClaw(){
        clawLeftServo.setPosition(clawLeftServoClosed);
    }

    protected void openRightClaw(){
        clawRightServo.setPosition(clawRightServoOpen);
    }
    protected void closeRightClaw(){
        clawRightServo.setPosition(clawRightServoClosed);
    }

    protected void moveArmUp(){
        wristServo.setPosition(wristServoBoardTop);
        encoderArmDrive(1.0, armPosBoardTop-2150, 20);
    }
    protected void moveArmDown(){
        wristServo.setPosition(wristServoFloor);
        encoderArmDrive(1.0, armPosFloor, 20);
    }

    protected void drone(){
        if(gamepad1.x && gamepad1.b){
            launchDrone();
        }
        if(gamepad1.a){
            resetDrone();
        }
    }
    protected void ArmAndWrist() {

        double armPower = signPreserveSquare(-gamepad2.left_stick_y);
        double armPos = armMotor.getCurrentPosition();
        armPower = Range.clip(armPower, -1.0, 1.0);
        if (!touchSensor.isPressed()) {
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (armPower < 0)
                armPower = 0;
        }
        armMotor.setPower(armPower);

        double wristPos = 0;
        if (armPos > armPosFloor) {
            if (armPos < armPosBoardTop) {
                double wristAdjust = (armPos-armPosFloor)/(armPosBoardTop-armPosFloor);
                wristPos = wristServoFloor + (wristServoBoardTop-wristServoFloor) * wristAdjust;
                wristServo.setPosition(wristPos);
            }
            else if (armPos >= armPosBoardTop && armPos < armPosBoardBottom) {
                double wristAdjust = (armPos-armPosBoardTop)/(armPosBoardBottom-armPosBoardTop);
                wristPos = wristServoBoardTop + (wristServoBoardBottom-wristServoBoardTop) * wristAdjust;
                wristServo.setPosition(wristPos);
            }
        }
        else {
            if (gamepad2.dpad_down) {
                wristServo.setPosition(wristServo.getPosition()+.01);
            }
            if (gamepad2.dpad_up) {
                wristServo.setPosition(wristServo.getPosition()-.01);
            }
        }

//        if(gamepad2.y){
//            armMotor.setTargetPosition((int)armPosFloor);
//            // Turn On RUN_TO_POSITION
//            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            armMotor.setPower(1);
//            wristServo.setPosition(wristServoFloor);
//            runtime.reset();
//            while (opModeIsActive() &&
//                    (runtime.seconds() < 30)&&
//                    (armMotor.isBusy())) {
//                telemetry.addData("Anything", "");
//            }
//
//            // Stop all motion;
//            armMotor.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }

        telemetry.addData("Motors", "arm Power(%.2f)", armPower);
        telemetry.addData("Motors", "arm Position(%.2f)", armPower);
        telemetry.addData("Servos", "wrist (%.2f)", wristPos);
    }


    protected void claw() {
        boolean leftBumper = gamepad2.left_bumper;
        boolean leftTrigger = gamepad2.left_trigger > 0.5;
        boolean rightBumper = gamepad2.right_bumper;
        boolean rightTrigger = gamepad2.right_trigger > 0.5;

        if (leftBumper) {
            openLeftClaw();
        }
        if (leftTrigger) {
            closeLeftClaw();
        }

        if (rightBumper) {
            openRightClaw();
        }
        if (rightTrigger) {
            closeRightClaw();
        }
    }

    protected void calibrationAndTelemetry() {
        double servoDelta = 0.005;

        double armPosit = armMotor.getCurrentPosition(); //gamepad1.left_stick_y
        double armPow = signPreserveSquare(gamepad1.left_stick_y);
        armPow = Range.clip(armPow, -1.0, 1.0);
        if (!touchSensor.isPressed()) {
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (armPow < 0)
                armPow = 0;
        }
        armMotor.setPower(armPow);

        double winchPosit = winchMotor.getCurrentPosition(); //gamepad2.right_stick_y
        double winchPow = signPreserveSquare(gamepad2.right_stick_y);
        winchPow = Range.clip(winchPow, -1.0, 1.0);
        winchMotor.setPower(winchPow);

        double leftClawPosit = clawLeftServo.getPosition(); //gamepad1.left_stick_x
        leftClawPosit = leftClawPosit + (signPreserveSquare(gamepad1.left_stick_x)*servoDelta);
        clawLeftServo.setPosition(leftClawPosit);

        double rightClawPosit = clawRightServo.getPosition(); //gamepad1.right_stick_x
        rightClawPosit = rightClawPosit + (signPreserveSquare(gamepad1.right_stick_x)*servoDelta);
        clawRightServo.setPosition(rightClawPosit);

        double wristPosit = wristServo.getPosition(); //gamepad1.dpad
        if(gamepad1.dpad_down){
            wristPosit -= servoDelta;
        }
        else if(gamepad1.dpad_up){
            wristPosit += servoDelta;
        }
        wristServo.setPosition(wristPosit);

        double dronePosit = droneServo.getPosition(); //gamepad2.dpad
        if(gamepad2.dpad_down){
            dronePosit -=servoDelta;
        }
        else if(gamepad2.dpad_up){
            dronePosit += servoDelta;
        }
        droneServo.setPosition(dronePosit);

        /*
        //Save
        if(gamepad1.y){
            Properties savedVal = new Properties();
            savedVal.setProperty("armMax", Double.toString(armPosit));
            savedVal.setProperty("winchMax", Double.toString(winchPosit));
            savedVal.setProperty("leftClawMax", Double.toString(leftClawPosit));
            savedVal.setProperty("rightClawMax", Double.toString(rightClawPosit));
            savedVal.setProperty("wristMax", Double.toString(wristPosit));
            String propPath = String.format("%s/caliration.properties", Environment.getExternalStorageDirectory().getAbsolutePath());
            try {
                FileOutputStream propStream = new FileOutputStream(propPath);
                savedVal.store(propStream, "Calibration Vals");
            }
            catch (Exception ex){
                telemetry.addData("exception: ", ex.getMessage());
            }
        }
         */


        telemetry.addData("ArmPos: ", armPosit);
        telemetry.addData("WinchPos: ", winchPosit);
        telemetry.addData("LClawPos: ", leftClawPosit);
        telemetry.addData("RClawPos: ", rightClawPosit);
        telemetry.addData("WristPos: ", wristPosit);
        telemetry.addData("dronePos: ", dronePosit);
        telemetry.addData("touch sensor is : ", !touchSensor.isPressed());

    }
}

