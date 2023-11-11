/*
    author: delia jasper
    purpose: another way to drive using mecanum wheels
 */

// i want to try this out soon and see if a it works and if b if the drivers like robot or field centric better

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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

    static final double clawLeftServoMin = 0;
    static final double clawRightServoMin = 0;
    static final double clawLeftServoMax = 0;
    static final double clawRightServoMax = 0;
    static final double wristServoMin = 0;
    static final double wristServoMax = 0;
    static final double armPosMin = 0;
    static final double armPosMax = 0;


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

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int) (25.4 * leftFrontInches * COUNTS_PER_MM);
            newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int) (25.4 * leftBackInches * COUNTS_PER_MM);
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int) (25.4 * rightFrontInches * COUNTS_PER_MM);
            newRightBackTarget = rightBackDrive.getCurrentPosition() + (int) (25.4 * rightBackInches * COUNTS_PER_MM);
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
            sleep(500);

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

            telemetry.addData("Running to", " %7d :%7d", newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget);
            telemetry.update();
            sleep(500);


            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    protected void mecanumDrive() {
        double y = gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * -1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

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
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive"); //port 0 on expansion hub
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive"); //port 1 on expansion hub
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor"); // port 2 on expansion hub
        winchMotor = hardwareMap.get(DcMotor.class, "winch_motor"); // port 3 on expansion hub
        clawLeftServo = hardwareMap.servo.get("servo1"); // port 1 on control hub
        clawRightServo = hardwareMap.servo.get("servo2"); // port 0 control hub
        wristServo = hardwareMap.servo.get("servo3"); // port 2 control hub

        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // setting direction for motors
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        winchMotor.setDirection(DcMotor.Direction.FORWARD);

        //Setting Encorders for motors
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        winchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    protected void winch() {
        double winchPower = gamepad2.right_stick_y;
        winchPower = Range.clip(winchPower, -1.0, 1.0);
        winchMotor.setPower(winchPower);
    }

    protected void ArmAndWrist() {

        double armPower = gamepad2.left_stick_y;
        armPower = Range.clip(armPower, -1.0, 1.0);
        armMotor.setPower(armPower);

        double wristAdjust = 0.02;
        double wristPos = armMotor.getCurrentPosition() * wristAdjust;
        wristServo.setPosition(wristPos);

        if (armMotor.getCurrentPosition() >= armPosMax) {
            armMotor.setPower(0);
        }
        if (armMotor.getCurrentPosition() >= armPosMax) {
            armMotor.setPower(0);
        }
        telemetry.addData("Motors", "arm Power(%.2f)", armPower);
        telemetry.addData("Motors", "arm Position(%.2f)", armPower);
        telemetry.addData("Servos", "wrist (%.2f)", wristPos);

    }

    protected void claw() {
        boolean leftBumper1 = gamepad1.left_bumper;
        boolean rightBumper1 = gamepad1.right_bumper;
        boolean leftBumper2 = gamepad2.left_bumper;
        boolean rightBumper2 = gamepad2.right_bumper;

        if (leftBumper1) {
            clawLeftServo.setPosition(clawLeftServoMin);
        }

        if (rightBumper1) {
            clawLeftServo.setPosition(clawLeftServoMax);
        }

        if (leftBumper2) {
            clawRightServo.setPosition(clawRightServoMin);
        }

        if (rightBumper2) {
            clawRightServo.setPosition(clawRightServoMax);
        }
    }

    protected void calibrationAndTelemetry() {

        double servoDelta = 0.005;

        double armPosit = armMotor.getCurrentPosition(); //gamepad1.left_stick_y
        double armPow = gamepad1.left_stick_y;
        armPow = Range.clip(armPow, -1.0, 1.0);
        armMotor.setPower(armPow);

        double winchPosit = winchMotor.getCurrentPosition(); //gamepad2.right_stick_y
        double winchPow = gamepad2.right_stick_y;
        winchPow = Range.clip(winchPow, -1.0, 1.0);
        armMotor.setPower(winchPow);

        double leftClawPosit = clawLeftServo.getPosition(); //gamepad1.left_stick_x
        leftClawPosit = leftClawPosit + (gamepad1.left_stick_x*servoDelta);
        clawLeftServo.setPosition(leftClawPosit);

        double rightClawPosit = clawRightServo.getPosition(); //gamepad1.right_stick_x
        rightClawPosit = rightClawPosit + (gamepad1.right_stick_x*servoDelta);
        clawRightServo.setPosition(rightClawPosit);

        double wristPosit = wristServo.getPosition(); //gamepad1.dpad_down
        if(gamepad1.dpad_down){
            wristPosit -= servoDelta;
        }
        else if(gamepad1.dpad_up){
            wristPosit += servoDelta;
        }
        wristServo.setPosition(wristPosit);

        telemetry.addData("ArmPos: ", armPosit);
        telemetry.addData("WinchPos: ", winchPosit);
        telemetry.addData("LClawPos: ", leftClawPosit);
        telemetry.addData("RClawPos: ", rightClawPosit);
        telemetry.addData("WristPos: ", wristPosit);
    }
}

