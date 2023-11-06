/*
    author: delia jasper
    purpose: another way to drive using mecanum wheels
 */

// i want to try this out soon and see if a it works and if b if the drivers like robot or field centric better

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

@TeleOp(name="Drive and Lift2", group="TeleOp")
abstract public class robotParent extends LinearOpMode {

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
//    static final double servoMin = 31;
//    static final double servo2Min = 63;

    public void driveEncorder(int driveSpeed, int leftFrontInches, int rightFrontInches, int leftBackInches, int rightBackInches, double timeOut){
        double curArmPosition = 31;
        double curArmPosition2 = 63;
    }

    public void mecanumDrive(){
        boolean leftBumper = gamepad2.left_bumper;
        boolean rightBumper = gamepad2.right_bumper;
    }

    public void initHardware() {

        // Initialize the drive system variables.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        winchMotor = hardwareMap.get(DcMotor.class, "winch_motor");
        clawLeftServo = hardwareMap.servo.get("servo1");
        clawRightServo = hardwareMap.servo.get("servo2");
        wristServo = hardwareMap.servo.get("servo3");

        // setting direction for motors
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
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

        waitForStart();

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * -1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            double deltaY = -gamepad2.left_stick_y;
            double deltaY2 = -gamepad2.right_stick_y;

            if (gamepad2.b)
            {
                curArmPosition = middleServo;
                curArmPosition2 = middleServo2;
            }

            if (gamepad2.y)
            {
                curArmPosition = topServo;
                curArmPosition2 = topServo2;
            }

            if (gamepad2.a)
            {
                curArmPosition = servoMin;
                curArmPosition2 = servo2Min;
                claw.setPosition(1);
            }

            if (gamepad2.x)
            {
                curArmPosition -= 7.5;
                curArmPosition2 -= 7.5;
                sleep(1000);
                claw.setPosition(1);
            }

            if(gamepad2.dpad_down)
            {
                for(int clap = 0; clap < 5; clap++) {
                    claw.setPosition(1);
                    sleep(500);
                    claw.setPosition(0);
                    sleep(500);
                }

            }

            curArmPosition2 += deltaY2*0.1;
            curArmPosition += deltaY*0.1;

            if (curArmPosition > servoMax) {
                curArmPosition = servoMax;
            }
            else if (curArmPosition < servoMin ) {
                curArmPosition = servoMin;
            }

            if (curArmPosition2 > servo2Max) {
                curArmPosition2 = servo2Max;
            }
            else if (curArmPosition2 < servo2Min) {
                curArmPosition2 = servo2Min;
            }

            servo.setPosition(curArmPosition/100);
            servo2.setPosition(curArmPosition2/100);

            if (gamepad2.left_bumper) {
                claw.setPosition(1);
            }

            if (gamepad2.right_bumper) {
                claw.setPosition(0);
            }

            // Wait for the game to start (driver presses PLAY)
            telemetry.addData("Current Position: ", curArmPosition);
            telemetry.addData("Current Position: ", curArmPosition2);
            telemetry.update();

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

    }
}
