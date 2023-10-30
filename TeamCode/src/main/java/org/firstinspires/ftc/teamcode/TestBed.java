/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 *
 */

@TeleOp(name="Test Bed", group="Testing")
public class TestBed extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor arm = null;
    Servo wristServo;
    Servo   leftServo;
    Servo   rightServo;
    TouchSensor touchSensor;
    DistanceSensor sensorDistance;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        arm  = hardwareMap.get(DcMotor.class, "arm_motor");
        wristServo = hardwareMap.get(Servo.class, "wrist servo");
        leftServo = hardwareMap.get(Servo.class, "left servo");
        rightServo = hardwareMap.get(Servo.class, "right servo");
        touchSensor = hardwareMap.get(TouchSensor.class, "arm zero");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "color");

        arm.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        double servoDelta = 0.005;

        while (opModeIsActive()) {
            double wristPos = wristServo.getPosition();
            if (gamepad1.dpad_down) {
                wristPos -= servoDelta;
            }
            else if (gamepad1.dpad_up) {
                wristPos += servoDelta;
            }
            wristServo.setPosition(wristPos);

            double leftServoPos = leftServo.getPosition();
            leftServoPos = leftServoPos + (gamepad1.left_stick_x*servoDelta);
            leftServo.setPosition(leftServoPos);

            double rightServoPos = rightServo.getPosition();
            rightServoPos = rightServoPos + (gamepad1.right_stick_x*servoDelta);
            rightServo.setPosition(rightServoPos);

            double armPower = gamepad1.left_stick_y;

            armPower   = Range.clip(armPower, -1.0, 1.0) ;

            arm.setPower(armPower);

            boolean touched = touchSensor.isPressed();
            double distance = sensorDistance.getDistance(DistanceUnit.CM);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "arm (%.2f)", armPower);
            telemetry.addData("Servos", "wrist (%.2f); left (%.2f); right (%.2f)", wristPos, leftServoPos, rightServoPos);
            telemetry.addData("Sensor", "touch (%b); range cm (%.2f)", touched, distance);
            telemetry.update();
        }
    }
}
