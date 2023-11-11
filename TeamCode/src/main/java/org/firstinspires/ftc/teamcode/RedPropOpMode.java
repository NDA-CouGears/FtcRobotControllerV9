package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "Red Team Prop", group = "Team OpModes")
public class RedPropOpMode extends TensorFlowObjectDetection {
    protected TfodProcessor getProcessor(){
        String[] labels = {"Red Prop"};
        TfodProcessor newProcessor = new TfodProcessor.Builder()
                .setModelAssetName("redtrainingfinal.tflite")
                .setModelLabels(labels)
                .setIsModelQuantized(true)
                .build();
        return newProcessor;
    }

}
