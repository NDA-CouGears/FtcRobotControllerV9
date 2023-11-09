package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@TeleOp(name = "Blue Team Prop", group = "Concept")
public class BluePropOpMode extends TensorFlowObjectDetection {
    protected TfodProcessor getProcessor(){
        String[] labels = {"Blue Prop"};
        TfodProcessor newProcessor = new TfodProcessor.Builder()
                .setModelAssetName("bluetrainingfinal.tflite")
                .setModelLabels(labels)
                .setIsModelQuantized(true)
                .build();
        return newProcessor;
    }

}
