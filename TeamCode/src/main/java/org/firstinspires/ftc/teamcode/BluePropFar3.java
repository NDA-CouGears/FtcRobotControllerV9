package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "Blue Team Prop Far3", group = "Team OpModes")
public class BluePropFar3 extends AutoMode {
    protected TfodProcessor getProcessor(){
        testlocation = 3;
        String[] labels = {"Blue Prop"};
        TfodProcessor newProcessor = new TfodProcessor.Builder()
                .setModelAssetName("bluetrainingfinal.tflite")
                .setModelLabels(labels)
                .setIsModelQuantized(true)
                .build();
        return newProcessor;
    }

    @Override
    protected boolean isBlue(){
        return true;
    }

    @Override
    protected boolean isNear() {
        return false;
    }
}

