package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "Blue Team Prop Far1", group = "Team OpModes")
public class BluePropFar1 extends AutoMode {
    protected TfodProcessor getProcessor(){
        testlocation = 1;
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

