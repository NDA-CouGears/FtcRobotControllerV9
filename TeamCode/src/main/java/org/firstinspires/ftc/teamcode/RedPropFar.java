package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "Red Team Prop Far 1", group = "Team OpModes")
public class RedPropFar extends AutoMode {
    protected TfodProcessor getProcessor(){
        String[] labels = {"Blue Prop"};
        TfodProcessor newProcessor = new TfodProcessor.Builder()
                .setModelAssetName("newRedModel.tflite")
                .setModelLabels(labels)
                .setIsModelQuantized(true)
                .build();
        return newProcessor;
    }

    @Override
    protected boolean isBlue(){
        return false;
    }

    @Override
    protected boolean isNear() {
        return false;
    }

}

