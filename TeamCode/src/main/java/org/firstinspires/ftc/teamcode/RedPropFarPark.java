package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "Red Team Prop Far Park", group = "Team OpModes")
public class RedPropFarPark extends AutoMode {
    protected TfodProcessor getProcessor(){
        String[] labels = {"Blue Prop"};
        TfodProcessor newProcessor = new TfodProcessor.Builder()
                .setModelAssetName("newerRedModel.tflite")
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

    @Override
    protected boolean justPark() { return true; }

}

