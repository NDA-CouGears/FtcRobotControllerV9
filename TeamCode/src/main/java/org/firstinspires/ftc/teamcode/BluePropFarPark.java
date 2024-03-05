package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "Blue Team Prop Far Park", group = "Team OpModes")
public class BluePropFarPark extends AutoMode {
    protected TfodProcessor getProcessor(){
        String[] labels = {"Blue Prop"};
        TfodProcessor newProcessor = new TfodProcessor.Builder()
                .setModelAssetName("newerBlueModel.tflite")
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


    @Override
    protected boolean justPark() { return true; }

}

