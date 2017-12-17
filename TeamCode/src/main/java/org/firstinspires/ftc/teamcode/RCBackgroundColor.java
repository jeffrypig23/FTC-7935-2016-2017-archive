package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Random;

/**
 * Created by Stephen Ogden on 1/25/2017.
 */

// This program should change the background to red when you press INIT, then to blue one opmode has started

@Autonomous(name="Background Color test", group ="Test")
@Disabled



public class RCBackgroundColor extends LinearOpMode {

    @Override

    public void runOpMode() throws InterruptedException {

        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

        telemetry.addData("Status", "Initialized! Please press play to start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && (gamepad1.x == false)) {


                relativeLayout.post(new Runnable() {
                    public void run() {
                        relativeLayout.setBackgroundColor(Color.rgb(0, 0, 255));
                    }
                });
            telemetry.addData("Color", "Blue");
            telemetry.update();
        }
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.rgb(255, 0, 0));
            }
        });
        telemetry.addData("Color", "Red");
        telemetry.update();
    }
}
