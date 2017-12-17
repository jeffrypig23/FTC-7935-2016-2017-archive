/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CloseableFrame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * This OpMode is an example which should display the X Y and Z offsets from the target
 */

@Autonomous(name="Vuforia Beacon Sensing Test", group ="Test")
@Disabled
public class VuforiaBeaconSensingTest extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {

		telemetry.addData(">", "Loading Vuforia...");
        telemetry.update();

        //<editor-fold desc="Vision stuffs">
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
		params.vuforiaLicenseKey = "AWbzKHD/////AAAAGbGEq1ADgkVssDlzbJne3JxpbC88knTtSCe2QqAfzYWx8tyZlkO1jwQT7T2HMKxvxFXuNVlr4lgmOlx8xOFE8+4tMHH+4fHj/r4vtqwel1noNNRWvMR4H0SIJfhpiyAVrweptR90dULCME5CZB2kOWmnl/cnT+9vf0rw21wF9ydAnP4UDxp1PFkzIz/6C3FG+xVTtD3RfsM1NcRnu/qgKDSXqtOv+Dy5MIAKLV2nramvFI/PiL1O3yWNaVYpSQiMmZcE7wn8GNWmJA5MDf1QZItNcIfz0MqX/8X8mqqTiSYdQ34O4Cwj1OjYDeOyqOlVwD9nfvzX1uTMk7w8Hzh270GphNhEULGF+ilLzXNwLSkV";
		params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
		VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
		Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
		vuforia.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time
		VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
		//makes object VuforiaTrackables called beacons but it comes from the asset FTC_2016-17.xml
		VuforiaTrackableDefaultListener wheels = ((VuforiaTrackableDefaultListener) beacons.get(0).getListener());
		int rows = 2;
		int cols = 2;
		Image rgb = null;
		long numImages;
		int color = 0;
		int pixels[][][] = new int[rows][cols][3];
		int leftColor[] = new int[4]; //leftColor[3] will hold the number of values added up
		int rightColor[] = new int[4]; //similar to above
        //</editor-fold>

        String forwardorbackwards = "This check is never used :(";


		/** Wait for the game to begin */
        telemetry.clearAll();
		telemetry.addData(">", "Press Play to start tracking");
		telemetry.update();
		waitForStart();
		VectorF translation = null;
        float beacondesc = 0;
        float zdata;
		int count = 0;
		/** Start tracking the data sets we care about. */
		beacons.activate();
		while (opModeIsActive()) {
			OpenGLMatrix pose = wheels.getPose();

            //<editor-fold desc="Stephen's crappy addition">
            //*
            //zdata = translation.get(2);//translation.get(2) will always be negative
            //beacondesc = 300 + zdata;

            if (beacondesc > 10) {
                //Go backwards
                forwardorbackwards = "Backwards";
            } else if (beacondesc < -10) {
                // go forwards
                forwardorbackwards = "Forward";
            } else {
                forwardorbackwards = "STAHP!!";
            }
            //*/
            //</editor-fold>

			if (pose != null) {
				translation = pose.getTranslation(); //the xz plane is parallel to the ground
				telemetry.addData("X", translation.get(0));
				telemetry.addData("Y", translation.get(1));
				telemetry.addData("Z", translation.get(2));

                //<editor-fold desc="More of stephen's tom-foolery">
                //*
                zdata = translation.get(2);
                beacondesc = 300 + zdata;
                telemetry.addData(" ", " ");
				telemetry.addData("Drive forward/backwards", forwardorbackwards);
                //HOLY $HIT IT WORKED!!!! ETHAN IM A GENIUS !
                //*/
                //</editor-fold>

                telemetry.addData(" ", " ");
				Orientation angle = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
				telemetry.addData("Angles", angle);
				telemetry.addData("angle", 180 - Math.abs(angle.secondAngle));
			} else {
                telemetry.addData("Status", "Image not found x _ x");
            }
            telemetry.update();
		}
	}
	public static int red(int color) {
		return (color >> 16) & 0xFF;
	}

	public static int green(int color) {
		return (color >> 8) & 0xFF;
	}

	public static int blue(int color) {
		return color & 0xFF;
	}
}
