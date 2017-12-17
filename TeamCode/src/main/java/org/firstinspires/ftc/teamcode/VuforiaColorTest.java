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
import android.graphics.Color;
import android.util.Log;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CloseableFrame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * This OpMode is an example which should display the X Y and Z offsets from the target
 */

@Autonomous(name="Vuforia RGB & HSV Test", group ="Test")
@Disabled
public class VuforiaColorTest extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {

		VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
		params.vuforiaLicenseKey = "AWbzKHD/////AAAAGbGEq1ADgkVssDlzbJne3JxpbC88knTtSCe2QqAfzYWx8tyZlkO1jwQT7T2HMKxvxFXuNVlr4lgmOlx8xOFE8+4tMHH+4fHj/r4vtqwel1noNNRWvMR4H0SIJfhpiyAVrweptR90dULCME5CZB2kOWmnl/cnT+9vf0rw21wF9ydAnP4UDxp1PFkzIz/6C3FG+xVTtD3RfsM1NcRnu/qgKDSXqtOv+Dy5MIAKLV2nramvFI/PiL1O3yWNaVYpSQiMmZcE7wn8GNWmJA5MDf1QZItNcIfz0MqX/8X8mqqTiSYdQ34O4Cwj1OjYDeOyqOlVwD9nfvzX1uTMk7w8Hzh270GphNhEULGF+ilLzXNwLSkV";
		params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
		VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
		Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
		vuforia.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time
		VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
		//makes object VuforiaTrackables called beacons but it comes from the asset FTC_2016-17.xml
		VuforiaTrackableDefaultListener wheels = ((VuforiaTrackableDefaultListener) beacons.get(0).getListener());
		Image rgb = null;
		long numImages;
		int color = 0;
		int pixel[] = new int[3];
		float pixelHSV[] = new float[3];
		Bitmap bm;
		int x = 600;
		int y = 600;



		/** Wait for the game to begin */
		telemetry.addData(">", "Press Play to start tracking");
		telemetry.update();
		waitForStart();
		VectorF translation = null;
		int count = 0;
		/** Start tracking the data sets we care about. */
		beacons.activate();
		while (opModeIsActive()) {
			OpenGLMatrix pose = wheels.getPose();
			if (pose != null) {
				translation = pose.getTranslation(); //the xz plane is parrallel to the ground
				telemetry.addData("X", translation.get(0));
				telemetry.addData("Y", translation.get(1));
				telemetry.addData("Z", translation.get(2));

				Orientation angle = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
				telemetry.addData("Angles", angle);
				telemetry.addData("angle", 180 - Math.abs(angle.secondAngle));
			}
			CloseableFrame frame = vuforia.getFrameQueue().take(); //takes the frame at the head of the queue
			numImages = frame.getNumImages();
			for (int i = 0; i < numImages; i++) {
				if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
					rgb = frame.getImage(i);
					break;
				}
			}
			if (rgb != null && rgb.getPixels() != null) {
				count++;
				bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
				bm.copyPixelsFromBuffer(rgb.getPixels());
				telemetry.addData("Width", rgb.getWidth());
				telemetry.addData("Height", rgb.getWidth());
				color = bm.getPixel(x, y)+16777216; //if we want 3 non edge columns, we must divide into fourths, etc
				pixel[0] = Color.red(color);
				pixel[1] = Color.green(color);
				pixel[2] = Color.blue(color);
				telemetry.addData("", "X: %d Y: %d", x, y);
				//telemetry.addData("COLOR", String.format("0x%06X", color));
				telemetry.addData("","R%d G%d B%d", pixel[0], pixel[1], pixel[2]);
				Color.RGBToHSV(pixel[0], pixel[1], pixel[2], pixelHSV);
				telemetry.addData("Count", count);
				frame.close();
			}
			else {
				telemetry.addData("Error", "Not good");
			}
			x = (int)(x + gamepad1.right_stick_x * 3);
			y = (int)(y + gamepad1.right_stick_y * 3);
			telemetry.update();
		}
		telemetry.addData("OpMode No Longer Active", " ");
		telemetry.update();
	}
}
