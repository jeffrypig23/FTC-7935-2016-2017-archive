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
import android.graphics.Bitmap.CompressFormat;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.os.Environment;
import android.util.Log;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.CameraCalibration;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec2F;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
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
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.util.Arrays;

import static android.R.attr.logo;
import static android.R.attr.path;
import static android.R.id.mask;


/**
 * This OpMode is an example which should display the X Y and Z offsets from the target
 */

@Autonomous(name="Vuforia OpenCV Testing", group ="Test")
@Disabled
public class VuforiaOpenCVTest extends LinearOpMode {
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
		int rows = 2;
		int cols = 2;
		Image rgb = null; //Image is an vuforia object, but it can be converted to a bitmap (simple array sort of thing) or Mat (used for OpenCV)
		long numImages;
		int color = 0;
		int pixels[][][] = new int[rows][cols][3];
		int leftColor[] = new int[4]; //leftColor[3] will hold the number of values added up
		int rightColor[] = new int[4]; //similar to above
		final String TAG = "ROBOT";
		boolean openCVLoaded = OpenCVLoader.initDebug(); // initializes openCV
		if (openCVLoaded == true) {
			Log.d(TAG, "OpenCV Loaded");
		}
		else {
			Log.d(TAG, "OpenCV NOT Loaded");
		}



		/** Wait for the game to begin */
		telemetry.addData(">", "Press Play to start tracking");
		telemetry.update();
		waitForStart();
		VectorF translation = null;
		int count = 0;
		/** Start tracking the data sets we care about. */
		beacons.activate();
		while (opModeIsActive()) {
			//step 1
			//Get frame as object rgb of type Image
			CloseableFrame frame = vuforia.getFrameQueue().take(); //takes the frame at the head of the queue
			numImages = frame.getNumImages();
			for (int i = 0; i < numImages; i++) { //finds an image with correct format (some are gray scale)
				if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
					rgb = frame.getImage(i);
					break;
				}
			}
			OpenGLMatrix pose = wheels.getRawPose();
			if (rgb != null && rgb.getPixels() != null && pose != null) {
				//step 2
				//Change rgb to to Bitmap bm (must have same width and height)
				Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
				bm = bm.copy(Bitmap.Config.RGB_565, true); //copies the bitmap such that it goes from an immutable to mutable form
				bm.copyPixelsFromBuffer(rgb.getPixels()); //at this point we have converted from a CloseableFrame to Image to Bitmap
				//step 3
				//Copy bm into Mat crop (must have same width and height) so that we can apply openCV functions to it.
				Mat crop = new Mat(bm.getHeight(), bm.getWidth(), CvType.CV_8UC3); //here we have width before height because the mat is specified as rows x cols
				Utils.bitmapToMat(bm, crop);
				//step 4
				//Find corners[] of the beacon by projecting certain points onto a reference frame which is an object called rawPose of type Matrix34f
				Matrix34F rawPose = new Matrix34F();
				rawPose.setData(Arrays.copyOfRange(pose.transposed().getData(), 0, 12));
				float corners[][] = new float[4][2];
				corners[0] = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(-127, 276, 0)).getData(); //upper left of beacon
				corners[1] = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(127, 276, 0)).getData(); //upper right of beacon
				corners[2] = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(127, 92, 0)).getData(); //lower right of beacon
				corners[3] = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(-127, 92, 0)).getData();
				//step 5
				//Create a Rect roi based on the corners[] and rows and cols of crop for later cropping and make new Mat cropped based on roi and crop
				float x = Math.min(Math.min(corners[1][0], corners[3][0]), Math.min(corners[0][0], corners[2][0])); //x is corner with smallest x value
				float y = Math.min(Math.min(corners[1][1], corners[3][1]), Math.min(corners[0][1], corners[2][1])); // y is corner with smallest y value
				float width = Math.max(Math.abs(corners[0][0] - corners[2][0]), Math.abs(corners[1][0] - corners[3][0]));
				float height = Math.max(Math.abs(corners[0][1] - corners[2][1]), Math.abs(corners[1][1] - corners[3][1]));

				//make sure our bounding box doesn't go outside of the image
				//OpenCV doesn't like that...
				x = Math.max(x, 0);
				y = Math.max(y, 0);
				width = (x + width > crop.cols())? crop.cols() - x : width;
				height = (y + height > crop.rows())? crop.rows() - y : height;
				Rect roi = new Rect((int) x, (int) y, (int) width, (int) height);
				Mat cropped = new Mat(crop, roi);
				//step 6
				//Rotate cropped into rotated so that it is aligned correctly
				/*if phone is upside down, image needs to be flipped 90 degrees CCW. Otherwise, it needs to be flipped 90 CW
				 to do this we will first transpose the matrix, then flip it over the y axis. It will go from
				 1 2 3         1 4         4 1
				 4 5 6   to    2 5   to    5 2
				               3 6         6 3
				*/
				Mat rotated = new Mat(cropped.width(), cropped.height(), CvType.CV_8UC3); //creates mat with transposed dimensions of cropped
				Core.transpose(cropped, rotated);
				Core.flip(rotated, rotated, 0); //positive flag flips along y axis. If we set flag to 0, it would be flipped along x thus giving a CCW rotation

				//step 7
				//convert to hsv from rgb
				Imgproc.cvtColor(rotated, rotated, Imgproc.COLOR_RGB2HSV_FULL);
				//step 8
				//filter the colors to get a binary image with blue corresponding to white and anything else corresponding to black
				Mat mask = new Mat();
				Core.inRange(rotated, new Scalar(108, 0, 220), new Scalar(178, 255, 255), mask);
				//step 9
				//find centroid of image and process this to find answer
				Moments m = Imgproc.moments(mask, true);
				Point centroid = new Point(m.get_m10() / m.get_m00(), m.get_m01() / m.get_m00()); //look at http://docs.opencv.org/2.4/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html
				String beaconConfig;
				/*if (m.get_m00() / mask.total() < 0.1) {
					beaconConfig = "No Blue";
				}
				else if (m.get_m00() / mask.total() > 0.6) {
					beaconConfig = "All Blue?";
				}
				else */if (centroid.x < mask.cols()/2) {
					beaconConfig = "Blue Red";
				}
				else {
					beaconConfig = "Red Blue";
				}
				telemetry.addData("Beacon Config", beaconConfig);



				if (count % 100 == 0) { //draw on and save image every 100 loops
					bm = Bitmap.createBitmap(mask.width(), mask.height(), Bitmap.Config.RGB_565);
					Utils.matToBitmap(mask, bm);

					Canvas canvas = new Canvas(bm); //creates a canvas over the bitmap so that it can be painted upon
					Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG);
					paint.setColor(Color.rgb(255, 255, 0)); //sets the paint color as yellow
					paint.setTextSize(30);
					canvas.drawText("X:" + String.valueOf((int) centroid.x) + " Y:" + String.valueOf((int) centroid.y), bm.getWidth()/2, bm.getHeight()*2/3, paint); //labels x and y of centroid in image
					canvas.drawText(beaconConfig, bm.getWidth()/2, bm.getHeight()/3, paint);
					paint.setColor(Color.rgb(0, 0, 255)); //sets the paint color as blue
					canvas.drawCircle((float) centroid.x, (float) centroid.y, 20, paint); //draws circle at centroid
					String path = Environment.getExternalStorageDirectory().toString();
					File file = new File(path, "VuforiaFrame" + count / 100 + ".png"); // the File to save , append increasing numeric counter to prevent files from getting overwritten.
					FileOutputStream out = null;
					try { //copy and pasted from stack exchange (mostly)
						out = new FileOutputStream(file);
						bm.compress(CompressFormat.JPEG, 70, out); //actual bit that saves the file
					} catch (Exception e) {
						e.printStackTrace();
					} finally {
						try {
							if (out != null) {
								out.close();
							}
						} catch (IOException e) {
							e.printStackTrace();
						}
					} //done copy and pasting
				}
				count++;
				telemetry.addData("Loops", count);
				frame.close();
			}
			else {
				telemetry.addData("Error", "Not Good");
			}
			telemetry.update();
		}
		telemetry.addData("OpMode No Longer Active", " ");
		telemetry.update();
	}
}
