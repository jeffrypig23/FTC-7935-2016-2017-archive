package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.os.Environment;
import android.util.Log;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.CameraCalibration;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Arrays;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.mmPerInch;

/**
 * Created by Stephen Ogden on 1/23/2017.
 */

//Fingers should be the back of the robot

@Autonomous(name = "Buggy Red Beacon Auto1", group = "Test")
@Disabled

public class RedVuforiaBeaconAuto1 extends LinearOpMode {

    //<editor-fold desc="Initialization">
    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftBackMotor = null;
    DcMotor rightBackMotor = null;
    DcMotor shooterMotor = null;
    DcMotor intakeMotor = null; //tetrix
    Servo upServo = null;

    final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

    final double wheelDiameter = 4; //wheel diameter in inches
    final double ticksPerRotation = 1120; //it has 1440 ticks because its tetrix

    double servoUpPos = 0.5;
    double servoDownPos = 0.1;
    //double robotPos = 0;

    float zdata;
    float beacondesc = 0;

    int average;
    //int shooterPos = 1600;
    //int shooterUpPos = 1000;
    int stage = 3;
    int errorstage = -2;
    int count = 0;

    String ErrorMessage = null;

    final String TAG = "ROBOT";
    //</editor-fold>

    @Override

    public void runOpMode() throws InterruptedException {

        //<editor-fold desc="Vuforia Setup">
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);

        params.vuforiaLicenseKey = "AWbzKHD/////AAAAGbGEq1ADgkVssDlzbJne3JxpbC88knTtSCe2QqAfzYWx8tyZlkO1jwQT7T2HMKxvxFXuNVlr4lgmOlx8xOFE8+4tMHH+4fHj/r4vtqwel1noNNRWvMR4H0SIJfhpiyAVrweptR90dULCME5CZB2kOWmnl/cnT+9vf0rw21wF9ydAnP4UDxp1PFkzIz/6C3FG+xVTtD3RfsM1NcRnu/qgKDSXqtOv+Dy5MIAKLV2nramvFI/PiL1O3yWNaVYpSQiMmZcE7wn8GNWmJA5MDf1QZItNcIfz0MqX/8X8mqqTiSYdQ34O4Cwj1OjYDeOyqOlVwD9nfvzX1uTMk7w8Hzh270GphNhEULGF+ilLzXNwLSkV";
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time
        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        //makes object VuforiaTrackables called beacons but it comes from the asset FTC_2016-17.xml
        VuforiaTrackableDefaultListener wheels = ((VuforiaTrackableDefaultListener) beacons.get(0).getListener());

        Image rgb = null; //Image is an vuforia object, but it can be converted to a bitmap (simple array sort of thing) or Mat (used for OpenCV)

        long numImages;

        boolean openCVLoaded = OpenCVLoader.initDebug(); // initializes openCV

        if (openCVLoaded == true) {
            Log.d(TAG, "OpenCV Loaded");
        } else {
            Log.d(TAG, "OpenCV NOT Loaded");
        }
        //</editor-fold>

        //<editor-fold desc="Motor declaration and behavior">
        leftFrontMotor = hardwareMap.dcMotor.get("LFMotor");
        rightFrontMotor = hardwareMap.dcMotor.get("RFMotor");
        leftBackMotor = hardwareMap.dcMotor.get("LBMotor");
        rightBackMotor = hardwareMap.dcMotor.get("RBMotor");
        shooterMotor = hardwareMap.dcMotor.get("SMotor");
        intakeMotor = hardwareMap.dcMotor.get("IMotor");
        upServo = hardwareMap.servo.get("UServo");
        //buttonServo = hardwareMap.servo.get("BServo");

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //</editor-fold>

        int color;
        //ElapsedTime runtime = new ElapsedTime();

        // Wait for the game to begin
        telemetry.addData("Status", "Initialized! Please press play to start.");
        telemetry.update();

        waitForStart();

        VectorF translation = null;

        // Start tracking the data sets we care about.
        beacons.activate();

        while (opModeIsActive()) {

            OpenGLMatrix pose = wheels.getPose(); // have this to find the image

            telemetry.addData("Stage", stage);
            telemetry.addData(" ", " ");

            // Change the RC background to red!
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.rgb(255, 0, 0));
                }
            });

            //<editor-fold desc="Error and test stages">

            if (ErrorMessage != null) {
                telemetry.addData("Error:", ErrorMessage);
                telemetry.addData("Error on stage", errorstage);
                telemetry.addData(" ", " ");
            }

            if (stage == -2) {

                telemetry.clearAll();
                telemetry.addData("Status", "Program Ended due to error");
                telemetry.addData("Solution", "Try yelling at Stephen");

                leftBackMotor.setPower(0.0);
                leftFrontMotor.setPower(0.0);
                rightBackMotor.setPower(0.0);
                rightFrontMotor.setPower(0.0);

                resetDriveMotors(0);

            } else if (stage == -1 || stage <= -3) {

                telemetry.addData("Status", "Program Ended for testing purposes");

            }
            //</editor-fold>

            if (stage == 0) {

                //<editor-fold desc="Step 0: Reset drive motors">
                //(Get it because programmers count from zero haha...im lonely :(    )
                telemetry.addData("Status", "Resetting drive motors");

                resetDriveMotors(0);

                stage = 1;
                errorstage = 0;
                //</editor-fold>

            } else if (stage == 1) {

                //<editor-fold desc="Step 1: Drive forward, to get in front of the first beacon (77 inches)">
                driveDistance(leftFrontMotor, 0.5, 77 * mmPerInch);
                driveDistance(leftBackMotor, 0.5, 77 * mmPerInch);
                driveDistance(rightFrontMotor, 0.5, 77 * mmPerInch);
                driveDistance(rightBackMotor, 0.5, 77 * mmPerInch);

                telemetry.addData("Status", "Driving towards the beacon...");

                if (isRightPos(leftFrontMotor, 100) && isRightPos(rightFrontMotor, 100) && isRightPos(leftBackMotor, 100) && isRightPos(rightBackMotor, 100)) {

                    stage = 2;
                    errorstage = 1;

                    leftFrontMotor.setPower(0.0);
                    leftBackMotor.setPower(0.0);
                    rightFrontMotor.setPower(0.0);
                    rightBackMotor.setPower(0.0);

                    resetDriveMotors(0);
                }
                //</editor-fold>

            } else if (stage == 2) {

                //<editor-fold desc="Step 1 continued: Turn towards beacon">
                driveDistance(rightFrontMotor, 0.5, .35 * mmPerInch);
                driveDistance(rightBackMotor, 0.5, .35 * mmPerInch);
                leftBackMotor.setPower(0.0);
                leftFrontMotor.setPower(0.0);

                telemetry.addData("Status", "Turning to face beacon...");

                if (isRightPos(rightBackMotor, 100) && isRightPos(rightFrontMotor, 100)) {

                    stage = 3;
                    errorstage = 2;

                    leftFrontMotor.setPower(0.0);
                    leftBackMotor.setPower(0.0);
                    rightFrontMotor.setPower(0.0);
                    rightBackMotor.setPower(0.0);

                    resetDriveMotors(1);
                }
                //</editor-fold>

            } else if (stage == 3) {

                //<editor-fold desc="Step 2: Get 300 mm within the beacon (In front of)">
                /*
                 * X: sides from beacon ()
                 * Y: up/down from beacon ()
                 * Z: forward/back from beacon (300)
                 */

                if (pose != null) {

                    translation = pose.getTranslation(); //the xz plane is parallel to the ground

                    zdata = translation.get(2);//translation.get(2) will always be negative
                    beacondesc = 300 + zdata;

                    driveDistance(leftFrontMotor, 0.25, -beacondesc);
                    driveDistance(leftBackMotor, 0.25, -beacondesc);
                    driveDistance(rightFrontMotor, 0.25, -beacondesc);
                    driveDistance(rightBackMotor, 0.25, -beacondesc);
                    if (isRightPos(rightBackMotor, 25) & isRightPos(leftBackMotor, 25) & isRightPos(rightFrontMotor, 25) & isRightPos(leftFrontMotor, 25)) {
                        telemetry.addData("Target position", "300");
                        telemetry.addData("Z data", translation.get(2));
                        telemetry.addData("Status", "Moving back...");
                        resetDriveMotors(0);
                        stage = 4;


                    }

                } else {

                    //Beacon or image or both not found
                    ErrorMessage = "Image not found x _ x";

                    stage = -2;
                    errorstage = 3;

                }
                //</editor-fold>

            } else if (stage == 4) {

                //<editor-fold desc="Step 3: Get frame as object rgb of type Image">
                VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take(); //takes the frame at the head of the queue
                numImages = frame.getNumImages();

                for (int i = 0; i < numImages; i++) { //finds an image with correct format (some are gray scale)
                    if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                        rgb = frame.getImage(i);
                        break;
                    }
                }

                for (int q = 0; q < 5; q++) {

                    color = beaconColor(rgb, wheels, vuforia.getCameraCalibration(), false);
                    //negative means blue-red, positive means red-blue, zero means error

                    average = (average + color) / q;

                    if (color == 1) {

                        telemetry.addData("Beacon Config", "Red | Blue");

                    } else if (color == -1) {

                        telemetry.addData("Beacon Config", "Blue | Red");

                    } else {

                        telemetry.addData("Beacon Config", "ERROR");

                    }
                }

                if (average == 1) {

                    telemetry.addData("Beacon Config", "Red | Blue");

                    stage = 5;

                } else if (average == -1) {

                    telemetry.addData("Beacon Config", "Blue | Red");

                    stage = 6;

                } else {

                    ErrorMessage = "Cannot determine beacon colors x _ x";

                    stage = -2;
                    errorstage = 4;

                }
                //</editor-fold>

            } else if (stage == 5) {

                // Based on beacon config (Red | Blue) Program is red btw (Drive to the left)

                // stage = 7;


            } else if (stage == 6) {

                // Based on beacon config (Blue | Red) Program is red btw (Drive to the right)

                // stage = 7;

            } else if (stage == 7) {

                //<editor-fold desc="Backup">
                driveDistance(leftFrontMotor, 0.5, 300);
                driveDistance(leftBackMotor, 0.5, 300);
                driveDistance(rightFrontMotor, 0.5, 300);
                driveDistance(rightBackMotor, 0.5, 300);

                telemetry.addData("Status", "Backing up...");

                if (isRightPos(leftFrontMotor, 100) && isRightPos(rightFrontMotor, 100) && isRightPos(leftBackMotor, 100) && isRightPos(rightBackMotor, 100)) {

                    stage = 2;

                    leftFrontMotor.setPower(0.0);
                    leftBackMotor.setPower(0.0);
                    rightFrontMotor.setPower(0.0);
                    rightBackMotor.setPower(0.0);

                    resetDriveMotors(0);
                }
                //</editor-fold>

                stage = 100;
                errorstage = 7;
            } else if (stage == 100) {
                //Say you're done!
                telemetry.addData("Status", "Done!");
                telemetry.update();
            }

            telemetry.update();

        }
    }

    public int beaconColor(Image img, VuforiaTrackableDefaultListener target, CameraCalibration calib, boolean saveImage) {

        int color;

        OpenGLMatrix pose = target.getRawPose();

        if (img != null && img.getPixels() != null && pose != null) {

            //step 1: Change rgb to to Bitmap bm (must have same width and height)
            Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
            bm = bm.copy(Bitmap.Config.RGB_565, true); //copies the bitmap such that it goes from an immutable to mutable form
            bm.copyPixelsFromBuffer(img.getPixels()); //at this point we have converted from a CloseableFrame to Image to Bitmap

            //step 2: Copy bm into Mat crop (must have same width and height) so that we can apply openCV functions to it.
            Mat crop = new Mat(bm.getHeight(), bm.getWidth(), CvType.CV_8UC3); //here we have width before height because the mat is specified as rows x cols
            Utils.bitmapToMat(bm, crop);

            //step 3: Find corners[] of the beacon by projecting certain points onto a reference frame which is an object called rawPose of type Matrix34f
            Matrix34F rawPose = new Matrix34F();
            rawPose.setData(Arrays.copyOfRange(pose.transposed().getData(), 0, 12));

            float corners[][] = new float[4][2];
            corners[0] = Tool.projectPoint(calib, rawPose, new Vec3F(-127, 276, 0)).getData(); //upper left of beacon
            corners[1] = Tool.projectPoint(calib, rawPose, new Vec3F(127, 276, 0)).getData(); //upper right of beacon
            corners[2] = Tool.projectPoint(calib, rawPose, new Vec3F(127, 92, 0)).getData(); //lower right of beacon
            corners[3] = Tool.projectPoint(calib, rawPose, new Vec3F(-127, 92, 0)).getData();

            //step 4: Create a Rect roi based on the corners[] and rows and cols of crop for later cropping and make new Mat cropped based on roi and crop
            float x = Math.min(Math.min(corners[1][0], corners[3][0]), Math.min(corners[0][0], corners[2][0])); //x is corner with smallest x value
            float y = Math.min(Math.min(corners[1][1], corners[3][1]), Math.min(corners[0][1], corners[2][1])); // y is corner with smallest y value
            float width = Math.max(Math.abs(corners[0][0] - corners[2][0]), Math.abs(corners[1][0] - corners[3][0]));
            float height = Math.max(Math.abs(corners[0][1] - corners[2][1]), Math.abs(corners[1][1] - corners[3][1]));

            //make sure our bounding box doesn't go outside of the image
            //OpenCV doesn't like that... :(
            x = Math.max(x, 0);
            y = Math.max(y, 0);
            width = (x + width > crop.cols()) ? crop.cols() - x : width;
            height = (y + height > crop.rows()) ? crop.rows() - y : height;
            Rect roi = new Rect((int) x, (int) y, (int) width, (int) height);
            Mat cropped = new Mat(crop, roi);

            //step 5: Rotate cropped into rotated so that it is aligned correctly
            /*if phone is upside down, image needs to be flipped 90 degrees CCW. Otherwise, it needs to be flipped 90 CW
                 to do this we will first transpose the matrix, then flip it over the y axis. It will go from
				 1 2 3         1 4         4 1
				 4 5 6   to    2 5   to    5 2
				               3 6         6 3
			*/
            Mat rotated = new Mat(cropped.width(), cropped.height(), CvType.CV_8UC3); //creates mat with transposed dimensions of cropped
            Core.transpose(cropped, rotated);
            Core.flip(rotated, rotated, 0); //positive flag flips along y axis. If we set flag to 0, it would be flipped along x thus giving a CCW rotation

            //step 6: convert to hsv from rgb
            Imgproc.cvtColor(rotated, rotated, Imgproc.COLOR_RGB2HSV_FULL);

            //step 7: filter the colors to get a binary image with blue corresponding to white and anything else corresponding to black
            Mat mask = new Mat();
            Core.inRange(rotated, new Scalar(108, 0, 220), new Scalar(178, 255, 255), mask); //put the newly created mask into Mat named mask

            //step 8: find centroid of image and process this to find answer
            Moments m = Imgproc.moments(mask, true);
            Point centroid = new Point(m.get_m10() / m.get_m00(), m.get_m01() / m.get_m00()); //look at http://docs.opencv.org/2.4/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html

            String beaconConfig;

            if (centroid.x < mask.cols() / 2) {

                beaconConfig = "Blue Red";
                color = -1;

            } else {

                beaconConfig = "Red Blue";
                color = 1;

            }

            Log.i(TAG, beaconConfig);

            if (saveImage) { //draw on and save image when asked

                bm = Bitmap.createBitmap(mask.width(), mask.height(), Bitmap.Config.RGB_565);
                Utils.matToBitmap(mask, bm);

                Canvas canvas = new Canvas(bm); //creates a canvas over the bitmap so that it can be painted upon

                Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG);
                paint.setColor(Color.rgb(255, 255, 0)); //sets the paint color as yellow
                paint.setTextSize(30);

                canvas.drawText("X:" + String.valueOf((int) centroid.x) + " Y:" + String.valueOf((int) centroid.y), bm.getWidth() / 2, bm.getHeight() * 2 / 3, paint); //labels x and y of centroid in image
                canvas.drawText(beaconConfig, bm.getWidth() / 2, bm.getHeight() / 3, paint);

                paint.setColor(Color.rgb(0, 0, 255)); //sets the paint color as blue

                canvas.drawCircle((float) centroid.x, (float) centroid.y, 20, paint); //draws circle at centroid

                String path = Environment.getExternalStorageDirectory().toString();

                //Log.i(TAG, path);
                File file = new File(path, "VuforiaFrame" + count / 100 + ".png"); // the File to save , append increasing numeric counter to prevent files from getting overwritten.

                count++;

                FileOutputStream out = null;

                try { //copy and pasted from stack exchange (mostly)

                    out = new FileOutputStream(file);
                    bm.compress(Bitmap.CompressFormat.JPEG, 70, out); //actual bit that saves the file

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

            return color;

        } else {

            return 0;

        }
    }

    public void driveDistance(DcMotor motor, double power, double distance) { //drives forward distance in inches
        int pos = (int) Math.round(((distance * mmPerInch) / (wheelDiameter * mmPerInch * Math.PI)) * ticksPerRotation);
        motor.setTargetPosition(pos);
        motor.setPower(power);
    }

    public void resetDriveMotors(int mode) {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (mode == 0) {
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else if (mode == 1) {
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public boolean isRightPos(DcMotor motor, int tolerance) {
        if (Math.abs(motor.getTargetPosition() - motor.getCurrentPosition()) < tolerance) {
            return true;
        } else {
            return false;
        }
    }

}
