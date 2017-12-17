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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.mmPerInch;

/*
In autonomous what we want to do is have one robot go with the phone and shoot two balls into our goals
the other robot shoudl
 */


@Autonomous(name="Don't pick this", group="Linear Opmode")  // @Autonomous(...) is the other common choice
@Disabled
/**
 * Because this program is literally called "Don't pick this", I am disabling it
 * ~ Stephen
 */

public class ShootAndBallAuto extends LinearOpMode {

    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftBackMotor = null;
    DcMotor rightBackMotor = null;
    DcMotor shooterMotor = null;
    DcMotor intakeMotor = null; //tetrix
    Servo upServo = null;

	final double wheelDiameter = 4; //wheel diameter in inches
	final double ticksPerRotation = 1120; //it has 1440 ticks because its tetrix
	double servoUpPos = 0.3;
	double servoDownPos = 0.0;
	int shooterPos = 1600;
	int stage = 0;
	int shootingDistance = 36; //number of inches to drive before shooting
	int ballDistance = 48; //number of inches to drive to hit the ball. 60-18=42 would put us flush with vortex base. We go 6 inches past it.

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("INITIALIZATION", "BEGINNING");
        telemetry.addData("Config", "SMotor" + "IMotor" + "LFMotor" + "RFMotor" + "LBMotor" + "RBMotor" + "UServo");
        telemetry.update();

        leftFrontMotor = hardwareMap.dcMotor.get("RFMotor");
        rightFrontMotor = hardwareMap.dcMotor.get("LFMotor");
        leftBackMotor = hardwareMap.dcMotor.get("RBMotor");
        rightBackMotor = hardwareMap.dcMotor.get("LBMotor");
        shooterMotor = hardwareMap.dcMotor.get("SMotor");
        intakeMotor = hardwareMap.dcMotor.get("IMotor");
        upServo = hardwareMap.servo.get("UServo");

        leftFrontMotor.setMode(RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(RunMode.RUN_TO_POSITION);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontMotor.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        rightFrontMotor.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        leftBackMotor.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        rightBackMotor.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        shooterMotor.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        intakeMotor.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);

	    leftFrontMotor.setDirection(Direction.REVERSE);
	    rightFrontMotor.setDirection(Direction.REVERSE);
	    leftBackMotor.setDirection(Direction.FORWARD);
	    rightBackMotor.setDirection(Direction.FORWARD);

        ElapsedTime runtime = new ElapsedTime();
        telemetry.addData("INITIALIZATION", "DONE");
        telemetry.update();
        waitForStart();  //Waits for the match to start
        runtime.reset();
        while (opModeIsActive()) {
	        if (stage == -1) {
		        telemetry.addData("Status", "Program Ended for testing purposes");
	        }
	        else if (stage == 0) {
		        resetDriveMotors();
		        stage = 1;
	        }
	        else if (stage == 1) {
				telemetry.addData("Status", "Driving to shooting position");
		        driveDistance(leftFrontMotor, 1.0, shootingDistance);
		        driveDistance(rightFrontMotor, 1.0, shootingDistance);
		        driveDistance(leftBackMotor, 1.0, shootingDistance);
		        driveDistance(rightBackMotor, 1.0, shootingDistance);
		        if (isRightPos(leftFrontMotor, 100) && isRightPos(rightFrontMotor, 100) && isRightPos(leftBackMotor, 100) && isRightPos(rightBackMotor, 100)) {
			        stage = -1;
			        resetDriveMotors();
		        }
	        }
	        else if (stage == 2) {
				telemetry.addData("Status", "Shooting");
		        sleep(500);
		        stage = 3;
		        /*shooterMotor.setTargetPosition(shooterPos);
		        if (isRightPos(shooterMotor, 100)) {
			        stage = 3;
			        shooterMotor.setTargetPosition(0);
		        }*/
	        }
	        else if (stage == 3) {
		        telemetry.addData("Status", "Driving to hit ball");
		        driveDistance(leftFrontMotor, 1.0, ballDistance - shootingDistance);
		        driveDistance(rightFrontMotor, 1.0, ballDistance - shootingDistance);
		        if (isRightPos(leftFrontMotor, 100) && isRightPos(rightFrontMotor, 100) && isRightPos(leftBackMotor, 100) && isRightPos(rightBackMotor, 100)) {
			        stage = 4;
			        resetDriveMotors();
		        }
	        }
	        else if (stage == 4) {
		        telemetry.addData("Status", "Turning 90 degrees");
		        driveDistance(leftFrontMotor, 1.0, (16/2)*Math.PI/4);
		        driveDistance(rightFrontMotor, 1.0, 0);
		        if (isRightPos(leftFrontMotor, 100) && isRightPos(rightFrontMotor, 100) && isRightPos(leftBackMotor, 100) && isRightPos(rightBackMotor, 100)) {
			        stage = -1;
			        resetDriveMotors();
		        }
	        }
	        rightBackMotor.setTargetPosition(rightFrontMotor.getCurrentPosition()); //set the motors as followers
	        leftBackMotor.setTargetPosition(leftFrontMotor.getCurrentPosition());
	        telemetry.addData("Encoders", "LF%d RF%d LB%d RB%d", leftFrontMotor.getCurrentPosition(), rightFrontMotor.getCurrentPosition(), leftBackMotor.getCurrentPosition(), rightBackMotor.getCurrentPosition());
	        telemetry.update();
        }


    }
    public void driveDistance(DcMotor motor, double power, double distance) { //drives forward distance in inches
        int pos = (int) Math.round(((distance*mmPerInch) / (wheelDiameter*mmPerInch*Math.PI))*ticksPerRotation);
        motor.setTargetPosition(pos);
        motor.setPower(power);
    }
	public boolean isRightPos(DcMotor motor, int tolerance) {
		if (Math.abs(motor.getTargetPosition()-motor.getCurrentPosition())  < tolerance) {
			return true;
		}
		else {
			return false;
		}
	}
	public void resetDriveMotors() {
		leftFrontMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
		rightFrontMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
		leftBackMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
		rightBackMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
		leftFrontMotor.setMode(RunMode.RUN_TO_POSITION);
		rightFrontMotor.setMode(RunMode.RUN_TO_POSITION);
		leftBackMotor.setMode(RunMode.RUN_TO_POSITION);
		rightBackMotor.setMode(RunMode.RUN_TO_POSITION);
	}
}
