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
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.mmPerInch;

/*
In autonomous what we want to do is have one robot go with the phone and shoot two balls into our goals
the other robot shoudl
 */


@Autonomous(name="Short Drive Auto", group="Linear Opmode")  // @Autonomous(...) is the other common choice
@Disabled

public class ShortDriveAuto extends LinearOpMode {

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
	    telemetry.addData("Config", "SMotor" + "IMotor" + "LFMotor" + "RFMotor" + "LBMotor" + "RBMotor");
	    telemetry.update();

	    leftFrontMotor = hardwareMap.dcMotor.get("LFMotor");
	    rightFrontMotor = hardwareMap.dcMotor.get("RFMotor");
	    leftBackMotor = hardwareMap.dcMotor.get("LBMotor");
	    rightBackMotor = hardwareMap.dcMotor.get("RBMotor");
	    shooterMotor = hardwareMap.dcMotor.get("SMotor");
	    intakeMotor = hardwareMap.dcMotor.get("IMotor");
	    upServo = hardwareMap.servo.get("UServo");
	    //buttonServo = hardwareMap.servo.get("BServo");

	    leftFrontMotor.setMode(RunMode.RUN_USING_ENCODER);
	    rightFrontMotor.setMode(RunMode.RUN_USING_ENCODER);
	    leftBackMotor.setMode(RunMode.RUN_USING_ENCODER);
	    rightBackMotor.setMode(RunMode.RUN_USING_ENCODER);
	    shooterMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
	    shooterMotor.setMode(RunMode.RUN_TO_POSITION);
	    intakeMotor.setMode(RunMode.RUN_USING_ENCODER);

	    leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	    rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	    leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	    rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	    shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	    intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

	    leftFrontMotor.setDirection(Direction.FORWARD);
	    rightFrontMotor.setDirection(Direction.REVERSE);
	    leftBackMotor.setDirection(Direction.REVERSE);
	    rightBackMotor.setDirection(Direction.FORWARD);

	    double robotPos=0;
	    int shooterUpPos = 1000;
        ElapsedTime runtime = new ElapsedTime();
        telemetry.addData("INITIALIZATION", "DONE");
        telemetry.update();
        waitForStart();  //Waits for the match to start
        runtime.reset();
        while (opModeIsActive()) {
	        robotPos=Math.PI*wheelDiameter*(rightBackMotor.getCurrentPosition()+leftBackMotor.getCurrentPosition()+rightFrontMotor.getCurrentPosition()+leftFrontMotor.getCurrentPosition())/(4*ticksPerRotation);
	        shooterMotor.setPower(1.0);
	        if (stage == -1) {
		        telemetry.addData("Status", "Program Ended for testing purposes");
	        }
	        else if (stage == 0) {
		        resetDriveMotors();
		        stage = 2;
	        }
	        else if (stage == 1) {
		        telemetry.addData("Status", "Shooting");
		        shooterMotor.setTargetPosition(shooterUpPos);
		        sleep(2000);
		        shooterMotor.setTargetPosition(0);
		        stage=2;
		        runtime.reset();
	        }
	        else if (stage == 2) {
		        telemetry.addData("Status", "Driving");
		        rightBackMotor.setPower(1.0);
		        rightFrontMotor.setPower(1.0);
		        leftBackMotor.setPower(1.0);
				leftFrontMotor.setPower(1.0);
		        if(runtime.milliseconds()>1800) //drive 50 inches
		        {
			        rightBackMotor.setPower(0.0);
			        rightFrontMotor.setPower(0.0);
			        leftBackMotor.setPower(0.0);
			        leftFrontMotor.setPower(0.0);
			        stage=3;
		        }
	        }
	        else if (stage ==3) {
		        telemetry.addData("Status", "Done");
		        resetDriveMotors();
	        }
	        /*BEST METHOD RIGHT HERE*/telemetry.addData("Average All", Math.PI*wheelDiameter*(rightBackMotor.getCurrentPosition()+leftBackMotor.getCurrentPosition()+rightFrontMotor.getCurrentPosition()+leftFrontMotor.getCurrentPosition())/(4*ticksPerRotation));
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
		leftFrontMotor.setMode(RunMode.RUN_USING_ENCODER);
		rightFrontMotor.setMode(RunMode.RUN_USING_ENCODER);
		leftBackMotor.setMode(RunMode.RUN_USING_ENCODER);
		rightBackMotor.setMode(RunMode.RUN_USING_ENCODER);
	}
}
