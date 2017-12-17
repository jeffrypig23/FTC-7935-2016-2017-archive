package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Ethan Hunter on 7/12/17.
 * FTC 6128 | 7935
 * FRC 1595
 */

/*
This programm allows for control of two or more robots consisting of an intake, shooter and two drive wheels each.
 */

@TeleOp(name = "NewTankDrive", group = "Text")  // @Autonomous(...) is the other common choice
// @Disabled //No @Disables for you muhahahaha! >:3

public class NewTankDrive extends LinearOpMode {

    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftBackMotor = null;
    DcMotor rightBackMotor = null;
    DcMotor shooterMotor = null;
    DcMotor intakeMotor = null; //tetrix
    Servo upServo = null;
    Servo buttonServo = null;
    double servoUpPos = 0.5;
    double servoDownPos = 0.1;
    int shooterUpPos = 1000;

    boolean isShooting = false;

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

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(RunMode.RUN_USING_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(RunMode.RUN_USING_ENCODER);

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        ElapsedTime runtime = new ElapsedTime();
        telemetry.addData("INITIALIZATION", "DONE");
        telemetry.update();
        waitForStart();  //Waits for the match to start
        runtime.reset();
        while (opModeIsActive()) {
            leftFrontMotor.setPower(gamepad1.left_stick_y);
            rightFrontMotor.setPower(gamepad1.right_stick_y);
            leftBackMotor.setPower(gamepad1.left_stick_y);
            //leftBackMotor.setTargetPosition(leftFrontMotor.getCurrentPosition());
            rightBackMotor.setPower(gamepad1.right_stick_y);
            //rightBackMotor.setTargetPosition(rightFrontMotor.getCurrentPosition());
            //shooterMotor.setPower(1.0);
            /*if (gamepad1.b) {
                buttonServo.setPosition();
            }
            else {
                buttonServo.setPosition();
            }*/
            /*
            if (gamepad1.a && shooterMotor.getTargetPosition()==0 && Math.abs(shooterMotor.getCurrentPosition())<15) {
                shooterMotor.setTargetPosition(shooterUpPos);
            }

            else if (shooterMotor.getTargetPosition()==shooterUpPos && Math.abs(shooterUpPos-shooterMotor.getCurrentPosition())<15) {
                shooterMotor.setTargetPosition(0);
            }
            */

            if (gamepad1.a && !isShooting && (shooterMotor.getCurrentPosition() <= 10)) {
                shooterMotor.setPower(1.0);
                isShooting = true;
            }

            if (shooterMotor.getCurrentPosition() >= 1000 && isShooting) {
                shooterMotor.setPower(-1.0);
            } else if (shooterMotor.getCurrentPosition() <= 10 && isShooting) {
                shooterMotor.setPower(0.0);
                isShooting = false;
            }

            if (gamepad1.y) {
                upServo.setPosition(servoUpPos);
            } else if (Math.abs(servoUpPos - upServo.getPosition()) < 5) {
                upServo.setPosition(servoDownPos);
            }
            if (gamepad1.right_bumper) {
                intakeMotor.setPower(1.0);
            } else if (gamepad1.left_bumper) {
                intakeMotor.setPower(-1.0);
            } else {
                intakeMotor.setPower(0.0);
            }
            telemetry.addData("Encoders", "LF%d RF%d LB%d RB%d", leftFrontMotor.getCurrentPosition(), rightFrontMotor.getCurrentPosition(), leftBackMotor.getCurrentPosition(), rightBackMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}

