package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Stephen Ogden on 1/26/2017.
 */

@TeleOp(name="Spinney Thingy test code", group="Test")  // @Autonomous(...) is the other common choice
@Disabled //No @Disables for you muhahahaha! >:3

public class TestTeleopCode extends LinearOpMode {

    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftBackMotor = null;
    DcMotor rightBackMotor = null;
    DcMotor shooterMotor = null; //This is the new spinney motor! :)

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("INITIALIZATION", "BEGINNING");
        telemetry.addData("Config", "SMotor" + "LFMotor" + "RFMotor" + "LBMotor" + "RBMotor");
        telemetry.update();

        leftFrontMotor = hardwareMap.dcMotor.get("LFMotor");
        rightFrontMotor = hardwareMap.dcMotor.get("RFMotor");
        leftBackMotor = hardwareMap.dcMotor.get("LBMotor");
        rightBackMotor = hardwareMap.dcMotor.get("RBMotor");
        shooterMotor = hardwareMap.dcMotor.get("SMotor");

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Should act differently

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        ElapsedTime runtime = new ElapsedTime();
        telemetry.addData("INITIALIZATION", "DONE");
        telemetry.update();

        waitForStart();  //Waits for the match to start
        runtime.reset();

        leftFrontMotor.setPower(gamepad1.left_stick_y);
        rightFrontMotor.setPower(gamepad1.right_stick_y);
        leftBackMotor.setPower(gamepad1.left_stick_y);
        rightBackMotor.setPower(gamepad1.right_stick_y);

        if (gamepad1.left_trigger >= .25) {
            shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            shooterMotor.setPower(1.0);
        } else if (gamepad1.right_trigger >= .25) {
            shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            shooterMotor.setPower(1.0);
        } else {
            shooterMotor.setPower(0.0);
        }

        telemetry.addData(" ", "The shooter and intake have been disabled in this code!");
        telemetry.addData(" ", " ");
        telemetry.addData("Encoders", "LF%d RF%d LB%d RB%d", leftFrontMotor.getCurrentPosition(), rightFrontMotor.getCurrentPosition(), leftBackMotor.getCurrentPosition(), rightBackMotor.getCurrentPosition());
        telemetry.update();

    }

}
