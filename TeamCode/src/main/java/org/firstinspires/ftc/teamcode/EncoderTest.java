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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
/*
This programm allows for control of two or more robots consisting of an intake, shooter and two drive wheels each.
 */

@TeleOp(name="Encoder Position Test", group="Debug")  // @Autonomous(...) is the other common choice
@Disabled //No @Disables for you muhahahaha! >:3

public class EncoderTest extends LinearOpMode {

	DcMotor motor = null;
	int pos = 0;
	double pow = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {


        telemetry.addData("INITIALIZATION", "BEGINNING");
        telemetry.update();

        motor = hardwareMap.dcMotor.get("Motor");
	    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("INITIALIZATION", "DONE");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
	            motor.setTargetPosition(pos);
            }
	        if (gamepad1.right_bumper) {
		        pos++;
	        }
	        if (gamepad1.left_bumper) {
		        pos--;
	        }
	        motor.setPower(pow);
	        if (gamepad1.y) {
		        if (gamepad1.right_bumper) {
			        pow=pow+0.05;
		        }
		        if (gamepad1.left_bumper) {
			        pow=pow-0.05;
		        }
	        }
	        telemetry.addData("Planned Position", pos);
	        telemetry.addData("Current Position", motor.getCurrentPosition());
	        telemetry.addData("Power", pow);
	        telemetry.addData("", "Use bumpers to adjust encoder position and A button to bring the motor to that position");
	        telemetry.addData("", "The motor power can be adjusted with bumpers while holding down Y");
	        telemetry.update();
	        sleep(20);
        }
    }
}
