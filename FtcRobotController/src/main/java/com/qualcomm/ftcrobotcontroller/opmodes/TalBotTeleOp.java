/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class TalBotTeleOp extends OpMode {

	DcMotor motorDrive_RF;
	DcMotor motorDrive_LF;
	DcMotor motorDrive_RB;
	DcMotor motorDrive_LB;

	DcMotor motorLift_R;
	DcMotor motorLift_L;
	DcMotor motorPull1;
	DcMotor motorPull2;

	Servo armR;
	double armPositionR;
	Servo armL;
	double armPositionL;

	Servo trigR;
	double trigPositionR;
	Servo trigL;
	double trigPositionL;

	final static double TRIG_R_UP = 1.0;
	final static double TRIG_R_DOWN = 0.485;
	final static double TRIG_L_UP = 0.0;
	final static double TRIG_L_DOWN = 0.45;

	ColorSensor colorSensor;


	/**
	 * Constructor
	 */
	public TalBotTeleOp() {

	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init() {


		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */
		
		/*
		 * For the demo Tetrix K9 bot we assume the following,
		 *   There are two motors "motor_1" and "motor_2"
		 *   "motor_1" is on the right side of the bot.
		 *   "motor_2" is on the left side of the bot and reversed.
		 *
		 */
		motorDrive_RF = hardwareMap.dcMotor.get("motorDrive_RF");
		motorDrive_RB = hardwareMap.dcMotor.get("motorDrive_RB");
		motorDrive_LF = hardwareMap.dcMotor.get("motorDrive_LF");
		motorDrive_LB = hardwareMap.dcMotor.get("motorDrive_LB");

		motorLift_R = hardwareMap.dcMotor.get("motorLift_R");
		motorLift_L = hardwareMap.dcMotor.get("motorLift_L");
		motorPull1 = hardwareMap.dcMotor.get("motorPull1");
		motorPull2 = hardwareMap.dcMotor.get("motorPull2");
		motorPull1.setDirection(DcMotor.Direction.REVERSE);

		armR = hardwareMap.servo.get("servoArmR");
		armPositionR = 0.0;
		armR.setPosition(armPositionR);
		armL = hardwareMap.servo.get("servoArmL");
		armPositionL = 0.93;
		armL.setPosition(armPositionL);


		trigR = hardwareMap.servo.get("servoTrigR");
		trigPositionR = TRIG_R_UP;
		trigR.setPosition(trigPositionR);

		trigL = hardwareMap.servo.get("servoTrigL");
		trigPositionL = TRIG_L_UP;
		trigL.setPosition(trigPositionL);

		colorSensor = hardwareMap.colorSensor.get("colorSensor");

	}

	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {
		//reset power of all motors
		resetMotors();

		checkGamepad1();
		checkGamepad2();

		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
		telemetry.addData("Text", "*** Robot Data***");
		telemetry.addData("RGB Reading", "RGB: (" + colorSensor.red() + ", " + colorSensor.green() + ", " + colorSensor.blue() + ")");
		String whiteMessage;
		if (colorSensor.red() > 170 && colorSensor.green() > 170 && colorSensor.blue() > 170){
			whiteMessage = "WHITE DETECTED";
		}
		else{
			whiteMessage = "NO white detected";
		}
		telemetry.addData("White Detected", whiteMessage);


	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {

	}

	void resetMotors(){
		motorDrive_RF.setPower(0.0);
		motorDrive_RB.setPower(0.0);
		motorDrive_LF.setPower(0.0);
		motorDrive_LB.setPower(0.0);

		motorLift_R.setPower(0.0);
		motorLift_L.setPower(0.0);
		motorPull1.setPower(0.0);
		motorPull2.setPower(0.0);

	}
    void checkGamepad1(){
		/*
		 * Gamepad 1
		 *
		 * Gamepad 1 controls the motors via the left stick, and it controls the
		 * wrist/claw via the a,b, x, y buttons
		 */


		// throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
		// 1 is full down
		// direction: left_stick_x ranges from -1 to 1, where -1 is full left
		// and 1 is full right
		float direction = gamepad1.left_stick_y;
		float throttle = gamepad1.left_stick_x;
		float right = throttle + direction;
		float left = throttle - direction;

		// clip the right/left values so that the values never exceed +/- 1
		right = Range.clip(right, -1, 1);
		left = Range.clip(left, -1, 1);

		// scale the joystick value to make it easier to control
		// the robot more precisely at slower speeds.
		//right = (float)scaleInput(right);
		//left =  (float)scaleInput(left);

		// write the values to the motors
		motorDrive_RF.setPower(right);
		motorDrive_RB.setPower(right);
		motorDrive_LF.setPower(left);
		motorDrive_LB.setPower(left);



		//Lift motors
		if (gamepad1.x){
			//rotate lift upward
			motorLift_R.setPower(-.2);
			motorLift_L.setPower(.2);

		}
		else if (gamepad1.b){
			//rotate lift downward
			motorLift_R.setPower(.2);
			motorLift_L.setPower(-.2);

		}

		//Extension
		if (gamepad1.a){
			//retract lift
			motorPull1.setPower(1.0);
			motorPull2.setPower(-1.0);


		}
		else if(gamepad1.y){
			//extend lift
			motorPull1.setPower(-1.0);
			motorPull2.setPower(1.0);

		}

		//Servo arm position
		//Note - upward for right servo is 0.5 (down is 0)
		// upward for left servo is 0
		// right bumper brings both arms down (parallel with ground)
		// left bumper makes arms face upwards

		if (gamepad1.left_bumper){
			armPositionR += .01;
			armPositionL -= 0.01;
		}
		else if (gamepad1.right_bumper){
			armPositionR -= .01;
			armPositionL += 0.01;
		}

		armPositionR = Range.clip(armPositionR, 0.0, 0.45);
		armR.setPosition(armPositionR);
		armPositionL = Range.clip(armPositionL, 0.5, 0.93);
		armL.setPosition(armPositionL);

		//Trigger servos positions
		//these are controlled by the d-pad
		//Note - upward for right servo is 1.0
		// upward for left servo is 0

		//d-pad up
		if (gamepad1.dpad_up){
			trigPositionR = TRIG_R_UP;
			trigPositionL = TRIG_L_UP;
		}
		//d-pad down - both servos down
		else if(gamepad1.dpad_down){
			trigPositionR = TRIG_R_DOWN;
			trigPositionL = TRIG_L_DOWN;
		}
		//d-pad right - right servo down, left up
		else if (gamepad1.dpad_right){
			trigPositionR = TRIG_R_DOWN;
			trigPositionL = TRIG_L_UP;
		}
		//d-pad left - left servo down, right up
		else if (gamepad1.dpad_left){
			trigPositionR = TRIG_R_UP;
			trigPositionL = TRIG_L_DOWN;
		}


		trigPositionR = Range.clip(trigPositionR, 0, 1);
		trigR.setPosition(trigPositionR);
		trigPositionL = Range.clip(trigPositionL, 0, 1);
		trigL.setPosition(trigPositionL);


	}
	void checkGamepad2(){
		//Lift motors
		//Rotation
		//slow rotation
		if (gamepad2.y){
			//rotate lift upward
			motorLift_R.setPower(-0.2);
			motorLift_L.setPower(0.2);

		}
		else if (gamepad2.a){
			//rotate lift downward
			motorLift_R.setPower(0.2);
			motorLift_L.setPower(-0.2);

		}

		//fast rotation
		if (gamepad2.left_stick_y>.3){
			//rotate lift upward
			motorLift_R.setPower(-1);
			motorLift_L.setPower(1);

		}
		else if (gamepad2.left_stick_y<-.3){
			//rotate lift downward
			motorLift_R.setPower(1);
			motorLift_L.setPower(-1);

		}


		//Extension
		if (gamepad2.left_trigger>.3){
			//retract lower lift
			motorPull1.setPower(1.0);


		}
		else if(gamepad2.left_bumper){
			//extend lower lift
			motorPull1.setPower(-1.0);


		}

		if (gamepad2.right_trigger>.3){
			//retract upper lift
			motorPull2.setPower(-1.0);


		}
		else if(gamepad2.right_bumper){
			//extend upper lift
			motorPull2.setPower(1.0);

		}


		//Trigger servos positions
		//these are controlled by the d-pad
		//Note - upward for right servo is 1.0
		// upward for left servo is 0

		//d-pad up - both servos up
		if (gamepad2.dpad_up){
			trigPositionR = TRIG_R_UP;
			trigPositionL = TRIG_L_UP;
		}
		//d-pad down - both servos down
		else if(gamepad2.dpad_down){
			trigPositionR = TRIG_R_DOWN;
			trigPositionL = TRIG_L_DOWN;
		}
		//d-pad right - right servo down, left up
		else if (gamepad2.dpad_right){
			trigPositionR = TRIG_R_DOWN;
			trigPositionL = TRIG_L_UP;
		}
		//d-pad left - left servo down, right up
		else if (gamepad2.dpad_left){
			trigPositionR = TRIG_R_UP;
			trigPositionL = TRIG_L_DOWN;
		}


		trigPositionR = Range.clip(trigPositionR, 0, 1);
		trigR.setPosition(trigPositionR);
		trigPositionL = Range.clip(trigPositionL, 0, 1);
		trigL.setPosition(trigPositionL);


	}

}
