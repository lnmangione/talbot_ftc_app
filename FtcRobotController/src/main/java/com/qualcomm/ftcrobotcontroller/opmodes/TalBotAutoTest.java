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
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
/*
* This is a test class for the TalBot Autonomous
*/

public class TalBotAutoTest extends TalBotOpMode {

	int numIterations;

	//some math for using encoders to drive set distance
	final static int ENCODER_CPR = 1120; // CPR is counts per revolution
	final static int WHEEL_RADIUS = 2; // in inches
	final static double ROTATIONAL_DISTANCE = 2 * Math.PI * WHEEL_RADIUS; // aka distance travelled per one wheel rotation

	double distanceToMove = 36; //in inches
	double desiredCounts = (distanceToMove/ROTATIONAL_DISTANCE) * ENCODER_CPR; //number of encoder counts to move specified distance
	boolean isDistanceTravelled = false;

	/**
	 * Constructor
	 */
	public TalBotAutoTest() {

	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init() {
		super.init();

		//setting up encoders
		motorDrive_RB.setMode(DcMotorController.RunMode.RESET_ENCODERS);
		motorDrive_LB.setMode(DcMotorController.RunMode.RESET_ENCODERS);

		numIterations = 0;

	}

	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {

		boolean  allControllersReadMode = motorDrive_RB.getController().getMotorControllerDeviceMode().equals(DcMotorController.DeviceMode.READ_ONLY) &&
				motorDrive_LB.getController().getMotorControllerDeviceMode().equals(DcMotorController.DeviceMode.READ_ONLY);
		boolean  allControllersWriteMode = motorDrive_RB.getController().getMotorControllerDeviceMode().equals(DcMotorController.DeviceMode.WRITE_ONLY) &&
				motorDrive_LB.getController().getMotorControllerDeviceMode().equals(DcMotorController.DeviceMode.WRITE_ONLY);

		if (numIterations == 0){
			motorDrive_RB.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
			motorDrive_LB.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
		}

		//after 1 sec, set motors to drive forward
		if (numIterations == 60){
			driveForward(.45);
		}
		//set controllers to read_only after starting drive forward
		if (numIterations == 65){
			motorDrive_RB.getController().setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
			motorDrive_LB.getController().setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);

		}


		//if readonly mode, test if robot has driven desired distance
		if (allControllersReadMode) {
			int avgCounts = (Math.abs(motorDrive_RB.getCurrentPosition()) + Math.abs(motorDrive_LB.getCurrentPosition()))/2;
			if (avgCounts > desiredCounts){
				motorDrive_RB.getController().setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
				motorDrive_LB.getController().setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
				isDistanceTravelled = true;

			}
		}

		//if distance has been travelled and able to stop robot, stop it
		if (isDistanceTravelled && allControllersWriteMode){
			resetMotors();
		}


		numIterations ++;


		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */

		telemetry.addData("Text", "*** Robot Data***");
		telemetry.addData("RGB Reading", "RGB: (" + colorSensor.red() + ", " + colorSensor.green() + ", " + colorSensor.blue() + ")");
		if (motorDrive_LB.getController().getMotorControllerDeviceMode() == DcMotorController.DeviceMode.READ_ONLY
		&& motorDrive_RB.getController().getMotorControllerDeviceMode() == DcMotorController.DeviceMode.READ_ONLY){
			telemetry.addData("left encdr pos",  "left  encdr pos: " + motorDrive_LB.getCurrentPosition());
			telemetry.addData("right encdr pos",  "right  encdr pos: " + motorDrive_RB.getCurrentPosition());

		}

	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {

	}

	void driveForward(double power){
		// write the values to the motors
		motorDrive_RF.setPower(-power);
		motorDrive_RB.setPower(-power);
		motorDrive_LF.setPower(power);
		motorDrive_LB.setPower(power);
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

		trigSave.setPosition(0.5);

	}

	void checkGamepad2(){
		//Lift motors
		//Rotation
		if (gamepad2.a){
			//rotate lift upward
			motorLift_R.setPower(-0.25);
			motorLift_L.setPower(0.25);

		}
		else if (gamepad2.b){
			//rotate lift downward
			motorLift_R.setPower(0.25);
			motorLift_L.setPower(-0.25);

		}

		//Extension
		if (gamepad2.x){
			//retract lift
			motorPull1.setPower(1.0);
			motorPull2.setPower(-1.0);


		}
		else if(gamepad2.y){
			//extend lift
			motorPull1.setPower(-1.0);
			motorPull2.setPower(1.0);

		}

		//Servo arm position
		//Note - upward for right servo is 1.0
		// upward for left servo is 0

		if (gamepad2.left_bumper){
			armPositionR -= .01;
			armSpeedL = 1.0;
		}
		else if (gamepad2.right_bumper){
			armPositionR += .01;
			armSpeedL = 0.0;
		}

		armPositionR = Range.clip(armPositionR, 0.52, 0.96);
		armR.setPosition(armPositionR);
		armL.setPosition(armSpeedL);

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

		//save servo using right and left servos
		//left trigger brings save down
		if (gamepad2.left_trigger > 0.6){
			trigSave.setPosition(0.9);
		}
		//right trigger brings save up
		else if (gamepad2.right_trigger > 0.6){
			trigSave.setPosition(0.1);
		}

	}


    /*
	double scaleInput(double dVal)  {
		double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
				0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };
		
		// get the corresponding index for the scaleInput array.
		int index = (int) (dVal * 16.0);
		
		// index should be positive.
		if (index < 0) {
			index = -index;
		}

		// index cannot exceed size of array minus 1.
		if (index > 16) {
			index = 16;
		}

		// get value from the array.
		double dScale = 0.0;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}

		//.8 speed test
		//dScale *= .8;

		// return scaled value.
		return dScale;
	}
    */
}
