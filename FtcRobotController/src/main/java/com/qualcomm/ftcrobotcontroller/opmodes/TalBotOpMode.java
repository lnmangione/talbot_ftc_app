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
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class TalBotOpMode extends OpMode {

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
	double armSpeedL;

	Servo trigR;
	double trigPositionR;
	Servo trigL;
	double trigPositionL;

	final static double TRIG_R_UP = 1.0;
	final static double TRIG_R_DOWN = 0.485;
	final static double TRIG_L_UP = 0.0;
	final static double TRIG_L_DOWN = 0.45;

	Servo trigSave;

	ColorSensor colorSensor;

	/**
	 * Constructor
	 */
	public TalBotOpMode() {

	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init() {
		motorDrive_RF = hardwareMap.dcMotor.get("motorDrive_RF");
		motorDrive_RB = hardwareMap.dcMotor.get("motorDrive_RB");
		motorDrive_LF = hardwareMap.dcMotor.get("motorDrive_LF");
		motorDrive_LB = hardwareMap.dcMotor.get("motorDrive_LB");

		motorLift_R = hardwareMap.dcMotor.get("motorLift_R");
		motorLift_L = hardwareMap.dcMotor.get("motorLift_L");
		motorPull1 = hardwareMap.dcMotor.get("motorPull1");
		motorPull2 = hardwareMap.dcMotor.get("motorPull2");

		armR = hardwareMap.servo.get("servoArmR");
		armPositionR = 0.5;
		armR.setPosition(armPositionR);
		armL = hardwareMap.servo.get("servoArmL");
		armSpeedL = 0.5;
		armL.setPosition(armSpeedL);


		trigR = hardwareMap.servo.get("servoTrigR");
		trigPositionR = TRIG_R_UP;
		trigR.setPosition(trigPositionR);

		trigL = hardwareMap.servo.get("servoTrigL");
		trigPositionL = TRIG_L_UP;
		trigL.setPosition(trigPositionL);

		trigSave = hardwareMap.servo.get("servoSave");
		trigSave.setPosition(0.5);

		colorSensor = hardwareMap.colorSensor.get("colorSensor");

	}

	/*
	 * This method will be called repeatedly in a loop
	 */
	@Override
	public void loop() {

	}

	/*
	 * Code to run when the op mode is first disabled goes here
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

}
