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

import com.qualcomm.robotcore.hardware.DcMotorController;
/*
* This is a test class for the TalBot Autonomous
*/

public class TalBotAutoClimber extends TalBotAutoOpMode {

	String team = "blue";

	//different phases of autonomous numbered
	int phase = 1;
	int phaseIterations = 0;
	int seekingIterations = 0;

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
	public TalBotAutoClimber() {

	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init() {
		super.init();

		//setting up motors to use encoders
		motorDrive_RB.setMode(DcMotorController.RunMode.RESET_ENCODERS);
		motorDrive_LB.setMode(DcMotorController.RunMode.RESET_ENCODERS);

	}

	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {
		if (this.time < 28.5){
			runPhase();
		}


		//turn on encoders
		//drive forward
		//after certain distance - drive sequence to push away debri in front
		//continue to drive forward until detect line
		//once detect line, follow it using line following algorithm
		//after encoders determine that have been following line long enough, position robot straight with line
		//dump climbers

		telemetry.addData("Text", "*** Robot Data***");
		telemetry.addData("Time", "t = " + time);
		telemetry.addData("RGB Reading", "RGB: (" + colorSensor.red() + ", " + colorSensor.green() + ", " + colorSensor.blue() + ")");
		if (motorDrive_LB.getController().getMotorControllerDeviceMode() == DcMotorController.DeviceMode.READ_ONLY
		&& motorDrive_RB.getController().getMotorControllerDeviceMode() == DcMotorController.DeviceMode.READ_ONLY){
			telemetry.addData("left encdr pos",  "left  encdr pos: " + motorDrive_LB.getCurrentPosition());
			telemetry.addData("right encdr pos",  "right  encdr pos: " + motorDrive_RB.getCurrentPosition());

		}

	}

	public void runPhase(){
		if (phase == 1){
			//wait a few seconds before moving
			runPhase1();
		}
		else if (phase == 2){
			//set the motors to drive forward
			runPhase2();
		}
		else if (phase == 3){
			runPhase3();
		}
		else if (phase == 4){
			runPhase4();
		}
		else if (phase == 5){
			runPhase5();
		}
		else if (phase == 6){
			runPhase6();
		}
		else if (phase==7){
			runPhase7();
		}
		else if (phase==8){
			runPhase8();
		}
		else if (phase==9){
			runPhase9();
		}
		else if (phase==10){
			runPhase10();
		}
		phaseIterations ++;

	}

	//waits 5 seconds before setting motors to drive forward
	public void runPhase1(){
		if (phaseIterations == 0){
			motorDrive_RB.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
			motorDrive_LB.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
		}

		if (this.time > 2){
			driveForward(.4);

			newPhase(2);
		}
	}


	public void runPhase2(){
		if (time < 13) {
			//if hit the colored line, next phase
			if (getColor().equals(team)) {
				newPhase(3);
			}
		}

	}

	public void runPhase3(){
		if (phaseIterations == 0){
			driveForward(0);
		}
		else if(phaseIterations == 10){
			driveForward(-.15);
		}
		else if(getColor().equals(team)){
			driveForward(0);
			newPhase(4);
		}


	}

	public void runPhase4() {
		int pauseIts = 60;
		int turnIts = 170;
		int driveIts = 200;
		int testIts = 0;

		//at start, turn left ~45 degrees
		if(phaseIterations == (testIts += pauseIts)){
			turnLeftAll(.25);
		}
		//then drive forward ~25 inches
		else if(phaseIterations == (testIts += turnIts)){
			driveForward(.32);
		}
		//stop
		else if(phaseIterations == (testIts += driveIts)){
			driveForward(0);

		}
		else if(phaseIterations == (testIts += pauseIts)){
			driveForward(-.32);
		}
		else if(phaseIterations == (testIts += driveIts)){
			driveForward(0);
		}
		else if(phaseIterations == (testIts += pauseIts)){
			turnRightAll(.25);
		}
		else if (phaseIterations == (testIts += turnIts)){
			driveForward(0);
		}
		else if (phaseIterations == (testIts + pauseIts)){
			newPhase(5);
		}

	}


	public void runPhase5(){
		if (phaseIterations == 0){
			driveForward(0.15);
		}

		if(getColor().equals("white")){
			driveForward(0);
			newPhase(6);
		}

	}

	public void runPhase6(){
		if(getColor().equals("white")){
			seekingIterations = 0;
			turnRightAll(.15);
		}
		else if (phaseIterations < 150){
			if(seekingIterations < 30){
				driveForward(-.15);
			}
			else if (seekingIterations < 60){
				driveForward(.15);
			}
			seekingIterations++;
		}
		else if(phaseIterations >= 415){
			driveForward(0);
			newPhase(7);
		}

	}
	public void runPhase7(){
		int topKek=200;
		if(phaseIterations==0)
		{
			extendLift(.5);
		}
		else if(phaseIterations==topKek)
		{
			extendLift(0);
			newPhase(8);
		}
	}

	public void runPhase8(){
		int topKek=300;
		if (phaseIterations==0)
		{
			dumpClimbers(.25);
		}
		else if (phaseIterations==topKek){
			dumpClimbers(0);
			newPhase(9);
		}
	}
	public void runPhase9(){
		int topKek=200;
		if(phaseIterations==0)
		{
			extendLift(-.5);
		}
		else if(phaseIterations==topKek)
		{
			extendLift(0);
			newPhase(10);
		}
	}
	public void runPhase10(){
		int topKek=300;
		if (phaseIterations==0)
		{
			dumpClimbers(-.25);
		}
		else if (phaseIterations==topKek){
			dumpClimbers(0);

		}
	}

	public void newPhase(int p){
		phase = p;
		phaseIterations = -1;
		seekingIterations = 0;
	}


	/*
	 * Code to run when the op mode is first disabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {

	}

	public void driveForward(double power){
		// write the values to the motors
		motorDrive_RF.setPower(-power);
		motorDrive_RB.setPower(-power);
		motorDrive_LF.setPower(power);
		motorDrive_LB.setPower(power);
	}

	public void turnLeftAll(double power) {
		motorDrive_RF.setPower(-power);
		motorDrive_RB.setPower(-power);
		motorDrive_LF.setPower(-power);
		motorDrive_LB.setPower(-power);

	}

	public void turnRightAll(double power) {
		motorDrive_RF.setPower(power);
		motorDrive_RB.setPower(power);
		motorDrive_LF.setPower(power);
		motorDrive_LB.setPower(power);

	}
	public void extendLift(double power){
		motorPull1.setPower(-power);
		motorPull2.setPower(power);

	}
	public void dumpClimbers (double power){
		motorLift_L.setPower(power);
		motorLift_R.setPower(-power);

	}
}
