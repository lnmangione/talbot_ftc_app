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

/*
* This is a test class for the TalBot Autonomous
*/

public class TalBotAutoClimberRed extends TalBotAutoClimber{

	String team = "red";

	/**
	 * Constructor
	 */
	public TalBotAutoClimberRed() {

	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init() {
		super.init();

	}

	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {
		super.loop();

	}

	@Override
	public void runPhase4() {
		int pauseIts = 60;
		int turnIts = 170;
		int driveIts = 200;
		int testIts = 0;

		//at start, turn right ~45 degrees
		if(phaseIterations == (testIts += pauseIts)){
			turnRightAll(.25);
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
			turnLeftAll(.25);
		}
		else if (phaseIterations == (testIts += turnIts)){
			driveForward(0);
		}
		else if (phaseIterations == (testIts + pauseIts)){
			newPhase(5);
		}

	}

@Override
	public void runPhase6(){
		if(getColor().equals("white")){
			seekingIterations = 0;
			turnLeftAll(.15);
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

}
